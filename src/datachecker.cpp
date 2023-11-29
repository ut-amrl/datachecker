// MIT License

// Copyright (c) 2022 Joydeep Biswas

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//========================================================================
/*!
\file    bagfile_synopsis.cc
\brief   Generates a synopsis of a ROS bag file
\author  Joydeep Biswas, (C) 2012
*/
//========================================================================

#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <string.h>
#include <vector>
#include <deque>
#include <unordered_map>
#include <map>
#include <yaml-cpp/yaml.h>

#include "ros/ros.h"
#include "ouster_ros/PacketMsg.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"

#include "nav_msgs/Odometry.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "std_msgs/Header.h"

using std::string;
using std::vector;
using std::deque;

struct TopicInfo {
  size_t windowSize = 0;
  std::deque<double> timestamps;

  double expectedPeriod;
  double expectedPeriodStdDev;

  double stdDevThreshold;
  size_t numObservations;
  size_t numInvalidObservations;

  bool last_seq_num_initialized = false;
  uint32_t last_seq_num;
};

static std::unordered_map<std::string, TopicInfo> topicInfo;
static std::vector<std::string> topicNames;
static int total_obs = 0;
static int invalid_obs = 0;

void RegisterTopic(std::string topicName, double expectedPeriod, double expectedPeriodStdDev, int windowSize=10, double stdDevThreshold=3.0) {
  TopicInfo info;
  info.windowSize = windowSize;
  info.timestamps = std::deque<double>();
  info.expectedPeriod = expectedPeriod;
  info.expectedPeriodStdDev = expectedPeriodStdDev;
  info.stdDevThreshold = stdDevThreshold;
  info.numObservations = 0;
  info.numInvalidObservations = 0;

  topicInfo[topicName] = info;

  topicNames.push_back(topicName);
}

void AddTopicObservation(std::string topicName, const rosbag::MessageInstance& message, FILE* log_fp){
  total_obs+=1;

  int windowSize = topicInfo[topicName].windowSize;
  std::deque<double>& timestamps = topicInfo[topicName].timestamps;

  double expectedPeriod = topicInfo[topicName].expectedPeriod;
  double expectedPeriodStdDev = topicInfo[topicName].expectedPeriodStdDev;
  double stdDevThreshold = topicInfo[topicName].stdDevThreshold;

  double lowerPeriodBound = expectedPeriod - expectedPeriodStdDev * stdDevThreshold;
  double upperPeriodBound = expectedPeriod + expectedPeriodStdDev * stdDevThreshold;

  timestamps.push_back(message.getTime().toSec());

  topicInfo[topicName].numObservations+=1;

  if (timestamps.size() > (uint32_t)windowSize) {
    timestamps.pop_front();

    double avgPeriod = (timestamps.back() - timestamps.front()) / (windowSize - 1);

    if (avgPeriod < lowerPeriodBound || avgPeriod > upperPeriodBound){
      invalid_obs += 1;
      
      fprintf(log_fp,
          "Topic Rate Drop\n"
          "  time = %2.4f;\n"
          "  topic = \"%s\";\n"
          "  rate_measured = %2.4f;\n"
          "  period_measured = %2.4f;\n"
          "  expected_period = %2.4f;\n"
          "  expected_period_std_dev = %2.4f;\n"
          "  window_size = %lu;\n",
          message.getTime().toSec(),
          topicName.c_str(),
          1.0 / avgPeriod,
          avgPeriod,
          topicInfo[topicName].expectedPeriod,
          topicInfo[topicName].expectedPeriodStdDev,
          topicInfo[topicName].windowSize
          );

      topicInfo[topicName].numInvalidObservations+=1;
    }
  }
}

class MessageParserBase {
  public:
    virtual void ParseMessageInstance(const rosbag::MessageInstance& message_instance, FILE* log_fp) = 0;
};

template<typename T>
class MessageParser : public MessageParserBase {
  public:
    void ParseMessageInstance(const rosbag::MessageInstance& message_instance, FILE* log_fp){ 
      if (!message_instance.isType<T>()){
        std::cout << "Message from topic " << message_instance.getTopic() << " should have type " << message_instance.getDataType() << "\n";
        exit(-1);
      }

      boost::shared_ptr<T> msg = message_instance.instantiate<T>();
      uint32_t seq_num = msg->header.seq;
      std::string topic = message_instance.getTopic();

      if (topicInfo[topic].last_seq_num_initialized && (topicInfo[topic].last_seq_num + 1 != seq_num)){
        fprintf(log_fp,
          "Topic SeqNum Skip\n"
          "  time = %2.4f;\n"
          "  topic = \"%s\";\n"
          "  prev_seq_num = %lu;\n"
          "  current_seq_num = %lu;\n",
          message_instance.getTime().toSec(),
          topic.c_str(),
          (unsigned long)topicInfo[topic].last_seq_num,
          (unsigned long)seq_num
          );
      }

      topicInfo[topic].last_seq_num = seq_num;
      topicInfo[topic].last_seq_num_initialized = true;
    }
};

bool ParseBagFile(const string& bag_file,
                  std::map<std::string, MessageParserBase*> msg_parser_map,
                  double* total_distance,
                  uint64_t* laser_scan_msgs_ptr,
                  uint64_t* kinect_scan_msgs_ptr,
                  uint64_t* kinect_image_msgs_ptr,
                  uint64_t* stargazer_sightings_ptr,
                  double* bag_duration_ptr,
                  double* enml_distance) {
  static const double kMinDistance = 30.0;
  static const bool debug = true;
  // printf("Reading %s\n", filename);
  rosbag::Bag bag;
  try {
    bag.open(bag_file,rosbag::bagmode::Read);
  } catch(rosbag::BagException const& exception) {
    printf("Unable to read %s, reason:\n %s\n", bag_file.c_str(), exception.what());
    return false;
  }

  string synopsis_file = string(bag_file) + string(".synopsis");
  string log_file      = string(bag_file) + string(".log");

  FILE* log_fp = fopen(log_file.c_str(), "w");

  rosbag::View view(bag, rosbag::TopicQuery(topicNames));

  double bag_time_start = -1.0;
  double bag_time = 0.0;
  double distance_traversed = 0.0;
  uint64_t& laser_scan_msgs = *laser_scan_msgs_ptr;
  uint64_t& kinect_scan_msgs = *kinect_scan_msgs_ptr;
  double& bag_duration = *bag_duration_ptr;
  laser_scan_msgs = 0;
  kinect_scan_msgs = 0;
  *total_distance = 0;
  bag_duration = 0.0;
  uint32_t autonomous_steps = 0;
  uint32_t total_steps = 0;
  // bool autonomous = false;
  bool using_enml = true;

  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;

    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }

    std::string topic = message.getTopic();
    if (msg_parser_map.find(message.getDataType()) == msg_parser_map.end()){
      std::cout << "Messages of type " << message.getDataType()  << " are not registered" << "\n";
      exit(-1);
    }

    MessageParserBase* parser = msg_parser_map[message.getDataType()];
    parser->ParseMessageInstance(message, log_fp);
    
    AddTopicObservation(topic, message, log_fp);
  }

  bag_duration = bag_time - bag_time_start;
  const bool is_interesting =
      distance_traversed > kMinDistance &&
      (laser_scan_msgs > 0 || kinect_scan_msgs > 0);

  fclose(log_fp);

  FILE* fp = fopen(synopsis_file.c_str(), "w");
  // ScopedFile fp(synopsis_file.c_str(), "w");
  if (fp == NULL) {
    printf("Unable to write to %s\n", synopsis_file.c_str());
    return false;
  }

  for (const string& topicName : topicNames) {
    fprintf(fp, "topic: %s\n"
            "  num_msgs: %lu\n"
            "  num_invalid_msgs: %lu\n",
            topicName.c_str(),
            topicInfo[topicName].numObservations,
            topicInfo[topicName].numInvalidObservations);
  }
  fprintf(fp,
          "synopsis = {\n"
          "  filename = \"%s\";\n"
          "  duration = %9.1f;\n"
          "  autonomous_distance_traversed = %7.1f;\n"
          "  laser_msgs = %lu;\n"
          "  kinect_msgs = %lu;\n"
          "  kinect_images = %lu;\n"
          "  stargazer_sightings = %lu;\n"
          "  autonomous_steps = %d;\n"
          "  total_steps = %d;\n"
          "  using_enml = %s;\n"
          "};\n",
          bag_file.c_str(),
          bag_duration,
          distance_traversed,
          laser_scan_msgs,
          kinect_scan_msgs,
          (*kinect_image_msgs_ptr),
          (*stargazer_sightings_ptr),
          autonomous_steps,
          total_steps,
          using_enml ? "true" : "false");
  fclose(fp);
  if (!is_interesting) {
    bag_duration = 0.0;
    laser_scan_msgs = 0;
  }
  if (using_enml) {
    *enml_distance = distance_traversed;
  } else {
    *enml_distance = 0;
  }
  return is_interesting;
}

int main(int argc, char** argv) {
  // ROS node init
  ros::init(argc, argv, "datachecker_node");
  ros::NodeHandle nh("~");

  std::string config_file;
  nh.getParam("config_file", config_file);
  ROS_INFO("Loading config file: %s", config_file.c_str());

  YAML::Node settings = YAML::LoadFile(config_file);
  YAML::Node topics = settings["topics"];

  ROS_INFO("Finished loading config file");

  std::string bag_file = settings["bag_file"].as<std::string>();

  rosbag::Bag bag;
  try {
    bag.open(bag_file,rosbag::bagmode::Read);
  } catch(rosbag::BagException const& exception) {
    std::cout << "Could not open bag file: " << settings["bag_file"].as<std::string>() << "\n";
    return -1;
  }

  for (const auto& kv : topics) {
    const YAML::Node& topic_info = kv.second;  // the value

    std::string topic_name = topic_info["name"].as<std::string>();
    double topic_period_mean = topic_info["period_mean"].as<double>();
    double topic_period_std_dev = topic_info["period_std_dev"].as<double>();
    RegisterTopic(topic_name, topic_period_mean, topic_period_std_dev);
  }

  rosbag::View view(bag); 

  int num_files = 1;
  vector<double> distances(num_files, 0.0);
  vector<double> enml_distances(num_files, 0.0);
  vector<double> durations(num_files, 0.0);
  vector<uint64_t> laser_scans(num_files, 0);
  vector<uint64_t> kinect_scans(num_files, 0);
  vector<uint64_t> kinect_images(num_files, 0);
  vector<uint64_t> stargazer_sigtings(num_files, 0);

  std::map<std::string, MessageParserBase*> msg_parser_map;

  // msg_parser_map["ouster/lidar_packets"]        = new MessageParser<ouster_ros::PacketMsg>();
  msg_parser_map["sensor_msgs/CameraInfo"]      = new MessageParser<sensor_msgs::CameraInfo>();
  msg_parser_map["sensor_msgs/CompressedImage"] = new MessageParser<sensor_msgs::CompressedImage>();
  msg_parser_map["sensor_msgs/Image"]           = new MessageParser<sensor_msgs::Image>();
  msg_parser_map["sensor_msgs/Imu"]             = new MessageParser<sensor_msgs::Imu>();
  msg_parser_map["sensor_msgs/MagneticField"]   = new MessageParser<sensor_msgs::MagneticField>();
  msg_parser_map["sensor_msgs/NavSatFix"]       = new MessageParser<sensor_msgs::NavSatFix>();

  msg_parser_map["nav_msgs/Odometry"]           = new MessageParser<nav_msgs::Odometry>();


  ParseBagFile(
        bag_file,
        msg_parser_map,
        &(distances[0]),
        &(laser_scans[0]),
        &(kinect_scans[0]),
        &(kinect_images[0]),
        &(stargazer_sigtings[0]),
        &(durations[0]),
        &(enml_distances[0]));

  ros::Rate loop(1);
  while(ros::ok()){
    //do some computations and publish messages
    // std::cout << settings['bag_file'] << '\n';
    loop.sleep();
    ros::spinOnce();
  }

  return 0;
}
