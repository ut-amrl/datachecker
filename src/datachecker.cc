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

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"

#include "nav_msgs/Odometry.h"

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "std_msgs/Header.h"

// Custom imports
#include "generic_message.h"

using std::string;
using std::vector;
using std::deque;


struct TopicInfo {
  int windowSize = 0;
  std::deque<double> timestamps;

  double expectedPeriod;
  double expectedPeriodStdDev;

  double stdDevThreshold;
  int numObservations;
  int numInvalidObservations;

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

template <typename T>
void AddTopicObservation(std::string topicName, const T& message, FILE* log_fp){
  /**
    TODO: Implement this using an interface class.
  */
  // total_obs+=1;

  // int windowSize = topicInfo[topicName].windowSize;
  // std::deque<double>& timestamps = topicInfo[topicName].timestamps;

  // double expectedPeriod = topicInfo[topicName].expectedPeriod;
  // double expectedPeriodStdDev = topicInfo[topicName].expectedPeriodStdDev;
  // double stdDevThreshold = topicInfo[topicName].stdDevThreshold;

  // double lowerPeriodBound = expectedPeriod - expectedPeriodStdDev * stdDevThreshold;
  // double upperPeriodBound = expectedPeriod + expectedPeriodStdDev * stdDevThreshold;

  // timestamps.push_back(message.getTime().toSec());

  // topicInfo[topicName].numObservations+=1;

  // if (timestamps.size() > (uint32_t)windowSize) {
  //   timestamps.pop_front();

  //   double avgPeriod = (timestamps.back() - timestamps.front()) / (windowSize - 1);

  //   if (avgPeriod < lowerPeriodBound || avgPeriod > upperPeriodBound){
  //     invalid_obs += 1;
      
  //     fprintf(log_fp,
  //         "Topic Rate Drop\n"
  //         "  time = %2.4f;\n"
  //         "  topic = \"%s\";\n"
  //         "  rate_measured = %2.4f;\n"
  //         "  period_measured = %2.4f;\n"
  //         "  expected_period = %2.4f;\n"
  //         "  expected_period_std_dev = %2.4f;\n"
  //         "  window_size = %lu;\n",
  //         message.getTime().toSec(),
  //         topicName.c_str(),
  //         1.0 / avgPeriod,
  //         avgPeriod,
  //         topicInfo[topicName].expectedPeriod,
  //         topicInfo[topicName].expectedPeriodStdDev,
  //         topicInfo[topicName].windowSize
  //         );

  //     topicInfo[topicName].numInvalidObservations+=1;
  //   }
  // }
}

template<typename T>
void ParseMessageInstance(const T& msg, FILE* log_fp){ 
  /*
    Writes the message high level contents to a file
    time
    topic
    prev_seq_num
    current_seq_num
    package_size

    TODO: Implement this after converting the templated T message to a interface with overloaded functions for
    getTime()
    getTopic()
    getSeqNum()
    getSize()
  */
  // uint32_t seq_num = msg.header.seq;
  // std::string topic = msg.getTopic();

  // if (topicInfo[topic].last_seq_num_initialized && (topicInfo[topic].last_seq_num + 1 != seq_num)){
  //   fprintf(log_fp,
  //     "Topic SeqNum Skip\n"
  //     "  time = %2.4f;\n"
  //     "  topic = \"%s\";\n"
  //     "  prev_seq_num = %lu;\n"
  //     "  current_seq_num = %lu;\n",
  //     msg.getTime().toSec(),
  //     topic.c_str(),
  //     (unsigned long)topicInfo[topic].last_seq_num,
  //     (unsigned long)seq_num
  //     );
  // }
  ROS_INFO("Message Received with %s", msg.header.frame_id.c_str());

  // topicInfo[topic].last_seq_num = seq_num;
  // topicInfo[topic].last_seq_num_initialized = true;
}

template<typename T>
void messageCallback(const T& msg, const std::string topic_name, FILE* log_fp)
{
  // TODO: Convert msg to interface class

  // Check for timestamp and make sure that we are getting everything at the right frequency
  AddTopicObservation(topic_name, msg, log_fp); // TODO: edit the function a bit to take in the header timestamp
  // Check for sequence number and make sure that we are not skipping any packets
  ROS_INFO("messageCallback with topic %s:", topic_name.c_str());
  ParseMessageInstance(msg, log_fp);
}

void dummyFunc(const sensor_msgs::CompressedImageConstPtr& msg ) {
    ROS_INFO("DUMMY");
}

template<typename MessageType>
void subscribeToTopic(const std::string& topic_name, int queue_size, ros::NodeHandle& n, 
                      FILE* log_fp, std::vector<ros::Subscriber>& subscribers) {
    auto callback = [topic_name, log_fp](const boost::shared_ptr<const MessageType>& msg) {
        ROS_INFO("Callback for topic %s", topic_name.c_str());
        messageCallback(*msg, topic_name, log_fp);
    };
    subscribers.push_back(n.subscribe<MessageType>(topic_name, queue_size, callback));
}

int main(int argc, char** argv) {
   
  YAML::Node settings = YAML::LoadFile("/home/datachecker/src/settings.yaml");
  YAML::Node topics = settings["topics"];

  ros::init(argc, argv, "datachecker");
  ros::NodeHandle n;

  // TODO: Change this to use current datetime instead
  std::string log_file      = string("datachecker_test") + string(".log");
  FILE* log_fp = fopen(log_file.c_str(), "w");

   std::map<std::string, std::function<void(const std::string&, int, ros::NodeHandle&, FILE*, std::vector<ros::Subscriber>&)>> subscriberMap = {
      {"CompressedImage", &subscribeToTopic<sensor_msgs::CompressedImage>},
      {"NavSatFix", &subscribeToTopic<sensor_msgs::NavSatFix>},
      {"Imu", &subscribeToTopic<sensor_msgs::Imu>},
      {"MagneticField", &subscribeToTopic<sensor_msgs::MagneticField>},
      {"Odometry", &subscribeToTopic<nav_msgs::Odometry>},
      {"CameraInfo", &subscribeToTopic<sensor_msgs::CameraInfo>}
  };

  std::vector<ros::Subscriber> subscribers;
  for (const auto& kv : topics) {
    const YAML::Node& topic_info = kv.second;  // the value

    std::string topic_name = topic_info["name"].as<std::string>();
    double topic_period_mean = topic_info["period_mean"].as<double>();
    double topic_period_std_dev = topic_info["period_std_dev"].as<double>();
    RegisterTopic(topic_name, topic_period_mean, topic_period_std_dev);

    std::string topic_type = topic_info["topic_type"].as<std::string>();
    ROS_INFO("Found topic name %s", topic_name.c_str());
    subscriberMap[topic_type](topic_name, 1, n, log_fp, subscribers);
  }

  while (ros::ok()) {
    // TODO: Check the message queue. For each message, call the callback function
    // to 
    // 1. observe the message (log to file),   
    // 2. check for sequence number and timestamp
    // 3. Update SensorHealth/SensorStatus vector
    // 4. Once every K seconds, publish an updated SensorHealth/SensorStatus vector 
    ros::spinOnce();
  }

  return 0;
}
