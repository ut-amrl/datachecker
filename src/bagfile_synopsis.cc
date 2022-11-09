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

#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

using std::string;
using std::vector;

// Name of the topic on Cobot's software stack that laser data is published on.
static const string kLaserScanTopic("/Cobot/Laser");
// Name of the topic on Cobot's software stack that Kinect scan data is
// published on.
static const string kKinectScanTopic("/Cobot/Kinect/Scan");
// Name of topic of Kinect depth images.
static const string kKinectDepthTopic("/Cobot/Kinect/Depth");
// Name of topic of throttled Kinect depth images.
static const string kKinectThrottledDepthTopic("/Cobot/Kinect/DepthThrottled");
// Name of the topic on Cobot's software stack that odometry is published on.
static const string kOdometryTopic("/Cobot/Odometry");
// Name of topic on Cobot's autonomous status is published.
static const string kCobotStatusTopic("/Cobot/Status");
// Name of topic on which Cobot's localzation is published.
static const string kCobotLocalizationTopic("/Cobot/Localization");
// Name of topic on which Cobot's StarGazer node data is published.
static const string kCobotStarGazerTopic("/Cobot/StarGazer");

void CheckKinectDepthMessage(
    const rosbag::MessageInstance& message,
    uint64_t* num_kinect_depth_msgs_ptr) {
  uint64_t& num_kinect_depth_msgs = *num_kinect_depth_msgs_ptr;
  sensor_msgs::ImagePtr depth_message =
      message.instantiate<sensor_msgs::Image>();
  if (depth_message != NULL) {
    if (message.getTopic() == kKinectDepthTopic ||
        message.getTopic() == kKinectThrottledDepthTopic) {
      ++num_kinect_depth_msgs;
    }
  }
}

void CheckLaserScanMessage(
    const rosbag::MessageInstance& message, uint64_t* num_laser_msgs_ptr,
    uint64_t* num_kinect_msgs_ptr) {
  uint64_t& laser_scan_msgs = *num_laser_msgs_ptr;
  uint64_t& kinect_scan_msgs = *num_kinect_msgs_ptr;
  // Check to see if this is a laser scan message.
  sensor_msgs::LaserScanPtr laser_message =
      message.instantiate<sensor_msgs::LaserScan>();
  if (laser_message != NULL) {
    if (message.getTopic() == kKinectScanTopic) {
      ++kinect_scan_msgs;
    } else if (message.getTopic() == kLaserScanTopic) {
      ++laser_scan_msgs;
    }
  }
}

bool GenerateBagfileSynopsis(const string& filename,
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
    bag.open(filename,rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
    printf("Unable to read %s, reason:\n %s\n", filename.c_str(), exception.what());
    return false;
  }
  vector<string> topics;
  topics.push_back(kLaserScanTopic);
  topics.push_back(kKinectScanTopic);
  topics.push_back(kKinectDepthTopic);
  topics.push_back(kKinectThrottledDepthTopic);
  topics.push_back(kOdometryTopic);
  topics.push_back(kCobotLocalizationTopic);
  topics.push_back(kCobotStatusTopic);
  topics.push_back(kCobotStarGazerTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

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
  bool autonomous = false;
  bool using_enml = true;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }
    // CheckIfAutonomous(message, &autonomous_steps, &total_steps, &autonomous);
    CheckLaserScanMessage(message, &laser_scan_msgs, kinect_scan_msgs_ptr);
    CheckKinectDepthMessage(message, kinect_image_msgs_ptr);
    // if (autonomous) {
    //   CheckOdometryMessage(message, &distance_traversed);
    //   CheckStargazerSightings(message, stargazer_sightings_ptr);
    //   // We assume CoBot is using EnML unless found otherwise.
    //   if (using_enml) CheckEnml(message, &using_enml);
    // }
  }
  bag_duration = bag_time - bag_time_start;
  const bool is_interesting =
      distance_traversed > kMinDistance &&
      (laser_scan_msgs > 0 || kinect_scan_msgs > 0);
  if (is_interesting) {
    *total_distance = *total_distance + distance_traversed;
    // Create a dummy file ".lta_interesting" to indicate that this bag file
    // is of interest for long-term autonomy (lta).
    // string interest_file = string(filename) + string(".lta_interesting");
    // ScopedFile fp(interest_file.c_str(), "w");
    // // Write a single byte, the newline character.
    // fprintf(fp, "\n");
  }
  if (debug) {
    printf("%s %9.1fs %7.1fm %6lu %6lu %9.1fm %6d %6d %d\n",
           filename.c_str(), bag_duration, distance_traversed,
           laser_scan_msgs, kinect_scan_msgs, *total_distance,
           autonomous_steps,
           total_steps, is_interesting?1:0);
  }
  string synopsis_file = string(filename) + string(".synopsis");

  FILE* fp = fopen(synopsis_file.c_str(), "w");
  // ScopedFile fp(synopsis_file.c_str(), "w");
  if (fp == NULL) {
    printf("Unable to write to %s\n", synopsis_file.c_str());
    return false;
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
          filename.c_str(),
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
  if (argc < 2) return 0;
  const size_t num_files = argc - 1;
  vector<double> distances(num_files, 0.0);
  vector<double> enml_distances(num_files, 0.0);
  vector<double> durations(num_files, 0.0);
  vector<uint64_t> laser_scans(num_files, 0);
  vector<uint64_t> kinect_scans(num_files, 0);
  vector<uint64_t> kinect_images(num_files, 0);
  vector<uint64_t> stargazer_sigtings(num_files, 0);

  // OMP_PARALLEL_FOR
  for (int i = 1; i < argc; ++i) {
    GenerateBagfileSynopsis(
        argv[i],
        &(distances[i - 1]),
        &(laser_scans[i - 1]),
        &(kinect_scans[i - 1]),
        &(kinect_images[i - 1]),
        &(stargazer_sigtings[i - 1]),
        &(durations[i - 1]),
        &(enml_distances[i - 1]));
  }

  double total_distance = 0.0;
  double total_enml_distance = 0.0;
  double total_duration = 0.0;
  uint64_t total_laser_scans = 0;
  uint64_t total_kinect_scans = 0;
  uint64_t total_kinect_images = 0;
  uint64_t total_stargazer_sightings = 0;
  for (size_t i = 0; i < num_files; ++i) {
    total_distance += distances[i];
    total_duration += durations[i];
    total_enml_distance += enml_distances[i];
    total_laser_scans += laser_scans[i];
    total_kinect_scans += kinect_scans[i];
    total_kinect_images += kinect_images[i];
    total_stargazer_sightings += stargazer_sigtings[i];
  }
  printf("Total distance traversed: %f km\n", total_distance / 1000.0);
  printf("Total laser scans: %lu\n", total_laser_scans);
  printf("Total Kinect scans: %lu\n", total_kinect_scans);
  printf("Total Kinect images: %lu\n", total_kinect_images);
  printf("Total stargazer sightings: %lu\n", total_stargazer_sightings);
  printf("Total duration: %20f\n", total_duration);
  printf("Total EnML distance: %f\n", total_enml_distance / 1000.0);
  return 0;
}
