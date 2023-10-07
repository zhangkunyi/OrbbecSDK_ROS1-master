/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#pragma once
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>

#include "types.h"
#include "utils.h"

namespace orbbec_camera {
class IMUSynchronizer {
 public:
  IMUSynchronizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~IMUSynchronizer();
  void messageCallback(const sensor_msgs::ImuConstPtr& acc_msg,
                       const sensor_msgs::ImuConstPtr& gyr_msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  message_filters::Subscriber<sensor_msgs::Imu> acc_sub_;
  message_filters::Subscriber<sensor_msgs::Imu> gyr_sub_;
  using MySyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> imu_synchronizer_;
  ros::Publisher imu_synchronizer_pub_;
};
}  // namespace orbbec_camera
