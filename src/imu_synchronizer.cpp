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

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/opencv.hpp>

#include "orbbec_camera/imu_synchronizer.h"

namespace orbbec_camera {
IMUSynchronizer::IMUSynchronizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  acc_sub_.subscribe(nh_, "accel/sample", 1);
  gyr_sub_.subscribe(nh_, "gyro/sample", 1);
  imu_synchronizer_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(1000), acc_sub_,
                                                                        gyr_sub_);
  imu_synchronizer_->registerCallback(boost::bind(&IMUSynchronizer::messageCallback, this, _1, _2));
  imu_synchronizer_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/sample", 1);
}
IMUSynchronizer::~IMUSynchronizer() = default;

void IMUSynchronizer::messageCallback(const sensor_msgs::ImuConstPtr& acc_msg,
                                      const sensor_msgs::ImuConstPtr& gyr_msg) {
  sensor_msgs::Imu imu_msg;
  imu_msg.angular_velocity.x = gyr_msg->angular_velocity.x;
  imu_msg.angular_velocity.y = gyr_msg->angular_velocity.y;
  imu_msg.angular_velocity.z = gyr_msg->angular_velocity.z;
  imu_msg.linear_acceleration.x = acc_msg->linear_acceleration.x;
  imu_msg.linear_acceleration.y = acc_msg->linear_acceleration.y;
  imu_msg.linear_acceleration.z = acc_msg->linear_acceleration.z;
  imu_msg.header.stamp = gyr_msg->header.stamp;
  imu_msg.header.frame_id = DEFAULT_IMU_OPTICAL_FRAME_ID;
  imu_msg.header.seq = gyr_msg->header.seq;
  imu_synchronizer_pub_.publish(imu_msg);
}

}  // namespace orbbec_camera
