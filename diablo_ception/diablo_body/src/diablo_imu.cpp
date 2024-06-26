// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diablo_imu.hpp"

using namespace std::chrono;

void diablo_imu_publisher::imu_pub_init(void)
{
  imu_Publisher_ = this->node_ptr->create_publisher<sensor_msgs::msg::Imu>("diablo/sensor/Imu", 10);
  timer_ =
    this->node_ptr->create_wall_timer(20ms, std::bind(&diablo_imu_publisher::lazyPublisher, this));
  this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_50Hz);
  this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_50Hz);
  this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_50Hz);
  this->vehicle->telemetry->configUpdate();
}

void diablo_imu_publisher::lazyPublisher(void)
{
  if (imu_Publisher_->get_subscription_count() > 0) {
    bool imu_Pub_mark = false;
    if (this->vehicle->telemetry->newcome & 0x10) {
      imu_Pub_mark = true;
      imu_msg_.linear_acceleration.x = this->vehicle->telemetry->accl.x;
      imu_msg_.linear_acceleration.y = this->vehicle->telemetry->accl.y;
      imu_msg_.linear_acceleration.z = this->vehicle->telemetry->accl.z;
      this->vehicle->telemetry->eraseNewcomeFlag(0xEF);
    }

    if (this->vehicle->telemetry->newcome & 0x08) {
      imu_Pub_mark = true;
      imu_msg_.angular_velocity.x = this->vehicle->telemetry->gyro.x;
      imu_msg_.angular_velocity.y = this->vehicle->telemetry->gyro.y;
      imu_msg_.angular_velocity.z = this->vehicle->telemetry->gyro.z;
      this->vehicle->telemetry->eraseNewcomeFlag(0xF7);
    }
    if (this->vehicle->telemetry->newcome & 0x20) {
      imu_Pub_mark = true;
      imu_msg_.orientation.w = this->vehicle->telemetry->quaternion.w;
      imu_msg_.orientation.x = this->vehicle->telemetry->quaternion.x;
      imu_msg_.orientation.y = this->vehicle->telemetry->quaternion.y;
      imu_msg_.orientation.z = this->vehicle->telemetry->quaternion.z;
      this->vehicle->telemetry->eraseNewcomeFlag(0xDF);
    }
    if (imu_Pub_mark) {
      imu_timestamp = this->node_ptr->get_clock()->now();
      imu_msg_.header.stamp = imu_timestamp;
      imu_msg_.header.frame_id = "diablo_robot";
      imu_Publisher_->publish(imu_msg_);
      imu_Pub_mark = false;
    }
  }
}

diablo_imu_publisher::diablo_imu_publisher(
  rclcpp::Node::SharedPtr node_ptr, DIABLO::OSDK::Vehicle * vehicle)
{
  this->node_ptr = node_ptr;
  this->vehicle = vehicle;
}
