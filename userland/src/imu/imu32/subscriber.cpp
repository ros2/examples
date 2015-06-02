// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_msgs/msg/imu32.hpp>
#include "userland/command_line_arguments.h"


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
  typename rclcpp::subscription::Subscription<T>::CallbackType callback)
{
  auto sub = node->create_subscription<T>("imu", 1000, callback);
  return sub;
}

void print_accel_data(const simple_msgs::msg::Imu32::ConstPtr & msg)
{

  std::cout << "-------------------------" << std::endl;
  std::cout << "Got Imu32 msg" << std::endl;
  std::cout << "-------------------------" << std::endl;

  // Define the header
  std::cout << "ros_msg->header.seq= " << msg->header.seq << std::endl;
  std::cout << "msg->header.stamp.sec= " << msg->header.stamp.sec << std::endl;
  std::cout << "msg->header.stamp.nanosec= " << msg->header.stamp.nanosec << std::endl;
  std::cout << "msg->header.frame_id= " << msg->header.frame_id << std::endl;

  // Define the orientation
  std::cout << "msg->orientation.x= " << msg->orientation.x << std::endl;
  std::cout << "msg->orientation.y= " << msg->orientation.y << std::endl;
  std::cout << "msg->orientation.z= " << msg->orientation.z << std::endl;
  std::cout << "msg->orientation.w= " << msg->orientation.w << std::endl;


  // Define the orientation_covariance
  for (int i = 0; i < 9; i++) {
    std::cout << "msg->orientation_covariance[i]= " << msg->orientation_covariance[i] << std::endl;
  }
  // Define the angular_velocity
  std::cout << "msg->angular_velocity.x= " << msg->angular_velocity.x << std::endl;
  std::cout << "msg->angular_velocity.y= " << msg->angular_velocity.y << std::endl;
  std::cout << "msg->angular_velocity.z= " << msg->angular_velocity.z << std::endl;

  // Define the angular_velocity_covariance
  for (int i = 0; i < 9; i++) {
    std::cout << "msg->angular_velocity_covariance[i]= " << msg->angular_velocity_covariance[i] <<
      std::endl;
  }
  // Define the linear_acceleration
  std::cout << "msg->linear_acceleration.x= " << msg->linear_acceleration.x << std::endl;
  std::cout << "msg->linear_acceleration.y= " << msg->linear_acceleration.y << std::endl;
  std::cout << "msg->linear_acceleration.z= " << msg->linear_acceleration.z << std::endl;

  // Define the linear_acceleration_covariance
  for (int i = 0; i < 9; i++) {
    std::cout << "msg->linear_acceleration_covariance[i]= " <<
      msg->linear_acceleration_covariance[i] << std::endl;
  }
  std::cout << "-------------------------" << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage:" << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  auto node = rclcpp::Node::make_shared("subscriber");
  rclcpp::subscription::SubscriptionBase::SharedPtr sub;

  sub = subscribe<simple_msgs::msg::Imu32>(node, print_accel_data);
  rclcpp::spin(node);

  return 0;
}
