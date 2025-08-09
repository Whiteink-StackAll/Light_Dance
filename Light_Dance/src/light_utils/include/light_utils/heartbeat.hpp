// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
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
#ifndef LIGHT_UTILS_HEARTBEAT_HPP_
#define LIGHT_UTILS_HEARTBEAT_HPP_

// std
#include <memory>
#include <thread>
#include <chrono>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class HeartBeatPublisher {
public:
  using SharedPtr = std::shared_ptr<HeartBeatPublisher>;

  static SharedPtr create(rclcpp::Node *node);

  ~HeartBeatPublisher();
private:
  explicit HeartBeatPublisher(rclcpp::Node *node);

  std_msgs::msg::Int64 message_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  std::thread pub_thread_;

  std::atomic<bool> is_running_{true};
};

#endif  // LIGHT_UTILS_HEARTBEAT_HPP_
