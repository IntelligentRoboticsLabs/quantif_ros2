// Copyright 2024 Intelligent Robotics Lab
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

#ifndef QUANTIF_ROS2__QR2NODE_HPP_
#define QUANTIF_ROS2__QR2NODE_HPP_

#include <memory>

#include "quantif_ros2/QR2Publisher.hpp"
#include "quantif_ros2/QR2Subscriber.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace quantif_ros2
{

class QR2Node : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(QR2Node)
  
  QR2Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void init();

protected:
  void control_cycle();
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<QR2PublisherFactory> publisher_factory_;
  std::shared_ptr<QR2SubscriberFactory> subscriber_factory_;
  std::vector<std::shared_ptr<QR2PublisherBase>> publishers_;
  std::vector<std::shared_ptr<QR2SubscriberBase>> subscribers_;

  std::string type_;
  double rate_ {0.0};
  double processing_time_ {0.0};
};

}  // namespace quantif_ros2

#endif  // QUANTIF_ROS2__QR2NODE_HPP_
