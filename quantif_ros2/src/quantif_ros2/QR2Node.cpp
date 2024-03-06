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

#include <memory>

#include "quantif_ros2/QR2Publisher.hpp"
#include "quantif_ros2/QR2Subscriber.hpp"
#include "quantif_ros2/QR2Node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace quantif_ros2
{

QR2Node::QR2Node(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
}

void
QR2Node::init()
{
  declare_parameter("type", type_);

  get_parameter("type", type_);
  if (type_ == "Processor" || type_ == "Producer") {
    declare_parameter("rate", rate_);
  }

  declare_parameter("processing_time", processing_time_);
  if (type_ == "Processor" || type_ == "Producer") {
    get_parameter("rate", rate_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&QR2Node::control_cycle, this));
  }

  get_parameter("processing_time", processing_time_);

  subscriber_factory_ = std::make_shared<QR2SubscriberFactory>(shared_from_this());
  publisher_factory_ = std::make_shared<QR2PublisherFactory>(shared_from_this());

  RCLCPP_INFO(
    get_logger(), "Node: [%s] type(%s) rate(%lf) processing_time(%lf)",
    get_name(), type_.c_str(), rate_, processing_time_);

  std::vector<std::string> subscribers;
  declare_parameter("subscribers", subscribers);
  get_parameter("subscribers", subscribers);

  RCLCPP_INFO(get_logger(), "\tSubscribers: [%zu]", subscribers.size());

  for (const auto & subscriber : subscribers) {
    auto new_subscriber = subscriber_factory_->create_subscriber(subscriber);
    subscribers_.push_back(new_subscriber);
  }

  std::vector<std::string> publishers;
  declare_parameter("publishers", publishers);
  get_parameter("publishers", publishers);

  RCLCPP_INFO(get_logger(), "\tPublishers: [%zu]", publishers.size());

  for (const auto & publisher : publishers) {
    auto new_publisher = publisher_factory_->create_publisher(publisher);
    publishers_.push_back(new_publisher);
  }
}


void
QR2Node::control_cycle()
{
  auto start = now();
  if (type_ == "Producer") {
    for (auto & publisher : publishers_) {
      publisher->produce_and_publish();
    }
    for (auto & subscriber : subscribers_) {
      // subscriber->produce_and_publish();
    }
  }

  while ((now() - start).seconds() < processing_time_) {};
}

}  // namespace quantif_ros2
