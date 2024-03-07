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


template<class T>
void get_and_publish(
  std::shared_ptr<QR2SubscriberBase> sub, const std::string & type,
  std::vector<std::shared_ptr<QR2PublisherBase>> & publishers)
{
  typename T::UniquePtr msg;
  auto typed_sub = std::dynamic_pointer_cast<QR2Subscriber<T>>(sub);
  typed_sub->get_last_msg(std::move(msg));

  if (msg != nullptr) {
    for (auto & publisher : publishers) {
      if (publisher->get_type() == type) {
        auto typed_pub = std::dynamic_pointer_cast<QR2Publisher<T>>(publisher);
        typed_pub->publish(std::move(msg));
        break;
      }
    }
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
  }

  if (type_ == "Processor") {
    for (auto & publisher : publishers_) {
      publisher->produce_and_publish();
    }
    for (auto & subscriber : subscribers_) {
      subscriber->clear_msgs();
    }
  }

  if (type_ == "Filter") {
    for (auto & subscriber : subscribers_) {
      auto type_sub = subscriber->get_type();

      if (type_sub == "Vector3") {
        get_and_publish<quantif_ros2_interfaces::msg::Vector3>(
          subscriber, type_sub, publishers_);
      }
      if (type_sub == "Twist") {
        get_and_publish<quantif_ros2_interfaces::msg::Twist>(
          subscriber, type_sub, publishers_);
      }
      if (type_sub == "LaserScan") {
        get_and_publish<quantif_ros2_interfaces::msg::LaserScan>(
          subscriber, type_sub, publishers_);
      }
      if (type_sub == "Image") {
        get_and_publish<quantif_ros2_interfaces::msg::Image>(
          subscriber, type_sub, publishers_);
      }
      if (type_sub == "PointCloud2") {
        get_and_publish<quantif_ros2_interfaces::msg::PointCloud2>(
          subscriber, type_sub, publishers_);
      }

      subscriber->clear_msgs();
    }
  }

  if (type_ == "Consumer") {
    for (auto & subscriber : subscribers_) {
      subscriber->clear_msgs();
    }
  }


  while ((now() - start).seconds() < processing_time_) {}
}

}  // namespace quantif_ros2
