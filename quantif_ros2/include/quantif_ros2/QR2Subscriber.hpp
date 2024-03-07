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

#ifndef QUANTIF_ROS2__QR2SUBSCRIBER_HPP_
#define QUANTIF_ROS2__QR2SUBSCRIBER_HPP_

#include <memory>

#include "quantif_ros2_interfaces/msg/vector3.hpp"
#include "quantif_ros2_interfaces/msg/twist.hpp"
#include "quantif_ros2_interfaces/msg/laser_scan.hpp"
#include "quantif_ros2_interfaces/msg/image.hpp"
#include "quantif_ros2_interfaces/msg/point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace quantif_ros2
{

class QR2SubscriberBase
{
public:
  virtual ~QR2SubscriberBase() {}

  virtual void clear_msgs() = 0;
  virtual std::string get_type() = 0;
};

template<class T>
class QR2Subscriber : public QR2SubscriberBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(QR2Subscriber)

  QR2Subscriber(
    rclcpp::Node::SharedPtr node, const std::string & topic,
    rclcpp::QoS qos, const std::string & type)
  {
    type_ = type;

    subscriber_ = node->create_subscription<T>(
      topic, qos,
      [&](typename T::UniquePtr msg) {
        last_msg_ = std::move(msg);
      });
  }

  void get_last_msg(typename T::UniquePtr msg)
  {
    msg = std::move(last_msg_);
  }

  void clear_msgs()
  {
    last_msg_ = nullptr;
  }

  std::string get_type()
  {
    return type_;
  }

protected:
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;
  typename T::UniquePtr last_msg_;
  std::string type_;
};

class QR2SubscriberFactory
{
public:
  QR2SubscriberFactory(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
  }

  std::shared_ptr<QR2SubscriberBase> create_subscriber(const std::string & id)
  {
    std::string topic;
    std::string type;
    std::string qos_id;
    node_->declare_parameter(id + ".topic", topic);
    node_->declare_parameter(id + ".type", type);
    node_->declare_parameter(id + ".qos", qos_id);
    node_->get_parameter(id + ".topic", topic);
    node_->get_parameter(id + ".type", type);
    node_->get_parameter(id + ".qos", qos_id);

    RCLCPP_INFO(
      node_->get_logger(), "\t\tSubscriber [%s] type(%s) qos(%s)",
      topic.c_str(), type.c_str(), qos_id.c_str());

    rclcpp::QoS qos(100);
    if (qos_id == "sensor") {
      qos = rclcpp::SensorDataQoS();
    } else if (qos_id == "sensor.reliable") {
      qos = rclcpp::SensorDataQoS().reliable();
    } else if (qos_id == "reliable") {
      qos = rclcpp::QoS(100);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "Unsupported qos (%s) for topic %s", qos_id.c_str(), id.c_str());
      return nullptr;
    }

    std::shared_ptr<QR2SubscriberBase> ret;
    if (type == "Vector3") {
      ret = std::make_shared<QR2Subscriber<quantif_ros2_interfaces::msg::Vector3>>(
        node_, topic, qos, type);
    } else if (type == "Twist") {
      ret = std::make_shared<QR2Subscriber<quantif_ros2_interfaces::msg::Twist>>(
        node_, topic, qos, type);
    } else if (type == "LaserScan") {
      ret = std::make_shared<QR2Subscriber<quantif_ros2_interfaces::msg::LaserScan>>(
        node_, topic, qos, type);
    } else if (type == "Image") {
      ret = std::make_shared<QR2Subscriber<quantif_ros2_interfaces::msg::Image>>(
        node_, topic, qos, type);
    } else if (type == "PointCloud2") {
      ret = std::make_shared<QR2Subscriber<quantif_ros2_interfaces::msg::PointCloud2>>(
        node_, topic, qos, type);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "Unsupported type (%s) for topic %s", type.c_str(), id.c_str());
      return nullptr;
    }

    return ret;
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace quantif_ros2

#endif  // QUANTIF_ROS2__QR2SUBSCRIBER_HPP_
