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

#include <list>

#include "quantif_ros2_interfaces/msg/vector3.hpp"
#include "quantif_ros2_interfaces/msg/twist.hpp"
#include "quantif_ros2_interfaces/msg/laser_scan.hpp"
#include "quantif_ros2_interfaces/msg/image.hpp"
#include "quantif_ros2_interfaces/msg/point_cloud2.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "quantif_ros2/QR2Node.hpp"
#include "test_utils.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;


TEST(qr2_tests, producer_laser)
{
  auto producer_node = quantif_ros2::QR2NodeTest::make_shared("laser_1");
  producer_node->init();

  auto test_node = rclcpp::Node::make_shared("test_node");
  std::list<quantif_ros2_interfaces::msg::LaserScan> received_msgs;
  auto subscriber = test_node->create_subscription<quantif_ros2_interfaces::msg::LaserScan>(
    "laser_1/out", rclcpp::SensorDataQoS().reliable(),
    [&received_msgs](quantif_ros2_interfaces::msg::LaserScan msg) {
      received_msgs.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(producer_node);
  executor.add_node(test_node);

  auto start = test_node->now();
  while ((test_node->now() - start) < 3s) {
    executor.spin_some();
  }

  ASSERT_NEAR(received_msgs.size(), 45, 3);
  ASSERT_NEAR(received_msgs.back().seq, 45, 3);
}

TEST(qr2_tests, producer_camera)
{
  auto producer_node = quantif_ros2::QR2NodeTest::make_shared("camera_1");
  producer_node->init();

  auto test_node = rclcpp::Node::make_shared("test_node");
  std::list<quantif_ros2_interfaces::msg::Image> received_msgs;
  auto subscriber = test_node->create_subscription<quantif_ros2_interfaces::msg::Image>(
    "camera_1/out", rclcpp::SensorDataQoS().reliable(),
    [&received_msgs](quantif_ros2_interfaces::msg::Image msg) {
      received_msgs.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(producer_node);
  executor.add_node(test_node);

  auto start = test_node->now();
  while ((test_node->now() - start) < 3s) {
    executor.spin_some();
  }

  ASSERT_NEAR(received_msgs.size(), 90, 3);
  ASSERT_NEAR(received_msgs.back().seq, 90, 3);
}

TEST(qr2_tests, processor_ball_detector)
{
  auto processor_node = quantif_ros2::QR2NodeTest::make_shared("ball_detector");
  auto producer_node = quantif_ros2::QR2NodeTest::make_shared("camera_1");
  processor_node->init();
  producer_node->init();

  auto test_node = rclcpp::Node::make_shared("test_node");
  std::list<quantif_ros2_interfaces::msg::Vector3> received_msgs;
  auto subscriber = test_node->create_subscription<quantif_ros2_interfaces::msg::Vector3>(
    "ball_detector/out", 100,
    [&received_msgs](quantif_ros2_interfaces::msg::Vector3 msg) {
      received_msgs.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(processor_node);
  executor.add_node(producer_node);
  executor.add_node(test_node);

  auto proc_subs = processor_node->get_subscribers();
  ASSERT_EQ(proc_subs.size(), 1u);
  auto proc_sub = std::dynamic_pointer_cast<
    quantif_ros2::QR2Subscriber<quantif_ros2_interfaces::msg::Image>>(proc_subs[0]);

  std::list<quantif_ros2_interfaces::msg::Image> in_msgs;
  int i = 0;
  auto start = test_node->now();
  while ((test_node->now() - start) < 3s) {
    auto in_msg = std::make_unique<quantif_ros2_interfaces::msg::Image>();
    proc_sub->get_last_msg(in_msg);

    if (in_msg != nullptr) {
      ASSERT_EQ(in_msg->seq, i++);
      in_msgs.push_back(*in_msg);
    }

    executor.spin_some();
  }

  ASSERT_NEAR(in_msgs.size(), 90, 3);
  ASSERT_NEAR(in_msgs.back().seq, 90, 3);
  ASSERT_NEAR(received_msgs.size(), 90, 3);
  ASSERT_NEAR(received_msgs.back().seq, 90, 3);
}

TEST(qr2_tests, obstacle_detector_detector)
{
  auto processor_node = quantif_ros2::QR2NodeTest::make_shared("obstacle_detector");
  auto producer_node = quantif_ros2::QR2NodeTest::make_shared("laser_1");
  processor_node->init();
  producer_node->init();

  auto test_node = rclcpp::Node::make_shared("test_node");
  std::list<quantif_ros2_interfaces::msg::Vector3> received_msgs;
  auto subscriber = test_node->create_subscription<quantif_ros2_interfaces::msg::Vector3>(
    "obstacle_detector/out", 100,
    [&received_msgs](quantif_ros2_interfaces::msg::Vector3 msg) {
      received_msgs.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(processor_node);
  executor.add_node(producer_node);
  executor.add_node(test_node);

  auto proc_subs = processor_node->get_subscribers();
  ASSERT_EQ(proc_subs.size(), 1u);
  auto proc_sub = std::dynamic_pointer_cast<
    quantif_ros2::QR2Subscriber<quantif_ros2_interfaces::msg::LaserScan>>(proc_subs[0]);

  std::list<quantif_ros2_interfaces::msg::LaserScan> in_msgs;
  int i = 0;
  auto start = test_node->now();
  while ((test_node->now() - start) < 3s) {
    auto in_msg = std::make_unique<quantif_ros2_interfaces::msg::LaserScan>();
    proc_sub->get_last_msg(in_msg);

    if (in_msg != nullptr) {
      ASSERT_EQ(in_msg->seq, i++);
      in_msgs.push_back(*in_msg);
    }

    executor.spin_some();
  }

  ASSERT_NEAR(in_msgs.size(), 45, 3);
  ASSERT_NEAR(in_msgs.back().seq, 45, 3);
  ASSERT_NEAR(received_msgs.size(), 90, 3);
  ASSERT_NEAR(received_msgs.back().seq, 90, 3);
}

int main(int argc, char ** argv)
{
  std::string pkg_dir = ament_index_cpp::get_package_share_directory("quantif_ros2");
  std::string path_config = pkg_dir + "/configs/test_config_1.yaml";

  std::vector<std::string> additionalArgs = {"--ros-args", "--params-file", path_config};
  auto [newArgc, newArgv] = quantif_ros2::extend_args(argc, argv, additionalArgs);

  rclcpp::init(newArgc, newArgv);

  testing::InitGoogleTest(&newArgc, newArgv);
  return RUN_ALL_TESTS();
}
