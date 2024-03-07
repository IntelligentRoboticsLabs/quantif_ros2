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


TEST(qr2_tests, complete_system)
{
  auto camera_1_node = quantif_ros2::QR2NodeTest::make_shared("camera_1");
  auto laser_1_node = quantif_ros2::QR2NodeTest::make_shared("laser_1");
  auto filter_1_node = quantif_ros2::QR2NodeTest::make_shared("filter_1");
  auto consumer_node = quantif_ros2::QR2NodeTest::make_shared("consumer");
  camera_1_node->init();
  laser_1_node->init();
  filter_1_node->init();
  consumer_node->init();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
  executor.add_node(camera_1_node);
  executor.add_node(laser_1_node);
  executor.add_node(filter_1_node);
  executor.add_node(consumer_node);

  auto consumer_subs = consumer_node->get_subscribers();
  ASSERT_EQ(consumer_subs.size(), 2u);
  auto in1 = std::dynamic_pointer_cast<
    quantif_ros2::QR2Subscriber<quantif_ros2_interfaces::msg::Image>>(consumer_subs[0]);
  auto in2 = std::dynamic_pointer_cast<
    quantif_ros2::QR2Subscriber<quantif_ros2_interfaces::msg::LaserScan>>(consumer_subs[1]);

  int laser_seq = 0;
  int image_seq = 0;
  std::list<quantif_ros2_interfaces::msg::Image> in1_msgs;
  std::list<quantif_ros2_interfaces::msg::LaserScan> in2_msgs;
  auto start = camera_1_node->now();

  while ((camera_1_node->now() - start) < 3s) {
    auto in1_msg = std::make_unique<quantif_ros2_interfaces::msg::Image>();
    auto in2_msg = std::make_unique<quantif_ros2_interfaces::msg::LaserScan>();
    in1->get_last_msg(in1_msg);
    in2->get_last_msg(in2_msg);

    if (in1_msg != nullptr) {
      ASSERT_EQ(in1_msg->seq, laser_seq++);
      in1_msgs.push_back(*in1_msg);
    }
    if (in2_msg != nullptr) {
      ASSERT_EQ(in2_msg->seq, image_seq++);
      in2_msgs.push_back(*in2_msg);
    }

    executor.spin_some();
  }

  ASSERT_NEAR(in1_msgs.size(), 90, 3);
  ASSERT_NEAR(in1_msgs.back().seq, 90, 3);
  ASSERT_NEAR(in2_msgs.size(), 45, 3);
  ASSERT_NEAR(in2_msgs.back().seq, 45, 3);
}


int main(int argc, char ** argv)
{
  std::string pkg_dir = ament_index_cpp::get_package_share_directory("quantif_ros2");
  std::string path_config = pkg_dir + "/configs/test_config_2.yaml";

  std::vector<std::string> additionalArgs = {"--ros-args", "--params-file", path_config};
  auto [newArgc, newArgv] = quantif_ros2::extend_args(argc, argv, additionalArgs);

  rclcpp::init(newArgc, newArgv);

  testing::InitGoogleTest(&newArgc, newArgv);
  return RUN_ALL_TESTS();
}
