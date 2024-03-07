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


#include "rclcpp/rclcpp.hpp"

#include "quantif_ros2/QR2Node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  std::vector<quantif_ros2::QR2Node::SharedPtr> nodes;
  std::vector<std::string> node_ids = {
    "camera_1",
    "laser_1",
    "ball_detector",
    "obstacle_detector",
    "controller",
    "mobile_base",
  };

  for (const auto & id : node_ids) {
    auto node = quantif_ros2::QR2Node::make_shared(id);
    node->init();
    nodes.push_back(node);
    executor.add_node(node);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
