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

#ifndef CONFIGS__TEST_UTILS_HPP_
#define CONFIGS__TEST_UTILS_HPP_

#include <string.h>
#include <tuple>
#include <string>
#include <vector>

namespace quantif_ros2
{

class QR2NodeTest : public quantif_ros2::QR2Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(QR2NodeTest)

  QR2NodeTest(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : QR2Node(node_name, options)
  {
  }

  std::vector<std::shared_ptr<QR2PublisherBase>> & get_publishers()
  {
    return publishers_;
  }

  std::vector<std::shared_ptr<QR2SubscriberBase>> & get_subscribers()
  {
    return subscribers_;
  }
};


std::tuple<int, char **>
extend_args(int argc, char ** argv, std::vector<std::string> args)
{
  int newArgc = argc + args.size();
  char ** newArgv = new char *[newArgc];

  for (int i = 0; i < argc; ++i) {
    size_t len = std::strlen(argv[i]) + 1;
    newArgv[i] = new char[len];
    std::strcpy(newArgv[i], argv[i]);
  }
  for (size_t i = 0; i < args.size(); ++i) {
    size_t len = args[i].length() + 1;
    newArgv[argc + i] = new char[len];
    std::strcpy(newArgv[argc + i], args[i].c_str());
  }

  return {newArgc, newArgv};
}

}  // namespace quantif_ros2

#endif  // CONFIGS__TEST_UTILS_HPP_
