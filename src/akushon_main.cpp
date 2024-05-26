// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <moveit/move_group_interface/move_group_interface.h>

#include <iostream>
#include <memory>
#include <string>

#include "akushon/action/node/action_manager.hpp"
#include "akushon/node/akushon_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the path!" << std::endl;
    return 0;
  }

  auto node = std::make_shared<rclcpp::Node>(
    "akushon_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto akushon_node = std::make_shared<akushon::AkushonNode>(node);

  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface(new MoveGroupInterface(node, "robot"));

  auto action_manager = std::make_shared<akushon::ActionManager>();

  std::string path = argv[1];

  action_manager->load_config(path);
  action_manager->load_move_group_interface(move_group_interface);

  akushon_node->run_action_manager(action_manager);
  akushon_node->run_config_service(path);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
