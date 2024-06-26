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

#include <memory>
#include <iostream>
#include <string>
#include <vector>

#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/model/action.hpp"
#include "akushon/action/model/action_name.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon/action/node/action_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  const char * help_message = 
    "Usage: ros2 run akushon action <path> <action_name>\n"
    "  <path>        Path to the action configuration file\n"
    "  <action_name> Name of the action to run\n";


  if (argc < 3) {
    std::cerr << "Bad arguments!\n";
    std::cerr << help_message;
    return 0;
  }

  auto action_manager = std::make_shared<akushon::ActionManager>();
  std::string path = argv[1];
  std::string action_name = argv[2];
  action_manager->load_config(path);

  auto node = std::make_shared<rclcpp::Node>("akushon_node");
  auto action_node = std::make_shared<akushon::ActionNode>(node, action_manager);

  rclcpp::Rate rcl_rate(8ms);
  int time = 0;

  if (action_node->start(action_name)) {
    while (rclcpp::ok()) {
      rcl_rate.sleep();

      if (!action_node->update(time)) {
        break;
      }

      time += 8;
    }
  } else {
    std::cout << "the action not found\n";
  }

  return 0;
}
