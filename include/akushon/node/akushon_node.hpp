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

#ifndef AKUSHON__NODE__AKUSHON_NODE_HPP_
#define AKUSHON__NODE__AKUSHON_NODE_HPP_

#include <memory>
#include <string>

#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/node/action_node.hpp"
#include "akushon/config/node/config_node.hpp"
#include "akushon_interfaces/srv/save_actions.hpp"
#include "akushon_interfaces/srv/get_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace akushon
{

class AkushonNode
{
public:
  explicit AkushonNode(rclcpp::Node::SharedPtr node);

  void run_action_manager(std::shared_ptr<ActionManager> action_manager);

  void run_config_service(const std::string & path, const std::shared_ptr<ActionManager> & action_manager);

private:
  double start_time;
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;

  std::shared_ptr<ActionNode> action_node;

  std::shared_ptr<ConfigNode> config_node;
};

}  // namespace akushon

#endif  // AKUSHON__NODE__AKUSHON_NODE_HPP_
