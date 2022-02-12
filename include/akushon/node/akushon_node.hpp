// Copyright (c) 2021 Ichiro ITS
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
#include "akushon_interfaces/action/run_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace akushon
{

class AkushonNode
{
public:
  enum
  {
    SUCCEEDED,
    CANCELED,
    FAILED
  };

  explicit AkushonNode(rclcpp::Node::SharedPtr node);

  void set_action_manager(std::shared_ptr<ActionManager> action_manager);

private:
  rclcpp::Node::SharedPtr node;

  std::shared_ptr<ActionNode> action_node;

  rclcpp_action::Server<akushon_interfaces::action::RunAction>::SharedPtr run_action_server;
};

}  // namespace akushon

#endif  // AKUSHON__NODE__AKUSHON_NODE_HPP_
