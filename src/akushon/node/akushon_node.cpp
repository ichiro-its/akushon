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

#include <memory>
#include <string>
#include <thread>

#include "akushon/node/akushon_node.hpp"

#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/node/action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace akushon
{

AkushonNode::AkushonNode(rclcpp::Node::SharedPtr node)
: node(node), action_node(nullptr)
{
}

void AkushonNode::set_action_manager(std::shared_ptr<ActionManager> action_manager)
{
  action_node = std::make_shared<ActionNode>(node, action_manager);
}

void AkushonNode::run_config_service(const std::string & path)
{
  config_node = std::make_shared<ConfigNode>(node, path);
}

}  // namespace akushon
