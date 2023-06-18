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
#include <string>

#include "akushon/config/node/config_node.hpp"

#include "akushon/config/utils/config.hpp"
#include "akushon_interfaces/srv/save_actions.hpp"
#include "akushon_interfaces/srv/get_actions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace akushon
{

ConfigNode::ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path)
: config_util(path)
{
  get_actions_service = node->create_service<GetActions>(
    get_node_prefix() + "/get_actions",
    [this](std::shared_ptr<GetActions::Request> request,
    std::shared_ptr<GetActions::Response> response) {
      response->json = this->config_util.get_config();
    }
  );

  save_actions_service = node->create_service<SaveActions>(
    get_node_prefix() + "/save_actions",
    [this](std::shared_ptr<SaveActions::Request> request,
    std::shared_ptr<SaveActions::Response> response) {
      this->config_util.save_config(request->json);
      response->status = "SAVED";
    }
  );
}

std::string ConfigNode::get_node_prefix() const
{
  return "akushon/config";
}

}  // namespace akushon
