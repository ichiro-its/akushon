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

#ifndef AKUSHON__ACTION_MANAGER_HPP_
#define AKUSHON__ACTION_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tachimawari_interfaces/srv/set_joints.hpp>
#include <tachimawari/joint.hpp>

#include <akushon/action.hpp>
#include <akushon/pose.hpp>

#include <string>
#include <map>
#include <memory>

namespace akushon
{

class ActionManager : public rclcpp::Node
{
public:
  explicit ActionManager(std::string node_name, std::string service_name);

  void insert_action(uint8_t id, std::shared_ptr<Action> action);
  void delete_action(uint8_t id);
  
  bool start_action(uint8_t id);

private:
  bool send_joints_request(std::vector<tachimawari::Joint> joints, float speed = 1, bool & request_status);

  std::map<uint8_t, std::shared_ptr<Action>> action_list;

  std::shared_ptr<rclcpp::Client<tachimawari_interfaces::srv::SetJoints>>
  set_joints_client;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION_MANAGER_HPP_
