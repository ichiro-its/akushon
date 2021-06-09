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
// #include <tachimawari_interfaces/srv/set_joints.hpp>
#include <tachimawari/joint.hpp>

#include <akushon/action.hpp>
#include <akushon/pose.hpp>

#include <string>
#include <map>
#include <vector>
#include <memory>

namespace akushon
{

class ActionManager
{
public:
  // explicit ActionManager(std::string node_name, std::string service_name);
  explicit ActionManager(std::vector<std::string> action_names = {});

  void insert_action(const uint8_t & id, const Action & action);
  void delete_action(const uint8_t & id);

  const Action & get_action_by_id(const uint8_t & id) const;

  void load_action_data(const std::string & path);
  void load_action_data(const std::string & path, const std::vector<std::string> & action_names);
  std::shared_ptr<Pose> run_action(const int & time);

  void set_current_action(const uint8_t & action_id, const Pose & pose);
  bool set_current_action(const std::string & action_name);
  bool set_current_action(const std::string & action_name, const Pose & pose);

  bool is_empty() const;
  bool is_running() const;
  bool is_ready() const;

  void clear_current_action();
  void clear_action_list();

  // std::shared_future<std::shared_ptr<tachimawari_interfaces::srv::SetJoints::Response>>
  // send_joints_request(std::vector<tachimawari::Joint> joints, float speed = 1);

private:
  std::vector<std::string> action_names;
  std::map<uint8_t, Action> action_list;
  std::shared_ptr<Action> current_action;
  std::shared_ptr<Pose> robot_pose;
  // std::shared_ptr<rclcpp::Client<tachimawari_interfaces::srv::SetJoints>> set_joints_client;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION_MANAGER_HPP_
