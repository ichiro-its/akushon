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

#ifndef AKUSHON__ACTION__NODE__ACTION_MANAGER_HPP_
#define AKUSHON__ACTION__NODE__ACTION_MANAGER_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>

#include "akushon/action/model/action.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon/action/process/interpolator.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

class ActionManager
{
public:
  ActionManager();

  void insert_action(std::string action_name, const Action & action);
  void delete_action(std::string action_name);
  Action get_action(std::string action_name) const;

  void load_config(const std::string & path);
  void set_config(const nlohmann::json & json);

  Action load_action(const nlohmann::json & action_data, const std::string & action_name) const;

  void start(std::string action_name, const Pose & initial_pose);
  void start(std::string action_name, std::string target_action_name, const Pose & initial_pose, float ball_x, float right_map_x_min_, float right_map_x_max_, float left_map_x_min_, float left_map_x_max_, bool right);
  void start(const Action & action, const Pose & initial_pose);
  void brake();
  void process(double time);

  bool is_playing() const;

  std::vector<tachimawari::joint::Joint> get_joints() const;

  bool using_dynamic_kick;
  double right_map_x_min;
  double right_map_x_max;
  double left_map_x_min;
  double left_map_x_max;
private:
  std::map<std::string, Action> actions;

  std::shared_ptr<Interpolator> interpolator;
  bool is_running;
  std::string config_name;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__NODE__ACTION_MANAGER_HPP_
