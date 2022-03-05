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

  void insert_action(int action_id, const Action & action);
  void delete_action(int action_id);
  Action get_action(int action_id) const;

  void load_data(const std::string & path);
  void save_data(const nlohmann::json & actions_data);

  void start(int action_id, const Pose & initial_pose);
  void process(int time);

  bool is_playing() const;

  std::vector<tachimawari::joint::Joint> get_joints() const;
  
  std::string get_actions_list() const;

private:
  std::map<int, Action> actions;
  std::string actions_list;
  std::string path_data;

  std::shared_ptr<Interpolator> interpolator;
  bool is_running;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__NODE__ACTION_MANAGER_HPP_
