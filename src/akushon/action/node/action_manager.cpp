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

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "akushon/action/model/action_name.hpp"
#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/process/interpolator.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/joint.hpp"

namespace akushon
{

ActionManager::ActionManager()
: actions({}), is_running(false)
{
  interpolator = std::make_shared<Interpolator>(std::vector<Action>(), Pose(""));
}

void ActionManager::insert_action(int action_id, const Action & action)
{
  actions.insert({action_id, action});
}

void ActionManager::delete_action(int action_id)
{
  actions.erase(action_id);
}

Action ActionManager::get_action(int action_id) const
{
  if (actions.find(action_id) != actions.end()) {
    return actions.at(action_id);
  }

  return Action("");
}

void ActionManager::load_data(const std::string & path)
{
  for (const auto & [name, id] : ActionName::map) {
    std::string file_name = path + "/action/" + name + ".json";

    try {
      std::ifstream file(file_name);
      nlohmann::json action_data = nlohmann::json::parse(file);

      Action action = load_action(action_data, name);

      actions.insert({id, action});
    } catch (nlohmann::json::parse_error & ex) {
      // TODO(maroqijalil): will be used for logging
      // std::cerr << "parse error at byte " << ex.byte << std::endl;
    }
  }
}

Action ActionManager::load_action(
  const nlohmann::json & action_data,
  const std::string & action_name) const
{
  Action action = Action(action_name);

  try {
    action.set_name(action_data["name"]);

    for (const auto & [key, val] : action_data.items()) {
      if (key.find("poses") != std::string::npos) {
        for (const auto & raw_pose : action_data["poses"]) {
          {
            using tachimawari::joint::JointId;
            using tachimawari::joint::Joint;

            Pose pose(raw_pose["name"]);
            std::vector<Joint> joints;

            for (const auto & [joint_key, joint_val] : raw_pose["joints"].items()) {
              Joint joint(JointId::by_name.at(joint_key), joint_val);
              joints.push_back(joint);
            }

            pose.set_pause(raw_pose["pause"]);
            pose.set_speed(raw_pose["speed"]);
            pose.set_joints(joints);
            action.add_pose(pose);
          }
        }
      } else if (key == "start_delay") {
        action.set_start_delay(val);
      } else if (key == "stop_delay") {
        action.set_stop_delay(val);
      } else if (key == "next") {
        action.set_next_action(val);
      }
    }
  } catch (nlohmann::json::parse_error & ex) {
    // TODO(maroqijalil): will be used for logging
    // std::cerr << "parse error at byte " << ex.byte << std::endl;
  }

  return action;
}

void ActionManager::start(int action_id, const Pose & initial_pose)
{
  std::vector<Action> target_actions;

  while (true) {
    const auto & action = actions.at(action_id);

    target_actions.push_back(action);

    if (action.get_next_action() != "") {
      int next_action_id = ActionName::map.at(action.get_next_action());

      if (actions.find(next_action_id) != actions.end()) {
        action_id = next_action_id;
      } else {
        break;
      }
    } else {
      break;
    }
  }

  interpolator = std::make_shared<Interpolator>(target_actions, initial_pose);
  is_running = true;
}

void ActionManager::start(const Action & action, const Pose & initial_pose)
{
  bool temporary = true;
  std::vector<Action> target_actions;
  target_actions.push_back(action);

  interpolator = std::make_shared<Interpolator>(target_actions, initial_pose);
  is_running = true;
}

void ActionManager::process(int time)
{
  interpolator->process(time);

  if (interpolator->is_finished()) {
    is_running = false;
  }
}

bool ActionManager::is_playing() const
{
  return is_running;
}

std::vector<tachimawari::joint::Joint> ActionManager::get_joints() const
{
  if (interpolator) {
    return interpolator->get_joints();
  }

  return {};
}

}  // namespace akushon
