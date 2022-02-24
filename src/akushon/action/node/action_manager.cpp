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
  nlohmann::json actions_list;
  for (const auto & [name, id] : ActionName::map) {
    std::string file_name = path + "/action/" + name + ".json";
    Action action = Action(name);

    try {
      std::ifstream file(file_name);
      nlohmann::json action_data = nlohmann::json::parse(file);

      actions_list["action_" + name] = action_data;
      action.set_name(action_data["name"]);

      for (const auto & [key, val] : action_data.items()) {
        if (key.find("step_") != std::string::npos) {
          {
            using tachimawari::joint::JointId;
            using tachimawari::joint::Joint;

            Pose pose(key);
            std::vector<Joint> joints;

            for (const auto & [steps_key, steps_val] : action_data[key].items()) {
              bool add_joint = steps_key.find("step_") == std::string::npos;
              add_joint &= JointId::by_name.find(steps_key) != JointId::by_name.end();

              if (add_joint) {
                Joint joint(JointId::by_name.at(steps_key), steps_val);

                joints.push_back(joint);
              } else if (steps_key == "step_pause") {
                pose.set_pause(steps_val);
              } else if (steps_key == "step_speed") {
                pose.set_speed(steps_val);
              }
            }

            pose.set_joints(joints);
            action.add_pose(pose);
          }
        } else if (key == "start_delay") {
          action.set_start_delay(val);
        } else if (key == "stop_delay") {
          action.set_stop_delay(val);
        } else if (key == "next_action") {
          action.set_next_action(val);
        }
      }

      actions.insert({id, action});
    } catch (nlohmann::json::parse_error & ex) {
      // TODO(maroqijalil): will be used for logging
      // std::cerr << "parse error at byte " << ex.byte << std::endl;
    }

  }
  this->actions_list = actions_list.dump();
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

std::string ActionManager::get_actions_list() const
{
  return actions_list;
}

}  // namespace akushon
