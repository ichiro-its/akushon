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

#include "stdlib.h"  // NOLINT
#include "unistd.h"  // NOLINT

namespace akushon
{

ActionManager::ActionManager()
: actions({}), interpolator(nullptr), is_running(false)
{
}

void ActionManager::insert_action(int index, const Action & action)
{
  actions.insert({index, action});
}

void ActionManager::delete_action(int index)
{
  actions.erase(index);
}

const Action & ActionManager::get_action(int index) const
{
  return actions.at(index);
}

const std::vector<tachimawari::joint::Joint> & ActionManager::get_joints() const
{
}

void ActionManager::load_data(const std::string & path)
{
  for (auto [name, id] : ActionName::map) {
    std::string file_name = path + "/action/" + name + ".json";
    Action action = Action(name);

    bool is_load_success = false;
    try {
      std::ifstream file(path);
      nlohmann::json action_data = nlohmann::json::parse(file);

      action.set_name(action_data["name"]);

      for (auto [key, val] : action_data.items()) {
        if (key.find("step_") != std::string::npos) {
          Pose pose(key);
          std::vector<tachimawari::joint::Joint> joints;

          for (auto [steps_key, steps_val] : action_data[key].items()) {
            if (!(steps_key.find("step_") != std::string::npos)) {
              tachimawari::joint::Joint joint(
                tachimawari::joint::JointId::by_name[steps_key],
                static_cast<float>(steps_val));

              joints.push_back(joint);
            } else if (steps_key == "step_pause") {
              pose.set_pause(static_cast<float>(steps_val));
            } else if (steps_key == "step_speed") {
              pose.set_speed(static_cast<float>(steps_val));
            }
          }

          pose.set_joints(joints);
          action.add_pose(pose);
        } else if (key == "start_delay") {
          action.set_start_delay(val);
        } else if (key == "stop_delay") {
          action.set_stop_delay(val);
        } else if (key == "next_action") {
          action.set_next_action(val);
        }
      }

      is_load_success = true;
    } catch (nlohmann::json::parse_error & ex) {
      // TODO(maroqijalil): will be used for logging
      // std::cerr << "parse error at byte " << ex.byte << std::endl;
    }

    if (is_load_success) {
      actions.insert({id, action});
    }
  }
}

void ActionManager::start(int action_id, const Pose & initial_pose)
{
  if (actions.find(action_id) != actions.end()) {
    std::vector<Action> target_actions;
    
    while (true) {
      target_actions.push_back(actions[action_id]);

      if (actions[action_id].get_next_action() != "") {
        int next_action_id = ActionName::map.at(actions[action_id].get_next_action());

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
}

void ActionManager::process(int time)
{
  if (is_running && interpolator) {
    interpolator->process(time);

    if (interpolator->is_finished()) {
      is_running = false;
    }
  }
}

bool ActionManager::is_running() const
{
  return is_running;
}

// std::shared_future<std::shared_ptr<tachimawari_interfaces::srv::SetJoints::Response>>
// ActionManager::send_joints_request(std::vector<tachimawari::Joint> joints, float speed)
// {
//   {
//     using SetJoints = tachimawari_interfaces::srv::SetJoints;

//     auto request = std::make_shared<SetJoints::Request>();
//     std::vector<tachimawari_interfaces::msg::Joint> joint_messages;

//     for (auto joint : joints) {
//       tachimawari_interfaces::msg::Joint joint_message;
//       joint_message.name = joint.get_joint_name();
//       joint_message.position = joint.get_goal_position();
//       joint_message.speed = speed;

//       joint_messages.push_back(joint_message);
//     }

//     request->joints = joint_messages;

//     return set_joints_client->async_send_request(request);
//   }
// }

}  // namespace akushon
