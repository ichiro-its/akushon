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

#include <unistd.h>
#include <limits.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "akushon/action/node/action_manager.hpp"

#include "akushon/action/model/action_name.hpp"
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

void ActionManager::insert_action(std::string action_name, const Action & action)
{
  actions.insert({action_name, action});
}

void ActionManager::delete_action(std::string action_name)
{
  actions.erase(action_name);
}

Action ActionManager::get_action(std::string action_name) const
{
  if (actions.find(action_name) != actions.end()) {
    return actions.at(action_name);
  }

  return Action("");
}

void ActionManager::load_config(const std::string & path)
{
  for (const auto & entry : std::filesystem::directory_iterator(path)) {
    std::string name = "";
    std::string file_name = entry.path();
    std::string extension_json = ".json";
    for (int i = path.length(); i < file_name.length() - extension_json.length(); i++) {
      name += file_name[i];
    }

    try {
      std::ifstream file(file_name);
      nlohmann::json action_data = nlohmann::json::parse(file);

      Action action = load_action(action_data, name);

      actions.insert({name, action});
    } catch (nlohmann::json::parse_error & ex) {
      std::cerr << "failed to load action: " << name << std::endl;
      std::cerr << "parse error at byte " << ex.byte << std::endl;
      throw ex;
    }
  }
}

Action ActionManager::load_action(const std::string & path, const std::string & action_name)
{
  std::ifstream file(path + action_name + ".json");
  nlohmann::json action_data = nlohmann::json::parse(file);

  return load_action(action_data, action_name);
}

Action ActionManager::load_action(
  const nlohmann::json & action_data,
  const std::string & action_name)
{
  Action action = Action(action_name);

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
            if (joint_key.compare(0, 4, "neck") == 0) {
              continue;
            }

            Joint joint(JointId::by_name.at(joint_key), joint_val);
            joints.push_back(joint);
          }

          pose.set_pause(raw_pose["pause"]);
          pose.set_speed(raw_pose["speed"]);
          pose.action_time = raw_pose["time"];
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
    } else if (key == "time_based") {
      action.time_based = val;
    }
  }

  return action;
}

void ActionManager::start(std::string action_name, const Pose & initial_pose)
{
  std::vector<Action> target_actions;

  while (true) {
    const auto & action = actions.at(action_name);

    target_actions.push_back(action);

    if (action.get_next_action() != "") {
      std::string next_action_name = action.get_next_action();

      if (actions.find(next_action_name) != actions.end()) {
        action_name = next_action_name;
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
  std::vector<Action> target_actions {action};

  interpolator = std::make_shared<Interpolator>(target_actions, initial_pose);
  is_running = true;
}

void ActionManager::cancel(const Pose & initial_pose)
{
  if (!interpolator) {
    return;
  }

  auto current_actions = interpolator->get_actions();
  int action_idx = interpolator->get_current_action_index();
  int pose_idx  = interpolator->get_current_pose_index();

  auto & first_action = current_actions.front();
  bool is_kick_motion = first_action.get_name().find("kick") != std::string::npos;

  if (is_kick_motion && (action_idx > 0 || pose_idx > 2)) {
    return;
  }

  brake();

  std::vector<Action> cancel_actions;

  if (is_kick_motion) {
    int pose_count = first_action.get_pose_count();

    if (pose_idx == 2) {
      Action cancel_action(first_action.get_name() + "_cancel");

      auto new_pose = first_action.get_pose(pose_count - 2);
      new_pose.action_time = 0.5;
      cancel_action.add_pose(new_pose);

      cancel_action.set_start_delay(0);
      cancel_action.set_stop_delay(0);
      cancel_action.time_based = first_action.time_based;

      cancel_actions.push_back(cancel_action);
    }
  } else {
    if (action_idx < static_cast<int>(current_actions.size())) {
      const auto & action = current_actions[action_idx];

      Action cancel_action(action.get_name() + "_reversed");

      for (int i = pose_idx; i >= 0; --i) {
        cancel_action.add_pose(action.get_pose(i));
      }

      cancel_action.set_start_delay(0);
      cancel_action.set_stop_delay(0);
      cancel_action.time_based = action.time_based;

      cancel_actions.push_back(cancel_action);
    }

    for (int i = action_idx - 1; i >= 0; --i) {
      const auto & action = current_actions[i];

      Action cancel_action(action.get_name() + "_reversed");

      for (int idx = action.get_pose_count() - 1; idx >= 0; --i) {
        cancel_action.add_pose(action.get_pose(idx));
      }

      cancel_action.set_start_delay(0);
      cancel_action.set_stop_delay(0);
      cancel_action.time_based = action.time_based;

      cancel_actions.push_back(cancel_action);
    }
  }

  auto walk_ready = actions.at(akushon::ActionName::WALKREADY);
  auto new_pose = walk_ready.get_pose(0);
  new_pose.action_time = 0.5;
  walk_ready.set_pose(0, new_pose);

  cancel_actions.push_back(walk_ready);

  interpolator = std::make_shared<Interpolator>(cancel_actions, initial_pose);
  is_running = true;
}

void ActionManager::process(double time)
{
  if (interpolator) {
    interpolator->process(time);

    if (interpolator->is_finished()) {
      interpolator = nullptr;
    }
  } else {
    is_running = false;
  }
}

void ActionManager::brake()
{
  interpolator = nullptr;
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
