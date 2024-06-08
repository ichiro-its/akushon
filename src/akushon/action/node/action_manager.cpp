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
: actions({}), is_running(false), config_name("dynamic_kick.json"), action_dir("actions/")
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
  try {
    std::ifstream file(path + config_name);
    nlohmann::json data = nlohmann::json::parse(file);

    set_config(data);

    file.close();
  } catch (nlohmann::json::parse_error & ex) {
      std::cerr << "failed to load action: " << config_name << std::endl;
      std::cerr << "parse error at byte " << ex.byte << std::endl;
      throw ex;
  }

  for (const auto & entry : std::filesystem::directory_iterator(path + action_dir)) {
    std::string name = "";
    std::string file_name = entry.path();
    std::string extension_json = ".json";
    for (int i = (path + action_dir).length(); i < file_name.length() - extension_json.length(); i++) {
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

void ActionManager::set_config(const nlohmann::json & json)
{
  for (auto &[key, val] : json.items()) {
    if (key == "dynamic_kick") {
      try {
        val.at("right_map_x_min").get_to(right_map_x_min);
        val.at("right_map_x_max").get_to(right_map_x_max);
        val.at("left_map_x_min").get_to(left_map_x_min);
        val.at("left_map_x_max").get_to(left_map_x_max);
        val.at("using_dynamic_kick").get_to(using_dynamic_kick);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "error key: " << key << std::endl;
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    }
  }
}

Action ActionManager::load_action(
  const nlohmann::json & action_data,
  const std::string & action_name) const
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

void ActionManager::start(std::string action_name, std::string target_action_name, const Pose & initial_pose, float ball_x, float right_map_x_min_, float right_map_x_max_, float left_map_x_min_, float left_map_x_max_, bool right)
{
  std::vector<Action> target_actions;

  while (true) {
    auto & action = actions.at(action_name);
    const auto & target_action = actions.at(target_action_name);

    for (int pose_index = 0; pose_index < action.get_pose_count(); pose_index++) {
      if (right)
      {
        RCLCPP_INFO(rclcpp::get_logger("DEBUG ACTION MANAGER"), "CEK1");
        action.map_action(action, target_action, pose_index, ball_x, right_map_x_min_, right_map_x_max_);
      } else {
        RCLCPP_INFO(rclcpp::get_logger("DEBUG ACTION MANAGER"), "CEK1");
        action.map_action(action, target_action, pose_index, ball_x, left_map_x_min_, left_map_x_max_);
      }
    }

    target_actions.push_back(action);
    RCLCPP_INFO(rclcpp::get_logger("DEBUG ACTION MANAGER"), "CEK");
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
