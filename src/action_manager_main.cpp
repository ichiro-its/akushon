// Copyright (c) 2021 ICHIRO ITS
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

#include <robocup_client/robocup_client.hpp>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <akushon/action_manager.hpp>
#include <akushon/pose.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

std::map<uint8_t, std::shared_ptr<akushon::Action>> load_action_data(
  std::vector<std::string> action_names);

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::cerr << "Please specify the host and the port!" << std::endl;
    return 0;
  }
  std::string host = argv[1];
  int port = std::stoi(argv[2]);

  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " <<
      client.get_port() << "!" << std::endl;

    return 1;
  }

  std::vector<std::string> action_names = {
    // "left_kick",
    "right_kick",
    // "back_standup",
    // "forward_standup",
    // "left_standup",
    // "right_standup"
  };

  robocup_client::MessageHandler message;

  client.send(*message.get_actuator_request());

  while (client.get_tcp_socket()->is_connected()) {
    try {
      std::map<uint8_t, std::shared_ptr<akushon::Action>> action_list;

      std::string cmds[2];
      std::cin >> cmds[0] >> cmds[1];

      if (cmds[0] == "q") {
        break;
      }

      std::cout << "cmd: " << cmds[0] << " " << cmds[1] << std::endl;
      action_list = load_action_data(action_names);

      if (cmds[0] == "run_action") {
        bool find_action = false;
        for (auto id = 0; id < action_names.size(); id++) {
          if (cmds[1] == action_names[id]) {
            find_action = true;

            auto action = action_list[id];

            while (!action->is_finished()) {
              akushon::Pose pose = action->get_pose();
              std::cout << "joints at pose: " << pose.get_name() << "\n";

              std::vector<tachimawari::Joint> joints = pose.get_joints();
              std::cout << "JOINTS" << std::endl;
              for (auto & joint : joints) {
                std::string joint_name = joint.get_joint_name();

                if (joint_name.find("shoulder_pitch") != std::string::npos) {
                  joint_name += " [shoulder]";
                  // std::cout << joint_name << std::endl;
                } else if (joint_name.find("hip_yaw") != std::string::npos) {
                  joint_name += " [hip]";
                  // std::cout << "hip" << std::endl;
                }

                std::cout << "joints: " << joint_name << " at " << joint.get_position() << "\n";
                message.add_motor_position(joint_name, joint.get_position());
              }
              client.send(*message.get_actuator_request());

              action->next_pose();
            }
            break;  // done pose
          }
        }
        // if doesn't find the action_name
        if (!find_action) {
          std::cout << "-ERR action_name was not found\n usage: run_action <action_name>" <<
            std::endl;
        }
      } else {
        std::cout << "-ERR first command is not valid\n usage: run_action <action_name>" <<
          std::endl;
      }

      auto sensors = client.receive();

      // Get Position Sensor Data
      for (int i = 0; i < sensors.get()->position_sensors_size(); i++) {
        auto position_sensor = sensors.get()->position_sensors(i);
        std::cout << position_sensor.name() << " " << position_sensor.value() << std::endl;
      }
      std::cout << std::endl;

      // Get time
      auto time = sensors.get()->time();
      std::cout << time << std::endl;
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}

std::map<uint8_t, std::shared_ptr<akushon::Action>> load_action_data(
  std::vector<std::string> action_names)
{
  uint8_t id = 0;
  std::map<uint8_t, std::shared_ptr<akushon::Action>> action_list;
  for (auto action_name : action_names) {
    std::string file_name = "/home/nathanael/ICHIRO/src/akushon/src/" + action_name + ".json";
    std::ifstream file(file_name);
    nlohmann::json action_data = nlohmann::json::parse(file);

    auto action = std::make_shared<akushon::Action>(action_data["name"]);

    for (auto &[key, val] : action_data.items()) {
      if (key.find("step_") != std::string::npos) {
        akushon::Pose pose(key);
        std::vector<tachimawari::Joint> joints;
        for (auto &[steps_key, steps_val] : action_data[key].items()) {
          if (!(steps_key.find("step_") != std::string::npos)) {
            tachimawari::Joint joint(steps_key, static_cast<float>(steps_val));  // init join
            joints.push_back(joint);
          } else if (steps_key == "step_pause") {
            pose.set_pause(static_cast<float>(steps_val));
          } else if (steps_key == "step_speed") {
            pose.set_speed(static_cast<float>(steps_val));
          }
        }
        pose.set_joints(joints);
        action->insert_pose(pose);
      }
    }
    action_list.insert(std::pair<uint8_t, std::shared_ptr<akushon::Action>>(id, action));
    // std::cout << "size" << action_list.size() << std::endl;
    id++;
  }

  return action_list;
}
