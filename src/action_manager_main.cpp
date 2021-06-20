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
#include <keisan/keisan.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "Please specify the host, the port, and the path!" << std::endl;
    return 0;
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);
  std::string path = argv[3];

  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " <<
      client.get_port() << "!" << std::endl;

    return 1;
  }

  std::vector<std::string> action_names = {
    "init",
    "walkready",
    "left_kick",
    "right_kick",
    "back_standup",
    "front_standup",
    "left_standup",
    "right_standup",
    "keeper",
    "right_keeper",
    "left_keeper"
    // "catch_ball"
  };


  auto action_manager = std::make_shared<akushon::ActionManager>(action_names);

  robocup_client::MessageHandler message;
  message.add_sensor_time_step("neck_yaw_s", 8);
  message.add_sensor_time_step("neck_pitch_s", 8);
  message.add_sensor_time_step("left_shoulder_pitch_s", 8);
  message.add_sensor_time_step("left_shoulder_roll_s", 8);
  message.add_sensor_time_step("left_elbow_s", 8);
  message.add_sensor_time_step("right_shoulder_pitch_s", 8);
  message.add_sensor_time_step("right_shoulder_roll_s", 8);
  message.add_sensor_time_step("right_elbow_s", 8);
  message.add_sensor_time_step("left_hip_yaw_s", 8);
  message.add_sensor_time_step("left_hip_roll_s", 8);
  message.add_sensor_time_step("left_hip_pitch_s", 8);
  message.add_sensor_time_step("left_knee_s", 8);
  message.add_sensor_time_step("left_ankle_roll_s", 8);
  message.add_sensor_time_step("left_ankle_pitch_s", 8);
  message.add_sensor_time_step("right_hip_yaw_s", 8);
  message.add_sensor_time_step("right_hip_roll_s", 8);
  message.add_sensor_time_step("right_hip_pitch_s", 8);
  message.add_sensor_time_step("right_knee_s", 8);
  message.add_sensor_time_step("right_ankle_roll_s", 8);
  message.add_sensor_time_step("right_ankle_pitch_s", 8);

  client.send(*message.get_actuator_request());

  std::string cmds[3] = {};

  bool is_running = false;
  std::thread input_handler([&cmds, &is_running] {
      while (true) {
        if (!is_running) {
          std::cout << "> run : ";
          std::cin >> cmds[0];

          std::cout << "  action : ";
          std::cin >> cmds[1];

          if (cmds[0] == "pose") {
            std::cout << "  pose : ";
            std::cin >> cmds[2];
          } else {
            cmds[2] = "empty";
          }

          is_running = true;
        }
      }
    });

  while (client.get_tcp_socket()->is_connected()) {
    try {
      message.clear_actuator_request();

      auto sensors = client.receive();

      // get time
      auto time = sensors.get()->time();

      if (action_manager->is_running()) {
        auto robot_pose = action_manager->process(time);

        for (auto & joint : robot_pose->get_joints()) {
          std::string joint_name = joint.get_joint_name();

          if (joint_name.find("shoulder_pitch") != std::string::npos) {
            joint_name += " [shoulder]";
          } else if (joint_name.find("hip_yaw") != std::string::npos) {
            joint_name += " [hip]";
          }

          message.add_motor_position_in_degree(joint_name, joint.get_position());
        }

        client.send(*message.get_actuator_request());

        if (!action_manager->is_running()) {
          is_running = false;
        }
      } else if (!cmds[0].empty() && !cmds[1].empty() && !cmds[2].empty()) {
        if (cmds[0] == "q") {
          break;
        }

        action_manager->load_data(path, action_names);

        if (cmds[0] == "action" && !cmds[1].empty()) {
          std::vector<tachimawari::Joint> joints;
          akushon::Pose robot_pose("robot");

          for (int i = 0; i < sensors.get()->position_sensors_size(); i++) {
            auto position_sensor = sensors.get()->position_sensors(i);
            auto joint_name =
              position_sensor.name().substr(0, position_sensor.name().size() - 2);

            tachimawari::Joint joint(joint_name, position_sensor.value() * 180.0 / M_PI);

            joints.push_back(joint);
          }
          robot_pose.set_joints(joints);

          bool find_action = action_manager->set_current_action(cmds[1], robot_pose);

          if (!find_action) {
            std::cout << "-ERR action_name was not found" << std::endl;
          }
        } else if (cmds[0] == "pose" && !cmds[2].empty()) {
          bool find_action = false;
          for (uint id = 0; id < action_names.size(); id++) {
            if (cmds[1] == action_names[id]) {
              if (std::stoi(cmds[2]) >= 0) {
                std::cout << "Running pose " << cmds[2] << " of " << cmds[1] << std::endl;
                find_action = true;

                auto action = action_manager->get_action_by_id(id);
                auto current_pose = action->get_pose_by_index(std::stoi(cmds[2]));

                for (auto & joint : current_pose.get_joints()) {
                  std::string joint_name = joint.get_joint_name();

                  if (joint_name.find("shoulder_pitch") != std::string::npos) {
                    joint_name += " [shoulder]";
                  } else if (joint_name.find("hip_yaw") != std::string::npos) {
                    joint_name += " [hip]";
                  }
                  message.add_motor_position_in_degree(joint_name, joint.get_position());
                }

                client.send(*message.get_actuator_request());
                is_running = false;
              } else {
                std::cout << "-ERR step is not defined (step in range 0 - 6)" << std::endl;
              }

              break;
            }
          }
          if (!find_action) {
            std::cout << "-ERR action_name was not found" << std::endl;
            is_running = false;
          }
        } else {
          std::cout << "-ERR command was not valid\n" << std::endl;
          is_running = false;
        }
        cmds[0].clear();
        cmds[1].clear();
        cmds[2].clear();
      }
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}
