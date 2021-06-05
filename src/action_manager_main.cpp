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
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::cerr << "Please specify the host, the port, and the path!" << std::endl;
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

  auto action_manager = std::make_shared<akushon::ActionManager>();

  std::vector<std::string> action_names = {
    // "left_kick",
    "right_kick",
    // "back_standup",
    // "forward_standup",
    // "left_standup",
    // "right_standup"
  };

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

  bool action_is_running = false;
  std::thread input_handler([&cmds, &action_is_running] {
      while (true) {
        if (!action_is_running) {
          std::cout << "> run ";
          std::cin >> cmds[0] >> cmds[1];
          action_is_running = true;
        }
      }
    });

  while (client.get_tcp_socket()->is_connected()) {
    try {
      message.clear_actuator_request();

      auto sensors = client.receive();

      // get time
      auto time = sensors.get()->time();

      if (!action_manager->is_empty()) {
        auto robot_pose = action_manager->run_action(time);

        for (auto & joint : robot_pose->get_joints()) {
          std::string joint_name = joint.get_joint_name();

          if (joint_name.find("shoulder_pitch") != std::string::npos) {
            joint_name += " [shoulder]";
          } else if (joint_name.find("hip_yaw") != std::string::npos) {
            joint_name += " [hip]";
          }
          message.add_motor_position(joint_name, joint.get_position() / 180 * 3.14);
        }

        client.send(*message.get_actuator_request());

        if (action_manager->is_empty()) {
          action_is_running = false;
        }
      } else if (!cmds[0].empty()) {
        if (cmds[0] == "q") {
          break;
        }

        action_manager->load_action_data(argv[3], action_names);

        if (cmds[0] == "action") {
          bool find_action = false;
          for (uint id = 0; id < action_names.size(); id++) {
            if (cmds[1] == action_names[id]) {
              std::cout << "Running action " << cmds[1] << std::endl;
              find_action = true;
              akushon::Pose init_pose("init");
              std::vector<tachimawari::Joint> joints;

              for (int i = 0; i < sensors.get()->position_sensors_size(); i++) {
                auto position_sensor = sensors.get()->position_sensors(i);
                auto joint_name =
                  position_sensor.name().substr(0, position_sensor.name().size() - 2);

                tachimawari::Joint joint(joint_name, position_sensor.value());

                joints.push_back(joint);
              }
              init_pose.set_joints(joints);
              action_manager->set_current_action(id, init_pose);
              break;
            }
          }
          if (!find_action) {
            std::cout << "-ERR action_name was not found" << std::endl;
          }
        } else if (cmds[0] == "pose") {
          std::cout << "action > ";
          std::cin >> cmds[1];
          std::cout << "step > ";
          std::cin >> cmds[2];
 
          bool find_action = false;
          for (uint id = 0; id < action_names.size(); id++) {
            if (cmds[1] == action_names[id]) {
              if (std::stoi(cmds[2]) >= 0 &&  std::stoi(cmds[2]) <= 6) {
                std::cout << "Running action " << cmds[1] << "in pose - " << cmds[2] << std::endl;
                find_action = true; 

                std::shared_ptr<akushon::Action> current_action = ;
                akushon::Pose current_pose = action_manager->get_action(id)->get_pose_at_index(std::stoi(cmds[2]));

                 for (auto & joint : current_pose->get_joints()) {
                  std::string joint_name = joint.get_joint_name();

                  if (joint_name.find("shoulder_pitch") != std::string::npos) {
                    joint_name += " [shoulder]";
                  } else if (joint_name.find("hip_yaw") != std::string::npos) {
                    joint_name += " [hip]";
                  }
                  message.add_motor_position(joint_name, joint.get_position() / 180 * 3.14);
                }

                client.send(*message.get_actuator_request());
              } else {
                std::cout << "-ERR step is not defined (step in range 0 - 6)" << std::endl;
              }
              
              break;
            }
          }
          if (!find_action) {
            std::cout << "-ERR action_name was not found" << std::endl;
          }
        } else {
          std::cout << "-ERR command was not valid\n usage: run_action <action_name>" << std::endl;
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
