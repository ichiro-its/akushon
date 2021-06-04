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
#include <utility>
#include <vector>

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

  auto action_manager =
    std::make_shared<akushon::ActionManager>("action_manager", "motion_manager");

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
  message.add_sensor_time_step("left_shoulder_pitch", 8);
  message.add_sensor_time_step("left_shoulder_roll", 8);
  message.add_sensor_time_step("left_elbow", 8);
  message.add_sensor_time_step("right_shoulder_pitch", 8);
  message.add_sensor_time_step("right_shoulder_roll", 8);
  message.add_sensor_time_step("right_elbow", 8);
  message.add_sensor_time_step("left_hip_yaw", 8);
  message.add_sensor_time_step("left_hip_roll", 8);
  message.add_sensor_time_step("left_hip_pitch", 8);
  message.add_sensor_time_step("left_knee", 8);
  message.add_sensor_time_step("left_ankle_roll", 8);
  message.add_sensor_time_step("left_ankle_pitch", 8);
  message.add_sensor_time_step("right_hip_yaw", 8);
  message.add_sensor_time_step("right_hip_roll", 8);
  message.add_sensor_time_step("right_hip_pitch", 8);
  message.add_sensor_time_step("right_knee", 8);
  message.add_sensor_time_step("right_ankle_roll", 8);
  message.add_sensor_time_step("right_ankle_pitch", 8);

  client.send(*message.get_actuator_request());

  while (client.get_tcp_socket()->is_connected()) {
    try {
      message.clear_actuator_request();

      auto sensors = client.receive();
      
      // get time
      auto time = sensors.get()->time();
      std::cout << time << std::endl;

      if (action_manager->is_running()) { //
        auto robot_pose = action_manager->run_action(); //

        for (auto & joint : robot_pose->get_joints()) {
          std::string joint_name = joint.get_joint_name();

          if (joint_name.find("shoulder_pitch") != std::string::npos) {
            joint_name += " [shoulder]";
          } else if (joint_name.find("hip_yaw") != std::string::npos) {
            joint_name += " [hip]";
          }
          message.add_motor_position(joint_name, joint.get_position());
        }

        client.send(*message.get_actuator_request());
        std::cout << "send" << std::endl;
      } else {
        std::string cmds[2];
        std::cin >> cmds[0] >> cmds[1];

        if (cmds[0] == "q") {
          break;
        }

        action_manager->load_action_data(action_names);

        if (cmds[0] == "run_action") {
          bool find_action = false;
          for (auto id = 0; id < action_names.size(); id++) {
            if (cmds[1] == action_names[id]) {
              find_action = true;
              akushon::Pose init_pose("init");
              std::vector<tachimawari::Joint> joints;

              for (int i = 0; i < sensors.get()->position_sensors_size(); i++) {
                auto position_sensor = sensors.get()->position_sensors(i);
                tachimawari::Joint joint(position_sensor.name(), position_sensor.value());
                joints.push_back(joint);
              }
              init_pose.set_joints(joints);
              action_manager->set_current_action(id, init_pose); // 
            }
            break;  // done pose
          }
          if (!find_action) {
            std::cout << "-ERR action_name was not found" <<  std::endl;
          }
        } else {
            std::cout << "-ERR command was not valid\n usage: run_action <action_name>" <<  std::endl;
        }

      }
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}