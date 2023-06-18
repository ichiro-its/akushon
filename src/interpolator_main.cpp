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

#include <memory>
#include <iostream>
#include <string>
#include <vector>

#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/model/action.hpp"
#include "akushon/action/model/action_name.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon/node/akushon_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/joint/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the path!" << std::endl;
    return 0;
  }

  auto action_manager = std::make_shared<akushon::ActionManager>();

  std::string path = argv[1];

  rclcpp::Rate rcl_rate(8ms);
  int time = 0;

  action_manager->load_config(path);
  akushon::Pose pose("init");

  {
    using tachimawari::joint::JointId;
    using tachimawari::joint::Joint;
    std::vector<Joint> joints;

    for (int id = 1; id < JointId::list.size(); id++) {
      Joint joint(id, 0);
      joints.push_back(joint);
    }
    pose.set_pause(0.0);
    pose.set_speed(0.0);
    pose.set_joints(joints);
  }

  action_manager->start(akushon::ActionName::WALKREADY, pose);
  while (rclcpp::ok()) {
    if (!action_manager->is_playing()) {
      break;
    }

    rcl_rate.sleep();
    action_manager->process(time);
    auto joints = action_manager->get_joints();

    for (const auto & joint : joints) {
      for (auto & i : tachimawari::joint::JointId::by_name) {
        if (i.second == static_cast<int>(joint.get_id())) {
          std::cout << i.first << " : ";
        }
      }
      std::cout << joint.get_position() << std::endl;
    }

    std::cout << std::endl;

    time += 8;
  }

  return 0;
}
