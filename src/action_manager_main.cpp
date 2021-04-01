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

#include <rclcpp/rclcpp.hpp>

#include <akushon/action_manager.hpp>
#include <akushon/pose.hpp>

#include <iostream>
#include <string>
#include <memory>

int main(int argc, char * argv[])
{
  // init rclcpp
  rclcpp::init(argc, argv);

  // init node
  auto action_manager =
    std::make_shared<akushon::ActionManager>("action_manager", "motion_manager");

  // declare action id
  int action_id = 0;
  if (argc > 1) {
    action_id = std::stoi(argv[1]);
  }

  // !! need to load action list before get the desired action
  auto action = action_manager->get_action(action_id);

  // move joints
  while (action->is_finished()) {
    akushon::Pose pose = action->get_pose();

    // check the node
    if (action_manager->is_ready()) {
      std::cout << "\033[H\033[J";

      // send request
      auto response_future =
        action_manager->send_joints_request(pose.get_joints(), pose.get_speed());

      // wait for response
      if (rclcpp::spin_until_future_complete(action_manager, response_future) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        auto response = response_future.get();
        if (static_cast<bool>(response->status)) {
          action->next_pose();
        } else {
          std::cout << "failed to move joints at pose: " << pose.get_name() << "\n";
          break;
        }
      }
    } else {
      std::cout << "waiting...\n";
    }
  }

  rclcpp::shutdown();
}
