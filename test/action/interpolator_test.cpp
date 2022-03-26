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

#include "gtest/gtest.h"

#include "akushon/action/model/action_name.hpp"
#include "akushon/action/model/action.hpp"
#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/process/interpolator.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/joint.hpp"

TEST(InterpolatorTest, JointTest) {
  akushon::ActionManager action_manager;
  int joint_tolerance = 4;

  action_manager.load_data("../../src/akushon/data/test");

  std::vector<akushon::Pose> poses = action_manager.get_action(akushon::Action::RIGHT_KICK).get_poses();

  action_manager.start(akushon::Action::RIGHT_KICK, poses[0]);

  std::vector<tachimawari::joint::Joint> expected_joints = poses[0].get_joints();
  std::vector<tachimawari::joint::Joint> action_manager_joints = action_manager.get_joints();

  for (int i = 0; i < 20; i++){
    ASSERT_TRUE(expected_joints[i].get_position() + joint_tolerance > action_manager_joints[i].get_position()
       && expected_joints[i].get_position() - joint_tolerance  < action_manager_joints[i].get_position());
  }

  for (int i = 0; i < 1100; i+=1){
    action_manager.process(i);
  }

  expected_joints = poses[1].get_joints();
  action_manager_joints = action_manager.get_joints();

  for (int i = 0; i < 20; i++)
  {
    ASSERT_TRUE(expected_joints[i].get_position() + joint_tolerance > action_manager_joints[i].get_position() 
      && expected_joints[i].get_position() - joint_tolerance < action_manager_joints[i].get_position());
  }

  for (int i = 1100; i < 2000; i++){
    action_manager.process(i);
  }

  expected_joints = poses[2].get_joints();
  action_manager_joints = action_manager.get_joints();

  for (int i = 0; i < 20; i++)
  {
    ASSERT_TRUE(expected_joints[i].get_position() + joint_tolerance > action_manager_joints[i].get_position()
       && expected_joints[i].get_position() - joint_tolerance  < action_manager_joints[i].get_position());
  }
}
