// Copyright (c) 2021-2024 Ichiro ITS
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

#include "akushon/action/process/moveit.hpp"

#include <keisan/keisan.hpp>

#include <cmath>
#include <string>
#include <vector>


namespace akushon
{

const std::array<std::string, 22> jointname = {
    "right_shoulder_pitch", // 1
    "left_shoulder_pitch", // 2
    "right_shoulder_roll", // 3
    "left_shoulder_roll", // 4
    "right_elbow", // 5
    "left_elbow",
    "right_hip_yaw",
    "left_hip_yaw", // 8
    "right_hip_roll", // 9
    "left_hip_roll", // 10
    "right_hip_pitch", // 11
    "left_hip_pitch", // 12
    "right_knee",
    "left_knee", // 14
    "right_ankle_pitch",
    "left_ankle_pitch", 
    "right_ankle_roll", // 17
    "left_ankle_roll",
    "neck_yaw", // 19
    "neck_pitch",
    "new_joint_1",
    "new_joint_2"
};

Moveit::Moveit(
  const std::vector<Action> & actions, const Pose & initial_pose,
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface)
: actions(actions), finished(false), move_group_interface(move_group_interface), initial_pose(initial_pose)
{
}

void Moveit::process(double time) {
    auto default_target = move_group_interface->getNamedTargetValues("init");

    move_group_interface->setMaxVelocityScalingFactor(initial_pose.get_speed());
    // TODO: add acceleration scaling factor on pose model
    move_group_interface->setMaxAccelerationScalingFactor(0.1);

    // pocess initial pose  first
    for (auto & joint : initial_pose.get_joints()) {
        if (joint.get_id() > 20) continue;

        double value;
        switch (joint.get_id()) {
            case 9: {
                value = keisan::map(joint.get_position(), -45.0f, 90.0f, -0.7853981633974483f, 1.5707963267948966f);
                break;
            }
            case 10: {
                value = keisan::map(joint.get_position(), -1.57f, 1.57f, -1.57f, 1.57f);
                break;
            }
            case 11: {
                value = keisan::map(joint.get_position(), -112.5f, 67.5f, -1.9634954084936207f, 1.1780972450961724f);
                break;
            }
            default: {
                value = 0.0f;
                break;
            }
        }

        default_target[jointname[joint.get_id()-1]] = value;
    }
    move_group_interface->setJointValueTarget(default_target);
    auto err = move_group_interface->move();

    // warn if error
    if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("akushon"), "Failed to move to initial pose");
    }

    finished = true;
}

bool Moveit::is_finished() { return finished; }

}  // namespace akushon
