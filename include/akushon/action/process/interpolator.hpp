

void Pose::interpolate()
{
  for (auto & current_joint : joints) {
    current_joint.interpolate();
  }
}

void Pose::set_target_position(const Pose & target_pose)
{
  std::vector<tachimawari::Joint> new_joints;
  for (auto & joint : get_joints()) {
    for (auto & target_joint : target_pose.get_joints()) {
      if (joint.get_joint_name() == target_joint.get_joint_name()) {
        tachimawari::Joint new_joint(joint.get_joint_name(), joint.get_position());
        new_joint.set_target_position(
          target_joint.get_position(), target_pose.get_speed());
        new_joints.push_back(new_joint);
      }
    }
  }

  set_joints(new_joints);
}

Pose Action::process(Pose robot_pose, const int & time)
{
  if (!is_start && !on_process) {
    start_stop_time = time;
    is_start = true;
  }

  if (time - start_stop_time > 1000 && is_start) {
    auto target_pose = get_current_pose();

    if (!on_process) {
      on_process = true;
      robot_pose.set_target_position(get_current_pose());
    }

    if (robot_pose == target_pose) {
      if (!on_pause) {
        pause_start_time = time;
        on_pause = true;
      }

      if (time - pause_start_time >= get_current_pose().get_pause() * 1000) {
        next_pose();
        on_pause = false;

        if (current_pose_index == pose_count) {
          start_stop_time = time;
          is_start = false;
        } else {
          robot_pose.set_target_position(get_current_pose());
        }
      }
    }

    if (!on_pause) {
      robot_pose.interpolate();
    }
  } else if (time - start_stop_time > 2000) {
    on_process = false;
    current_pose_index = 0;
  }

  return robot_pose;
}

void Action::load_data(const std::string & path)
{
  std::ifstream file(path);
  nlohmann::json action_data = nlohmann::json::parse(file);

  name = action_data["name"];
  for (auto &[key, val] : action_data.items()) {
    if (key.find("step_") != std::string::npos) {
      Pose pose(key);
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
      insert_pose(pose);
    }
  }
}

#include <tachimawari/joint.hpp>

#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>

namespace tachimawari
{

const std::map<std::string, uint8_t> Joint::ids = {
  // head motors
  {"neck_yaw", 19},
  {"neck_pitch", 20},

  // left arm motors
  {"left_shoulder_pitch", 2},
  {"left_shoulder_roll", 4},
  {"left_elbow", 6},

  // right arm motors
  {"right_shoulder_pitch", 1},
  {"right_shoulder_roll", 3},
  {"right_elbow", 5},

  // left leg motors
  {"left_hip_yaw", 8},
  {"left_hip_roll", 10},
  {"left_hip_pitch", 12},
  {"left_knee", 14},
  {"left_ankle_roll", 16},
  {"left_ankle_pitch", 18},

  // right leg motors
  {"right_hip_yaw", 7},
  {"right_hip_roll", 9},
  {"right_hip_pitch", 11},
  {"right_knee", 13},
  {"right_ankle_roll", 15},
  {"right_ankle_pitch", 17},
};

Joint::Joint(const std::string & joint_name, const float & present_position)
: id(Joint::ids.at(joint_name)), name(joint_name), position(present_position)
{
}

void Joint::set_target_position(const float & target_position, const float & speed)
{
  goal_position = target_position;
  additional_position = (goal_position - position) * speed;
}

void Joint::set_present_position(const float & present_position)
{
  position = present_position;
}

void Joint::set_pid_gain(const float & p, const float & i, const float & d)
{
  p_gain = p;
  i_gain = i;
  d_gain = d;
}

void Joint::interpolate()
{
  bool goal_position_is_reached = false;
  goal_position_is_reached |= (additional_position >= 0 &&
    position + additional_position >= goal_position);
  goal_position_is_reached |= (additional_position <= 0 &&
    position + additional_position < goal_position);

  if (goal_position_is_reached) {
    position = goal_position;
    additional_position = 0.0;
  } else {
    position = position + additional_position;
  }
}

const uint8_t & Joint::get_id() const
{
  return id;
}

const std::string & Joint::get_joint_name() const
{
  return name;
}

const float & Joint::get_position() const
{
  return position;
}

const float & Joint::get_goal_position() const
{
  return goal_position;
}

std::vector<float> Joint::get_pid_gain() const  // temporary
{
  return {p_gain, i_gain, d_gain};
}

}  // namespace tachimawari
