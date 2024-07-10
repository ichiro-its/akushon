// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <map>
#include <string>

#include "akushon/action/model/action_name.hpp"
#include "akushon/action/model/action.hpp"

namespace akushon
{

const std::map<std::string, int> ActionName::map = {
  {"init", 0},
  {"walk_ready", 1},
  {"sit_down", 2},
  {"forward_up", 3},
  {"backward_up", 4},
  {"leftward_up", 5}
};

const char * ActionName::INIT = "init";
const char * ActionName::WALKREADY = "walk_ready";
const char * ActionName::SIT_DOWN = "sit_down";
const char * ActionName::FORWARD_UP = "forward_up";
const char * ActionName::BACKWARD_UP = "backward_up";
const char * ActionName::LEFTWARD_UP = "leftward_up";
const char * ActionName::RIGHTWARD_UP = "rightward_up";
const char * ActionName::RIGHT_KICK = "right_kick";
const char * ActionName::LEFT_KICK = "left_kick";
const char * ActionName::RIGHT_KICK_SHORT = "right_kick_short";
const char * ActionName::LEFT_KICK_SHORT = "left_kick_short";
const char * ActionName::RIGHT_KICK_WIDE = "right_kick_wide";
const char * ActionName::LEFT_KICK_WIDE = "left_kick_wide";
const char * ActionName::LEFT_SIDEKICK = "left_sidekick";
const char * ActionName::RIGHT_SIDEKICK = "right_sidekick";
const char * ActionName::KEEPER_SIT = "keeper_sit";
const char * ActionName::KEEPER_UP = "keeper_up";
const char * ActionName::LEFT_SIDEKICK_90 = "left_sidekick90";
const char * ActionName::RIGHT_SIDEKICK_90 = "right_sidekick90";
const char * ActionName::DK_READY = "dk_ready";
const char * ActionName::DK_KICK = "dk_kick";
const char * ActionName::HIGH_KICK = "high_kick";
const char * ActionName::PARKOUR_UP = "parkour_up";
const char * ActionName::PARKOUR_DOWN = "parkour_down";

}  // namespace akushon
