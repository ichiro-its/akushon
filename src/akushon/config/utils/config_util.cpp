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
#include <string>

#include "akushon/action/model/action_name.hpp"
#include "akushon/config/utils/config_util.hpp"
#include "nlohmann/json.hpp"

namespace akushon
{

ConfigUtil::ConfigUtil(const std::string & path)
: path(path)
{
}

std::string ConfigUtil::get_config() const
{
  nlohmann::json actions_list;
  for (const auto & [name, id] : ActionName::map) {
    std::string file_name = path + "/action/" + name + ".json";

    try {
      std::ifstream file(file_name);
      nlohmann::json action_data = nlohmann::json::parse(file);

      actions_list["action_" + name] = action_data;
    } catch (nlohmann::json::parse_error & ex) {
      // TODO(maroqijalil): will be used for logging
      // std::cerr << "parse error at byte " << ex.byte << std::endl;
    }
  }
  return actions_list.dump();
}

void ConfigUtil::set_config(const std::string & actions_data)
{
  nlohmann::json actions_list = nlohmann::json::parse(actions_data);
  for (const auto & [key, val] : actions_list.items()) {
    std::locale loc;
    std::string action_name = key;
    std::replace(action_name.begin(), action_name.end(), ' ', '_');
    std::string file_name = path + "/action/" + action_name + ".json";
    std::ofstream file;

    file.open(file_name);
    file << val.dump(2);
    file.close();
  }
}

}  // namespace akushon
