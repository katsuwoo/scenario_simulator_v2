// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <openscenario_interpreter/syntax/misc_object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
MiscObject::operator openscenario_msgs::msg::MiscObjectParameters() const
{
  openscenario_msgs::msg::MiscObjectParameters misc_object_parameters;
  {
    misc_object_parameters.misc_object_category = boost::lexical_cast<String>(misc_object_category);
    misc_object_parameters.name = name;
    misc_object_parameters.bounding_box =
      static_cast<const openscenario_msgs::msg::BoundingBox>(bounding_box);
  }

  return misc_object_parameters;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
