// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/syntax/license.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
License::License(const pugi::xml_node & tree, Scope & scope)
: name(readAttribute<String>("name", tree, scope)),
  resource(readAttribute<String>("resource", tree, scope, String())),  // NOTE: Optional attribute
  spdxId(readAttribute<String>("spdxId", tree, scope, String())),  // NOTE: Optional attribute
  text(readContent<String>(tree, scope))  // NOTE: Optional content
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
