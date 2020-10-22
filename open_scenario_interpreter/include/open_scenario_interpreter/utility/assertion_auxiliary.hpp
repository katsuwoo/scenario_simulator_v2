// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__UTILITY__ASSERTION_AUXILIARY_HPP_
#define OPEN_SCENARIO_INTERPRETER__UTILITY__ASSERTION_AUXILIARY_HPP_

#include <utility>

namespace open_scenario_interpreter
{
inline namespace utility
{
#define ASSERT_DEFAULT_CONSTRUCTIBLE(TYPE) \
  static_assert( \
    std::is_default_constructible<TYPE>::value, \
    "OpenSCENARIO specification uses type '" #TYPE "' as optional element (as minOccurs=\"0\"), " \
    "thus type '" #TYPE "' must be met concept DefaultConstructible.")
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__UTILITY__ASSERTION_AUXILIARY_HPP_
