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

#ifndef SCENARIO_RUNNER__SYNTAX__DOUBLE_HPP_
#define SCENARIO_RUNNER__SYNTAX__DOUBLE_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/float64.hpp>

#include <limits>
#include <regex>
#include <string>

namespace scenario_runner
{
inline namespace syntax
{
struct Double
  : public std_msgs::msg::Float64
{
  using value_type = decltype(std_msgs::msg::Float64::data);

  explicit Double(value_type value = {})
  {
    data = value;
  }

  explicit Double(const std::string & s) try
  {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    std::stringstream ss {};
    ss << "can't treat value \"" << s << "\" as type Double";
    throw SyntaxError {ss.str()};
  }

  constexpr operator value_type() const noexcept
  {
    return data;
  }

  static auto infinity() noexcept
  {
    return static_cast<Double>(std::numeric_limits<value_type>::infinity());
  }
};

template<typename ... Ts>
decltype(auto) operator<<(std::basic_ostream<Ts...>&os, const Double & rhs)
{
  return os << std::fixed << rhs.data;
}

std::istream & operator>>(std::istream & is, Double & rhs)
{
  std::string token {};

  is >> token;

  static const std::regex infinity {R"([+-]?inf(inity)?)"};

  std::smatch result {};

  if (std::regex_match(token, result, infinity)) {
      #ifndef SCENARIO_RUNNER_ALLOW_INFINITY
      #define SCENARIO_RUNNER_DOUBLE_INFINITY max
      #else
      #define SCENARIO_RUNNER_DOUBLE_INFINITY infinity
      #endif

    rhs.data =
      (result.str(1) ==
      "-" ? -1 : 1) *
      std::numeric_limits<Double::value_type>::SCENARIO_RUNNER_DOUBLE_INFINITY();

      #undef SCENARIO_RUNNER_DOUBLE_INFINITY
  } else {
    rhs.data = boost::lexical_cast<Double::value_type>(token);
  }

  return is;
}
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__DOUBLE_HPP_
