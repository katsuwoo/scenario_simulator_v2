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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_SHORT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_SHORT_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct UnsignedShort
  : public std_msgs::msg::UInt16
{
  using value_type = decltype(std_msgs::msg::UInt16::data);

  explicit UnsignedShort(value_type value = {})
  {
    data = value;
  }

  explicit UnsignedShort(const std::string & s) try
  {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    std::stringstream ss {};
    ss << "can't treat value \"" << s << "\" as type UnsignedShort";
    throw SyntaxError {ss.str()};
  }

  constexpr operator value_type() const noexcept
  {
    return data;
  }

  decltype(auto) operator++() noexcept
  {
    ++data;
    return *this;
  }

  auto & operator+=(const double & rhs)
  {
    data += rhs;
    return *this;
  }

  auto & operator*=(const double & rhs)
  {
    data *= rhs;
    return *this;
  }
};

template<typename ... Ts>
decltype(auto) operator<<(std::basic_ostream<Ts...>&os, const UnsignedShort & rhs)
{
  return os << rhs.data;
}

template<typename ... Ts>
decltype(auto) operator>>(std::basic_istream<Ts...>&is, UnsignedShort & rhs)
{
  return is >> rhs.data;
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_SHORT_HPP_