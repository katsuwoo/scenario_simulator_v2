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

#include <cmath>
#include <geometry/plane.hpp>

namespace math
{
namespace geometry
{

Plane::Plane(const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Vector3 & normal)
: normal_(normal), d_(-(normal.x * point.x + normal.y * point.y + normal.z * point.z))
{
}

auto Plane::calculateOffset(const geometry_msgs::msg::Point & point) const -> double
{
  return normal_.x * point.x + normal_.y * point.y + normal_.z * point.z + d_;
}

auto makePlane(const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Vector3 & normal)
  -> std::optional<Plane>
{
  if (normal.x == 0.0 && normal.y == 0.0 && normal.z == 0.0) {
    return std::nullopt;
  }

  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
    return std::nullopt;
  }

  return Plane(point, normal);
}

}  // namespace geometry
}  // namespace math
