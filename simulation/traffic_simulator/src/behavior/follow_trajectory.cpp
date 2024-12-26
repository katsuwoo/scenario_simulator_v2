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

#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/truncate.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
template <typename F, typename T, typename... Ts>
auto any(F f, T && x, Ts &&... xs)
{
  if constexpr (math::geometry::IsLikeVector3<std::decay_t<decltype(x)>>::value) {
    return any(f, x.x, x.y, x.z);
  } else if constexpr (0 < sizeof...(xs)) {
    return f(x) or any(f, std::forward<decltype(xs)>(xs)...);
  } else {
    return f(x);
  }
}

auto makeUpdatedStatus(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const double step_time,
  double matching_distance, std::optional<double> target_speed) -> std::optional<EntityStatus>
{
  using math::arithmetic::isApproximatelyEqualTo;
  using math::arithmetic::isDefinitelyLessThan;

  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;
  using math::geometry::operator+=;

  using math::geometry::CatmullRomSpline;
  using math::geometry::hypot;
  using math::geometry::innerProduct;
  using math::geometry::norm;
  using math::geometry::normalize;
  using math::geometry::truncate;

  auto distance_along_lanelet =
    [&](const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to) -> double {
    if (const auto from_lanelet_pose =
          hdmap_utils->toLaneletPose(from, entity_status.bounding_box, false, matching_distance);
        from_lanelet_pose) {
      if (const auto to_lanelet_pose =
            hdmap_utils->toLaneletPose(to, entity_status.bounding_box, false, matching_distance);
          to_lanelet_pose) {
        if (const auto distance = hdmap_utils->getLongitudinalDistance(
              from_lanelet_pose.value(), to_lanelet_pose.value());
            distance) {
          return distance.value();
        }
      }
    }
    return hypot(from, to);
  };

  auto discard_the_front_waypoint_and_recurse = [&]() {
    if (
      not std::isnan(polyline_trajectory.base_time) and
      not std::isnan(polyline_trajectory.shape.vertices.front().time)) {
      polyline_trajectory.base_time = entity_status.time;
    }

    if (std::rotate(
          std::begin(polyline_trajectory.shape.vertices),
          std::begin(polyline_trajectory.shape.vertices) + 1,
          std::end(polyline_trajectory.shape.vertices));
        not polyline_trajectory.closed) {
      polyline_trajectory.shape.vertices.pop_back();
    }

    return makeUpdatedStatus(
      entity_status, polyline_trajectory, behavior_parameter, hdmap_utils, step_time,
      matching_distance, target_speed);
  };

  auto is_infinity_or_nan = [](auto x) constexpr { return std::isinf(x) or std::isnan(x); };

  auto first_waypoint_with_arrival_time_specified = [&]() {
    return std::find_if(
      polyline_trajectory.shape.vertices.begin(), polyline_trajectory.shape.vertices.end(),
      [](auto && vertex) { return not std::isnan(vertex.time); });
  };


  if (polyline_trajectory.shape.vertices.empty()) {
    return std::nullopt;
  } else if (const auto position = entity_status.pose.position; any(is_infinity_or_nan, position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), " coordinate value contains NaN or infinity. The value is [",
      position.x, ", ", position.y, ", ", position.z, "].");
  } else if (
    const auto target_position = polyline_trajectory.shape.vertices.front().position.position;
    any(is_infinity_or_nan, target_position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
  } else if (
    const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] = std::make_tuple(
      distance_along_lanelet(position, target_position),
      (not std::isnan(polyline_trajectory.base_time) ? polyline_trajectory.base_time : 0.0) +
        polyline_trajectory.shape.vertices.front().time - entity_status.time);
    isDefinitelyLessThan(distance_to_front_waypoint, std::numeric_limits<double>::epsilon())) {
    return discard_the_front_waypoint_and_recurse();
  } else if (const auto speed = entity_status.action_status.twist.linear.x;  // [m/s]
             std::isinf(speed) or std::isnan(speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s speed value is NaN or infinity. The value is ", speed,
      ". ");
  } else {
     const auto desired_velocity =
      [&]() -> geometry_msgs::msg::Vector3 {
        if (polyline_trajectory.dynamic_constraints_ignorable) {
          const auto dx = target_position.x - position.x;
          const auto dy = target_position.y - position.y;
          const auto pitch =
            entity_status.lanelet_pose_valid
              ? -math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).y
              : std::atan2(target_position.z - position.z, std::hypot(dy, dx));
          const auto yaw = std::atan2(dy, dx);

          double desired_speed;

          if (polyline_trajectory.shape.vertices.size() == 1) {
             desired_speed = target_speed.value_or(behavior_parameter.dynamic_constraints.max_speed);
          } else {
            if (first_waypoint_with_arrival_time_specified() == std::end(polyline_trajectory.shape.vertices)) {
               desired_speed = target_speed.value_or(behavior_parameter.dynamic_constraints.max_speed);
            } else {
                auto next_waypoint_iter = std::next(std::begin(polyline_trajectory.shape.vertices));
                auto current_waypoint_time = polyline_trajectory.shape.vertices.front().time;
                if (next_waypoint_iter != std::end(polyline_trajectory.shape.vertices)){
                    auto next_waypoint = *next_waypoint_iter;
                   auto next_waypoint_position = next_waypoint.position.position;
                   auto next_waypoint_time = next_waypoint.time;
                   auto distance = distance_along_lanelet(target_position, next_waypoint_position);
                    desired_speed = distance / std::abs(next_waypoint_time - current_waypoint_time);
                } else {
                    desired_speed = target_speed.value_or(behavior_parameter.dynamic_constraints.max_speed);
                }
            }

          }
          return geometry_msgs::build<geometry_msgs::msg::Vector3>()
            .x(std::cos(pitch) * std::cos(yaw) * desired_speed)
            .y(std::cos(pitch) * std::sin(yaw) * desired_speed)
            .z(std::sin(pitch) * desired_speed);
        } else {
          throw common::SimulationError(
            "The followingMode is only supported for position.");
        }
      }();


    if (any(is_infinity_or_nan, desired_velocity)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
        desired_velocity.y, ", ", desired_velocity.z, "].");
    }

     const auto current_velocity =
      [&]() {
        const auto pitch =
          -math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).y;
        const auto yaw =
          math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).z;
        return geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(std::cos(pitch) * std::cos(yaw) * speed)
          .y(std::cos(pitch) * std::sin(yaw) * speed)
          .z(std::sin(pitch) * speed);
      }();

      if ((speed * step_time) > distance_to_front_waypoint &&
          innerProduct(desired_velocity, current_velocity) < 0.0) {
        return discard_the_front_waypoint_and_recurse();
      }

    auto updated_status = entity_status;

    updated_status.pose.position += desired_velocity * step_time;

    updated_status.pose.orientation = [&]() {
      if (desired_velocity.y == 0 && desired_velocity.x == 0 && desired_velocity.z == 0) {
        return entity_status.pose.orientation;
      } else {
        const geometry_msgs::msg::Vector3 direction =
          geometry_msgs::build<geometry_msgs::msg::Vector3>()
            .x(0.0)
            .y(std::atan2(-desired_velocity.z, std::hypot(desired_velocity.x, desired_velocity.y)))
            .z(std::atan2(desired_velocity.y, desired_velocity.x));
        return math::geometry::convertEulerAngleToQuaternion(direction);
      }
    }();

    updated_status.action_status.twist.linear.x = norm(desired_velocity);

    updated_status.action_status.twist.linear.y = 0;

    updated_status.action_status.twist.linear.z = 0;

    updated_status.action_status.twist.angular =
      math::geometry::convertQuaternionToEulerAngle(math::geometry::getRotation(
        entity_status.pose.orientation, updated_status.pose.orientation)) /
      step_time;

    // Acceleration is not considered
    updated_status.action_status.accel.linear.x = 0;
    updated_status.action_status.accel.linear.y = 0;
    updated_status.action_status.accel.linear.z = 0;

    updated_status.action_status.accel.angular =
      (updated_status.action_status.twist.angular - entity_status.action_status.twist.angular) /
      step_time;

    updated_status.time = entity_status.time + step_time;

    updated_status.lanelet_pose_valid = false;

    return updated_status;
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
