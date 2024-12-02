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

#ifndef OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
#define OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_

#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/routing_algorithm.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/unsigned_integer.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace openscenario_interpreter
{
using NativeWorldPosition = geometry_msgs::msg::Pose;

using NativeRelativeWorldPosition = NativeWorldPosition;

using NativeLanePosition = traffic_simulator::CanonicalizedLaneletPose;

using NativeRelativeLanePosition = traffic_simulator::LaneletPose;

class SimulatorCore
{
  static inline std::unique_ptr<traffic_simulator::API> core = nullptr;

public:
  template <typename Node, typename... Ts>
  static auto activate(
    const Node & node, const traffic_simulator::Configuration & configuration, Ts &&... xs) -> void
  {
    if (not active()) {
      core = std::make_unique<traffic_simulator::API>(
        node, configuration, std::forward<decltype(xs)>(xs)...);
    } else {
      throw Error("The simulator core has already been instantiated.");
    }
  }

  static auto active() { return static_cast<bool>(core); }

  static auto deactivate() -> void
  {
    if (active()) {
      core->despawnEntities();
      core->closeZMQConnection();
      core.reset();
    }
  }

  static auto update() -> void { core->updateFrame(); }

  class CoordinateSystemConversion
  {
  protected:
    static auto canonicalize(const traffic_simulator::LaneletPose & non_canonicalized)
      -> NativeLanePosition
    {
      return NativeLanePosition(non_canonicalized, core->getHdmapUtils());
    }

    template <typename T, typename std::enable_if_t<std::is_same_v<T, NativeLanePosition>, int> = 0>
    static auto convert(const NativeWorldPosition & pose) -> NativeLanePosition
    {
      constexpr bool include_crosswalk{false};
      if (
        const auto result = traffic_simulator::pose::toCanonicalizedLaneletPose(
          pose, include_crosswalk, core->getHdmapUtils())) {
        return result.value();
      } else {
        throw Error(
          "The specified WorldPosition = [", pose.position.x, ", ", pose.position.y, ", ",
          pose.position.z,
          "] could not be approximated to the proper Lane. Perhaps the "
          "WorldPosition points to a location where multiple lanes overlap, and "
          "there are at least two or more candidates for a LanePosition that "
          "can be approximated to that WorldPosition. This issue can be "
          "resolved by strictly specifying the location using LanePosition "
          "instead of WorldPosition");
      }
    }

    template <
      typename T, typename std::enable_if_t<std::is_same_v<T, NativeWorldPosition>, int> = 0>
    static auto convert(const NativeLanePosition & native_lane_position) -> NativeWorldPosition
    {
      return traffic_simulator::pose::toMapPose(native_lane_position);
    }

    static auto makeNativeRelativeWorldPosition(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto to_entity = core->getEntity(to_entity_name)) {
          if (
            const auto relative_pose = traffic_simulator::pose::relativePose(
              from_entity->getMapPose(), to_entity->getMapPose()))
            return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeRelativeWorldPosition(
      const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (
          const auto relative_pose =
            traffic_simulator::pose::relativePose(from_entity->getMapPose(), to_map_pose)) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeRelativeWorldPosition(
      const NativeWorldPosition & from_map_pose, const std::string & to_entity_name)
    {
      if (const auto to_entity = core->getEntity(to_entity_name)) {
        if (
          const auto relative_pose =
            traffic_simulator::pose::relativePose(from_map_pose, to_entity->getMapPose())) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeRelativeLanePosition(
      const std::string & from_entity_name, const std::string & to_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      if (const auto to_entity = core->getEntity(to_entity_name)) {
        if (const auto to_lanelet_pose = to_entity->getCanonicalizedLaneletPose()) {
          return makeNativeRelativeLanePosition(
            from_entity_name, to_lanelet_pose.value(), routing_algorithm);
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeRelativeLanePosition(
      const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto from_lanelet_pose = from_entity->getCanonicalizedLaneletPose()) {
          return makeNativeRelativeLanePosition(
            from_lanelet_pose.value(), to_lanelet_pose, routing_algorithm);
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeRelativeLanePosition(
      const NativeLanePosition & from_lanelet_pose, const NativeLanePosition & to_lanelet_pose,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      return traffic_simulator::pose::relativeLaneletPose(
        from_lanelet_pose, to_lanelet_pose, routing_configuration, core->getHdmapUtils());
    }

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name, const std::string & to_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto to_entity = core->getEntity(to_entity_name)) {
          if (const auto from_lanelet_pose = from_entity->getCanonicalizedLaneletPose()) {
            if (const auto to_lanelet_pose = to_entity->getCanonicalizedLaneletPose()) {
              return makeNativeBoundingBoxRelativeLanePosition(
                from_lanelet_pose.value(), from_entity->getBoundingBox(), to_lanelet_pose.value(),
                to_entity->getBoundingBox(), routing_algorithm);
            }
          }
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto from_lanelet_pose = from_entity->getCanonicalizedLaneletPose()) {
          return makeNativeBoundingBoxRelativeLanePosition(
            from_lanelet_pose.value(), from_entity->getBoundingBox(), to_lanelet_pose,
            traffic_simulator_msgs::msg::BoundingBox(), routing_algorithm);
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const NativeLanePosition & from_lanelet_pose,
      const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
      const NativeLanePosition & to_lanelet_pose,
      const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      return traffic_simulator::pose::boundingBoxRelativeLaneletPose(
        from_lanelet_pose, from_bounding_box, to_lanelet_pose, to_bounding_box,
        routing_configuration, core->getHdmapUtils());
    }

    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto to_entity = core->getEntity(to_entity_name)) {
          if (
            const auto relative_pose = traffic_simulator::pose::boundingBoxRelativePose(
              from_entity->getMapPose(), from_entity->getBoundingBox(), to_entity->getMapPose(),
              to_entity->getBoundingBox())) {
            return relative_pose.value();
          }
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (
          const auto relative_pose = traffic_simulator::pose::boundingBoxRelativePose(
            from_entity->getMapPose(), from_entity->getBoundingBox(), to_map_pose,
            traffic_simulator_msgs::msg::BoundingBox())) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto evaluateLateralRelativeLanes(
      const std::string & from_entity_name, const std::string & to_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> int
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto to_entity = core->getEntity(to_entity_name)) {
          if (
            auto lane_changes = traffic_simulator::distance::countLaneChanges(
              from_entity->getCanonicalizedLaneletPose().value(),
              to_entity->getCanonicalizedLaneletPose().value(), routing_configuration,
              core->getHdmapUtils())) {
            return lane_changes.value().first - lane_changes.value().second;
          }
        }
      }
      throw common::Error(
        "Failed to evaluate lateral relative lanes between ", from_entity_name, " and ",
        to_entity_name);
    }
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename... Ts>
    static auto applyAcquirePositionAction(const std::string & entity_ref, Ts &&... xs)
    {
      core->requestClearRoute(entity_ref);
      return core->requestAcquirePosition(entity_ref, std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAddEntityAction(Ts &&... xs)
    {
      return core->spawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename EntityRef, typename DynamicConstraints>
    static auto applyProfileAction(
      const EntityRef & entity_ref, const DynamicConstraints & dynamic_constraints) -> void
    {
      return core->setBehaviorParameter(entity_ref, [&]() {
        auto behavior_parameter = core->getBehaviorParameter(entity_ref);

        if (not std::isinf(dynamic_constraints.max_speed)) {
          behavior_parameter.dynamic_constraints.max_speed = dynamic_constraints.max_speed;
        }

        if (not std::isinf(dynamic_constraints.max_acceleration)) {
          behavior_parameter.dynamic_constraints.max_acceleration =
            dynamic_constraints.max_acceleration;
        }

        if (not std::isinf(dynamic_constraints.max_acceleration_rate)) {
          behavior_parameter.dynamic_constraints.max_acceleration_rate =
            dynamic_constraints.max_acceleration_rate;
        }

        if (not std::isinf(dynamic_constraints.max_deceleration)) {
          behavior_parameter.dynamic_constraints.max_deceleration =
            dynamic_constraints.max_deceleration;
        }

        if (not std::isinf(dynamic_constraints.max_deceleration_rate)) {
          behavior_parameter.dynamic_constraints.max_deceleration_rate =
            dynamic_constraints.max_deceleration_rate;
        }

        return behavior_parameter;
      }());
    }

    template <typename Controller>
    static auto applyAssignControllerAction(
      const std::string & entity_ref, Controller && controller) -> void
    {
      core->setVelocityLimit(
        entity_ref, controller.properties.template get<Double>(
                      "maxSpeed", std::numeric_limits<Double::value_type>::max()));

      core->setBehaviorParameter(entity_ref, [&]() {
        auto message = core->getBehaviorParameter(entity_ref);
        message.see_around = not controller.properties.template get<Boolean>("isBlind");
        /// The default values written in https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/traffic_simulator_msgs/msg/DynamicConstraints.msg
        message.dynamic_constraints.max_acceleration =
          controller.properties.template get<Double>("maxAcceleration", 10.0);
        message.dynamic_constraints.max_acceleration_rate =
          controller.properties.template get<Double>("maxAccelerationRate", 3.0);
        message.dynamic_constraints.max_deceleration =
          controller.properties.template get<Double>("maxDeceleration", 10.0);
        message.dynamic_constraints.max_deceleration_rate =
          controller.properties.template get<Double>("maxDecelerationRate", 3.0);
        message.dynamic_constraints.max_speed =
          controller.properties.template get<Double>("maxSpeed", 50.0);
        return message;
      }());

      if (controller.isAutoware()) {
        core->attachImuSensor(entity_ref, [&]() {
          simulation_api_schema::ImuSensorConfiguration configuration;
          configuration.set_entity(entity_ref);
          configuration.set_frame_id("base_link");
          configuration.set_add_gravity(true);
          configuration.set_use_seed(true);
          configuration.set_seed(0);
          configuration.set_noise_standard_deviation_orientation(0.01);
          configuration.set_noise_standard_deviation_twist(0.01);
          configuration.set_noise_standard_deviation_acceleration(0.01);
          return configuration;
        }());

        core->attachLidarSensor([&]() {
          simulation_api_schema::LidarConfiguration configuration;

          auto degree_to_radian = [](auto degree) {
            return degree / 180.0 * boost::math::constants::pi<double>();
          };

          // clang-format off
          configuration.set_architecture_type(core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
          configuration.set_entity(entity_ref);
          configuration.set_horizontal_resolution(degree_to_radian(controller.properties.template get<Double>("pointcloudHorizontalResolution", 1.0)));
          configuration.set_lidar_sensor_delay(controller.properties.template get<Double>("pointcloudPublishingDelay"));
          configuration.set_scan_duration(0.1);
          // clang-format on

          const auto vertical_field_of_view = degree_to_radian(
            controller.properties.template get<Double>("pointcloudVerticalFieldOfView", 30.0));

          const auto channels =
            controller.properties.template get<UnsignedInteger>("pointcloudChannels", 16);

          for (std::size_t i = 0; i < channels; ++i) {
            configuration.add_vertical_angles(
              vertical_field_of_view / 2 - vertical_field_of_view / channels * i);
          }

          return configuration;
        }());

        core->attachDetectionSensor([&]() {
          simulation_api_schema::DetectionSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
          configuration.set_entity(entity_ref);
          configuration.set_detect_all_objects_in_range(controller.properties.template get<Boolean>("isClairvoyant"));
          configuration.set_object_recognition_delay(controller.properties.template get<Double>("detectedObjectPublishingDelay"));
          configuration.set_pos_noise_stddev(controller.properties.template get<Double>("detectedObjectPositionStandardDeviation"));
          configuration.set_probability_of_lost(controller.properties.template get<Double>("detectedObjectMissingProbability"));
          configuration.set_random_seed(controller.properties.template get<UnsignedInteger>("randomSeed"));
          configuration.set_range(controller.properties.template get<Double>("detectionSensorRange",300.0));
          configuration.set_object_recognition_ground_truth_delay(controller.properties.template get<Double>("detectedObjectGroundTruthPublishingDelay"));
          configuration.set_update_duration(0.1);
          // clang-format on
          return configuration;
        }());

        core->attachOccupancyGridSensor([&]() {
          simulation_api_schema::OccupancyGridSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
          configuration.set_entity(entity_ref);
          configuration.set_filter_by_range(controller.properties.template get<Boolean>("isClairvoyant"));
          configuration.set_height(200);
          configuration.set_range(300);
          configuration.set_resolution(0.5);
          configuration.set_update_duration(0.1);
          configuration.set_width(200);
          // clang-format on
          return configuration;
        }());

        core->attachPseudoTrafficLightDetector([&]() {
          simulation_api_schema::PseudoTrafficLightDetectorConfiguration configuration;
          configuration.set_architecture_type(
            core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
          return configuration;
        }());

        core->asFieldOperatorApplication(entity_ref)
          .declare_parameter<bool>(
            "allow_goal_modification",
            controller.properties.template get<Boolean>("allowGoalModification"));

        for (const auto & module :
             [](std::string manual_modules_string) {
               manual_modules_string.erase(
                 std::remove_if(
                   manual_modules_string.begin(), manual_modules_string.end(),
                   [](const auto & c) { return std::isspace(c); }),
                 manual_modules_string.end());

               std::vector<std::string> modules;
               std::string buffer;
               std::istringstream modules_stream(manual_modules_string);
               while (std::getline(modules_stream, buffer, ',')) {
                 modules.push_back(buffer);
               }
               return modules;
             }(controller.properties.template get<String>(
               "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions"))) {
          try {
            core->asFieldOperatorApplication(entity_ref)
              .requestAutoModeForCooperation(module, false);
          } catch (const Error & error) {
            throw Error(
              "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions is not "
              "supported in this environment: ",
              error.what());
          }
        }
      }
    }

    template <typename... Ts>
    static auto applyAssignRouteAction(const std::string & entity_ref, Ts &&... xs)
    {
      core->requestClearRoute(entity_ref);
      return core->requestAssignRoute(entity_ref, std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyDeleteEntityAction(Ts &&... xs)
    {
      return core->despawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyFollowTrajectoryAction(Ts &&... xs)
    {
      return core->requestFollowTrajectory(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyLaneChangeAction(Ts &&... xs)
    {
      return core->requestLaneChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applySpeedAction(Ts &&... xs)
    {
      return core->requestSpeedChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(Ts &&... xs)
    {
      return core->setEntityStatus(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyWalkStraightAction(Ts &&... xs)
    {
      return core->requestWalkStraight(std::forward<decltype(xs)>(xs)...);
    }
  };

  // OpenSCENARIO 1.1.1 Section 3.1.5
  class ConditionEvaluation : protected CoordinateSystemConversion
  {
  protected:
    template <typename... Ts>
    static auto evaluateAcceleration(Ts &&... xs)
    {
      return core->getCurrentAccel(std::forward<decltype(xs)>(xs)...).linear.x;
    }

    template <typename... Ts>
    static auto evaluateCollisionCondition(Ts &&... xs) -> bool
    {
      return core->checkCollision(std::forward<decltype(xs)>(xs)...);
    }

    static auto evaluateBoundingBoxEuclideanDistance(
      const std::string & from_entity_name,
      const std::string & to_entity_name)  // for RelativeDistanceCondition
    {
      if (const auto from_entity = core->getEntity(from_entity_name)) {
        if (const auto to_entity = core->getEntity(to_entity_name)) {
          if (
            const auto distance = traffic_simulator::distance::boundingBoxDistance(
              from_entity->getMapPose(), from_entity->getBoundingBox(), to_entity->getMapPose(),
              to_entity->getBoundingBox())) {
            return distance.value();
          }
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    static auto evaluateRelativeSpeed(const Entity & from, const Entity & to) -> Eigen::Vector3d
    {
      if (const auto observer = core->getEntity(from.name())) {
        if (const auto observed = core->getEntity(to.name())) {
          auto velocity = [](const auto & entity) -> Eigen::Vector3d {
            auto direction = [](auto orientation) -> Eigen::Vector3d {
              const auto euler_angle = math::geometry::convertQuaternionToEulerAngle(orientation);
              const auto r = euler_angle.x;
              const auto p = euler_angle.y;
              const auto y = euler_angle.z;
              return Eigen::Vector3d(
                std::cos(y) * std::cos(p), std::sin(y) * std::cos(p), std::sin(p));
            };

            return direction(entity->getMapPose().orientation) * entity->getCurrentTwist().linear.x;
          };

          const Eigen::Matrix3d rotation =
            math::geometry::getRotationMatrix(observer->getMapPose().orientation);

          return rotation.transpose() * velocity(observed) -
                 rotation.transpose() * velocity(observer);
        }
      }

      return Eigen::Vector3d(Double::nan(), Double::nan(), Double::nan());
    }

    template <typename... Ts>
    static auto evaluateSimulationTime(Ts &&... xs) -> double
    {
      if (SimulatorCore::active()) {
        return core->getCurrentTime(std::forward<decltype(xs)>(xs)...);
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    template <typename... Ts>
    static auto evaluateSpeed(Ts &&... xs)
    {
      return core->getCurrentTwist(std::forward<decltype(xs)>(xs)...).linear;
    }

    template <typename... Ts>
    static auto evaluateStandStill(Ts &&... xs)
    {
      return core->getStandStillDuration(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto evaluateTimeHeadway(Ts &&... xs)
    {
      if (const auto result = core->getTimeHeadway(std::forward<decltype(xs)>(xs)...); result) {
        return result.value();
      } else {
        using value_type = typename std::decay<decltype(result)>::type::value_type;
        return std::numeric_limits<value_type>::quiet_NaN();
      }
    }

    static auto evaluateTimeToCollisionCondition(const Entity & from, const Entity & to)
    {
      /*
         This function uses the Separating Axis Theorem (SAT) to find the
         Time-To-Collision (TTC) of two entities in the world coordinate
         system.

         First, the expected time of collision is obtained independently for
         each separating axis. Then, we check whether the polygons of the
         entities actually collide after that time. If no collision occurs, the
         expected collision time is rejected. The lowest value of the expected
         collision times obtained for each separate axis is then returned as
         the TTC.
      */

      if (const auto entity_a = core->getEntity(from.name())) {
        if (const auto entity_b = core->getEntity(to.name())) {
          struct OrientedBoundingBox
          {
            Eigen::Vector3d position;

            Eigen::Matrix3d rotation;

            Eigen::Vector3d size;  // [width, height, depth]

            explicit OrientedBoundingBox(
              const geometry_msgs::msg::Pose & pose,
              const traffic_simulator_msgs::msg::BoundingBox & box)
            : position(
                pose.position.x + box.center.x, pose.position.y + box.center.y,
                pose.position.z + box.center.z),
              rotation(math::geometry::getRotationMatrix(pose.orientation)),
              size(box.dimensions.y / 2, box.dimensions.z / 2, box.dimensions.x / 2)
            {
            }

            auto axis(std::size_t index) const -> Eigen::Vector3d { return rotation.col(index); }
          };

          const Eigen::Vector3d v = evaluateRelativeSpeed(from, to);

          const auto a = OrientedBoundingBox(entity_a->getMapPose(), entity_a->getBoundingBox());
          const auto b = OrientedBoundingBox(entity_b->getMapPose(), entity_b->getBoundingBox());

          const auto axes = std::array<Eigen::Vector3d, 15>{
            a.axis(0),
            a.axis(1),
            a.axis(2),

            b.axis(0),
            b.axis(1),
            b.axis(2),

            a.axis(0).cross(b.axis(0)),
            a.axis(0).cross(b.axis(1)),
            a.axis(0).cross(b.axis(2)),

            a.axis(1).cross(b.axis(0)),
            a.axis(1).cross(b.axis(1)),
            a.axis(1).cross(b.axis(2)),

            a.axis(2).cross(b.axis(0)),
            a.axis(2).cross(b.axis(1)),
            a.axis(2).cross(b.axis(2)),
          };

          Eigen::Vector3d d = b.position - a.position;

          auto t_min = std::numeric_limits<double>::infinity();

          auto projection = [&](const auto & box, const auto & axis) {
            return std::abs(axis.dot(box.axis(0)) * box.size(0)) +
                   std::abs(axis.dot(box.axis(1)) * box.size(1)) +
                   std::abs(axis.dot(box.axis(2)) * box.size(2));
          };

          for (const auto & axis : axes) {
            /*
               The variable `separation` means how far apart two Oriented
               Bounding Boxes (OBBs) are on the separation axis. When it is
               positive, it means they are separated by `separation`, i.e. the
               OBBs do not overlap on the separation axis. When it is less than
               zero, it means the OBBs overlap on the separation axis.

               d.dot(axis) is the projection of the center distance between the
               two OBBs onto the separation axis, and projection(a, axis) and
               projection(b, axis) are half the projected range of the boxes
               when the two OBBs are projected onto the separation axis.
            */
            if (const auto separation =
                  std::abs(d.dot(axis)) - projection(a, axis) - projection(b, axis);
                0 < separation) {
              /*
                 v.dot(axis) is the projection of the relative distance onto
                 the separation axis. In other words, we want to find the TTC
                 for each separation axis. The relative velocity is the
                 velocity vector of the observed object measured in the
                 observer's rest frame, so a negative value means that the two
                 OBBs are approaching each other.
              */
              if (const auto relative_speed = v.dot(axis); relative_speed < 0) {
                /*
                   Since this t is only a Time-To-Collision on this separation
                   axis, it is necessary to check that a and b are not
                   separated on all separation axes after t seconds.
                */
                const auto t = separation / -relative_speed;

                const auto a_t = a;

                const auto b_t = [&]() {
                  /*
                     Since t is the exact time when contact occurs, it may not
                     be possible to say that a and b are in collision after t
                     seconds. Therefore, 0.01 seconds is added to t so that
                     collision can be expected to occur with certainty. This
                     value "0.01" has no technical basis and is an arbitrary
                     value.
                  */
                  constexpr auto tolerance = 0.01;
                  auto copy = b;
                  copy.position += v * (t + tolerance);
                  return copy;
                }();

                Eigen::Vector3d d_t = b_t.position - a_t.position;

                if (std::all_of(axes.begin(), axes.end(), [&](const auto & axis) {
                      const auto separation_t =
                        std::abs(d_t.dot(axis)) - projection(a_t, axis) - projection(b_t, axis);
                      return separation_t <= 0;
                    })) {
                  t_min = std::min(t_min, t);
                }
              }
            }
          }

          return t_min;
        }
      }

      return std::numeric_limits<double>::quiet_NaN();
    }
  };

  class NonStandardOperation : private CoordinateSystemConversion
  {
  protected:
    template <typename Performance, typename Properties>
    static auto activatePerformanceAssertion(
      const std::string & entity_ref, const Performance & performance,
      const Properties & properties)
    {
      core->activateOutOfRangeJob(
        entity_ref, -performance.max_speed, +performance.max_speed, -performance.max_deceleration,
        +performance.max_acceleration, properties.template get<Double>("minJerk", Double::lowest()),
        properties.template get<Double>("maxJerk", Double::max()));
    }

    template <typename... Ts>
    static auto asFieldOperatorApplication(Ts &&... xs) -> decltype(auto)
    {
      return core->asFieldOperatorApplication(std::forward<decltype(xs)>(xs)...);
    }

    static auto activateNonUserDefinedControllers() -> decltype(auto)
    {
      return core->startNpcLogic();
    }

    template <typename... Ts>
    static auto evaluateCurrentState(Ts &&... xs) -> decltype(auto)
    {
      return core->getCurrentAction(std::forward<decltype(xs)>(xs)...);
    }

    template <typename EntityRef, typename OSCLanePosition>
    static auto evaluateRelativeHeading(
      const EntityRef & entity_ref, const OSCLanePosition & osc_lane_position)
    {
      if (const auto entity = core->getEntity(entity_ref)) {
        const auto from_map_pose = entity->getMapPose();
        const auto to_map_pose = static_cast<NativeWorldPosition>(osc_lane_position);
        if (
          const auto relative_pose =
            traffic_simulator::pose::relativePose(from_map_pose, to_map_pose)) {
          return static_cast<Double>(std::abs(
            math::geometry::convertQuaternionToEulerAngle(relative_pose.value().orientation).z));
        }
      }
      return Double::nan();
    }

    template <typename EntityRef>
    static auto evaluateRelativeHeading(const EntityRef & entity_ref)
    {
      if (const auto entity = core->getEntity(entity_ref)) {
        if (const auto canonicalized_lanelet_pose = entity->getCanonicalizedLaneletPose()) {
          return static_cast<Double>(std::abs(
            static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose.value()).rpy.z));
        }
      }
      return Double::nan();
    }

    template <typename... Ts>
    static auto sendCooperateCommand(Ts &&... xs) -> decltype(auto)
    {
      return asFieldOperatorApplication(core->getEgoName())
        .sendCooperateCommand(std::forward<decltype(xs)>(xs)...);
    }

    // TrafficLights - Conventional and V2I
    template <typename... Ts>
    static auto setConventionalTrafficLightsState(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->setTrafficLightsState(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto setConventionalTrafficLightConfidence(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->setTrafficLightsConfidence(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto getConventionalTrafficLightsComposedState(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->getTrafficLightsComposedState(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto compareConventionalTrafficLightsState(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->compareTrafficLightsState(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto resetConventionalTrafficLightPublishRate(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->resetUpdate(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto setV2ITrafficLightsState(Ts &&... xs) -> decltype(auto)
    {
      return core->getV2ITrafficLights()->setTrafficLightsState(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto resetV2ITrafficLightPublishRate(Ts &&... xs) -> decltype(auto)
    {
      return core->getV2ITrafficLights()->resetUpdate(std::forward<decltype(xs)>(xs)...);
    }
  };
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
