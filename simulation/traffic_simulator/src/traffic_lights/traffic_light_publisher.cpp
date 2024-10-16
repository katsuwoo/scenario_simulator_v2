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

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>

namespace traffic_simulator
{
template <>
auto TrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::publish(
  const TrafficLightsBase & traffic_lights) const -> void
{
  traffic_light_state_array_publisher_->publish(traffic_lights.generateAutowareAutoPerceptionMsg());
}

template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const TrafficLightsBase & traffic_lights) const -> void
{
  traffic_light_state_array_publisher_->publish(traffic_lights.generateAutowarePerceptionMsg());
}

template <>
auto TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>::publish(
  const TrafficLightsBase & traffic_lights) const -> void
{
  traffic_light_state_array_publisher_->publish(traffic_lights.generateTrafficSimulatorV1Msg());
}
}  // namespace traffic_simulator
