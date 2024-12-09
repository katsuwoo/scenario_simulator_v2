/**
 * @file traffic_controller.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class implementation for the traffic controller
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 2015 TIER IV.inc. All rights reserved.
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

#include <memory>
#include <string>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/traffic/traffic_controller.hpp>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
TrafficController::TrafficController(
  const std::shared_ptr<entity::EntityManager> entity_manager_ptr,
  const std::set<std::uint8_t> & sinkable_entity_type, bool auto_sink)
: entity_manager_ptr(entity_manager_ptr),
  sinkable_entity_type(sinkable_entity_type),
  auto_sink(auto_sink)
{
  if (auto_sink) {
    autoSink();
  }
}

void TrafficController::autoSink()
{
  const auto hdmap_utils_ptr = entity_manager_ptr->getHdmapUtils();
  for (const auto & lanelet_id : hdmap_utils_ptr->getLaneletIds()) {
    if (hdmap_utils_ptr->getNextLaneletIds(lanelet_id).empty()) {
      LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = lanelet_id;
      lanelet_pose.s = pose::laneletLength(lanelet_id, hdmap_utils_ptr);
      const auto pose = pose::toMapPose(lanelet_pose, hdmap_utils_ptr);
      addModule<TrafficSink>(entity_manager_ptr, 1, pose.position, sinkable_entity_type);
    }
  }
}

void TrafficController::execute(const double current_time, const double step_time)
{
  for (const auto & module : modules_) {
    module->execute(current_time, step_time);
  }
}

auto TrafficController::makeDebugMarker() const -> const visualization_msgs::msg::MarkerArray
{
  static const auto marker_array = [&]() {
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < modules_.size(); ++i) {
      modules_[i]->appendDebugMarker(marker_array);
    }
    return marker_array;
  }();
  return marker_array;
}
}  // namespace traffic
}  // namespace traffic_simulator
