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

#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <traffic_simulator/hdmap_utils/traffic_rules.hpp>

lanelet::traffic_rules::RegisterTrafficRules<hdmap_utils::GermanRoadShoulderPassableVehicle>
  germanRoadShoulderPassableVehicleRules(
    hdmap_utils::Locations::RoadShoulderPassableGermany, lanelet::Participants::Vehicle);

lanelet::traffic_rules::LaneChangeType
hdmap_utils::GermanRoadShoulderPassableVehicle::laneChangeType(
  const lanelet::ConstLineString3d &, bool) const
{
  /// @note allow lane-changes everywhere even if prohibited by lanelet2 map, because lane-change settings are not for entities but only for Autoware.
  return lanelet::traffic_rules::LaneChangeType::Both;
}
