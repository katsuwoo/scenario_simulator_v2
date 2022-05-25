// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <quaternion_operation/quaternion_operation.h>

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid.hpp>

namespace simple_sensor_simulator
{
LineSegment::LineSegment(
  const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point)
: start_point(start_point), end_point(end_point)
{
}

LineSegment::~LineSegment() {}

bool LineSegment::intersection2D(const LineSegment & l0) const
{
  double s, t;
  s = (l0.start_point.x - l0.end_point.x) * (start_point.y - l0.start_point.y) -
      (l0.start_point.y - l0.end_point.y) * (start_point.x - l0.start_point.x);
  t = (l0.start_point.x - l0.end_point.x) * (end_point.y - l0.start_point.y) -
      (l0.start_point.y - l0.end_point.y) * (end_point.x - l0.start_point.x);
  if (s * t > 0) {
    return false;
  }
  s = (start_point.x - end_point.x) * (l0.start_point.y - start_point.y) -
      (start_point.y - end_point.y) * (l0.start_point.x - start_point.x);
  t = (start_point.x - end_point.x) * (l0.end_point.y - start_point.y) -
      (start_point.y - end_point.y) * (l0.end_point.x - start_point.x);
  if (s * t > 0) {
    return false;
  }
  return true;
}

GridCell::GridCell(
  const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col)
: origin(origin), size(size), index(index), row(row), col(col)
{
}

std::array<LineSegment, 4> GridCell::getLineSegments() const
{
  geometry_msgs::msg::Point left_up;
  left_up.x = size * 0.5;
  left_up.y = size * 0.5;
  left_up = transformToWorld(left_up);
  geometry_msgs::msg::Point left_down;
  left_down.x = size * 0.5;
  left_down.y = -size * 0.5;
  left_down = transformToWorld(left_down);
  geometry_msgs::msg::Point right_up;
  right_up.x = -size * 0.5;
  right_up.y = size * 0.5;
  right_up = transformToWorld(right_up);
  geometry_msgs::msg::Point right_down;
  right_down.x = -size * 0.5;
  right_down.y = -size * 0.5;
  right_down = transformToWorld(right_down);
  std::array<LineSegment, 4> ret = {
    LineSegment(left_up, left_down), LineSegment(left_down, right_down),
    LineSegment(right_down, right_up), LineSegment(right_up, left_up)};
  return ret;
}

geometry_msgs::msg::Point GridCell::transformToWorld(const geometry_msgs::msg::Point & point) const
{
  auto mat = quaternion_operation::getRotationMatrix(origin.orientation);
  Eigen::VectorXd p(3);
  p(0) = point.x;
  p(1) = point.y;
  p(2) = point.z;
  p = mat * p;
  p(0) = p(0) + origin.position.x;
  p(1) = p(1) + origin.position.y;
  p(2) = p(2) + origin.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = p(0);
  ret.y = p(1);
  ret.z = p(2);
  return ret;
}

bool GridCell::intersection2D(const LineSegment & line) const
{
  for (const auto & outside : getLineSegments()) {
    if (outside.intersection2D(line)) {
      return true;
    }
  }
  return false;
}

bool GridCell::intersection2D(const std::vector<LineSegment> & line_segments) const
{
  for (const auto & line_segment : line_segments) {
    if (intersection2D(line_segment)) {
      return true;
    }
  }
  return false;
}

Grid::Grid(double resolution, double height, double width)
: resolution(resolution), height(height), width(width)
{
}

std::vector<GridCell> Grid::getOccupiedCandidates(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive,
  const geometry_msgs::msg::Pose & sensor_pose) const
{
  std::vector<GridCell> ret;
  const auto x_max = primitive->getMax(simple_sensor_simulator::Axis::X, sensor_pose);
  const auto y_max = primitive->getMax(simple_sensor_simulator::Axis::Y, sensor_pose);
  const auto x_min = primitive->getMin(simple_sensor_simulator::Axis::X, sensor_pose);
  const auto y_min = primitive->getMin(simple_sensor_simulator::Axis::Y, sensor_pose);
  if (!x_max || !y_max || !x_min || !y_min) {
    return ret;
  }
  int x_min_index = std::floor((x_min.get() + resolution * 0.5 * height) / resolution);
  int x_max_index = std::ceil((x_max.get() + resolution * 0.5 * width) / resolution);
  int y_min_index = std::floor((y_min.get() + resolution * 0.5 * height) / resolution);
  int y_max_index = std::ceil((y_max.get() + resolution * 0.5 * width) / resolution);
  for (int x_index = x_min_index; x_index <= x_max_index; x_index++) {
    for (int y_index = y_min_index; y_index <= y_max_index; y_index++) {
      geometry_msgs::msg::Pose cell_origin;
      cell_origin.position.x = sensor_pose.position.x + (x_index - 0.5 * height) * resolution;
      cell_origin.position.y = sensor_pose.position.y + (y_index - 0.5 * width) * resolution;
      cell_origin.position.z = sensor_pose.position.z;
      cell_origin.orientation = sensor_pose.orientation;
      ret.emplace_back(
        GridCell(cell_origin, resolution, width * y_index + x_index, y_index, x_index));
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByRow(const std::vector<GridCell> & cells, size_t row) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.row == row) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByCol(const std::vector<GridCell> & cells, size_t col) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.col == col) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByIntersection(
  const std::vector<GridCell> & cells, const std::vector<LineSegment> & line_segments) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.intersection2D(line_segments)) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<size_t> Grid::getRows(const std::vector<GridCell> & cells)
{
  std::vector<size_t> ret;
  for (const auto & cell : cells) {
    ret.emplace_back(cell.row);
  }
  std::sort(ret.begin(), ret.end());
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

std::vector<size_t> Grid::getCols(const std::vector<GridCell> & cells)
{
  std::vector<size_t> ret;
  for (const auto & cell : cells) {
    ret.emplace_back(cell.col);
  }
  std::sort(ret.begin(), ret.end());
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

std::vector<GridCell> Grid::getCell(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive,
  const geometry_msgs::msg::Pose & sensor_pose) const
{
  auto candidates = getOccupiedCandidates(primitive, sensor_pose);
  // candidates = filterByIntersection();
  return candidates;
}
}  // namespace simple_sensor_simulator
