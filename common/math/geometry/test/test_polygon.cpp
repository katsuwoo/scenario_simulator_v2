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

#include <gtest/gtest.h>

#include <geometry/polygon/polygon.hpp>
#include <geometry/test/expect_eq_macros.hpp>
#include <geometry/test/test_utils.hpp>
#include <scenario_simulator_exception/exception.hpp>

TEST(Polygon, filterByAxis)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(5, 2, 3), makePoint(1, 4, 5), makePoint(-1, 2, -3)};

  std::vector<double> values_x = math::geometry::filterByAxis(points, math::geometry::Axis::X);
  EXPECT_DOUBLE_EQ(values_x[0], 5);
  EXPECT_DOUBLE_EQ(values_x[1], 1);
  EXPECT_DOUBLE_EQ(values_x[2], -1);

  std::vector<double> values_y = math::geometry::filterByAxis(points, math::geometry::Axis::Y);
  EXPECT_DOUBLE_EQ(values_y[0], 2);
  EXPECT_DOUBLE_EQ(values_y[1], 4);
  EXPECT_DOUBLE_EQ(values_y[2], 2);

  std::vector<double> values_z = math::geometry::filterByAxis(points, math::geometry::Axis::Z);
  EXPECT_DOUBLE_EQ(values_z[0], 3);
  EXPECT_DOUBLE_EQ(values_z[1], 5);
  EXPECT_DOUBLE_EQ(values_z[2], -3);
}

TEST(Polygon, filterByAxisEmptyVector)
{
  std::vector<geometry_msgs::msg::Point> points;

  std::vector<double> values_x = math::geometry::filterByAxis(points, math::geometry::Axis::X);
  EXPECT_TRUE(values_x.empty());

  std::vector<double> values_y = math::geometry::filterByAxis(points, math::geometry::Axis::Y);
  EXPECT_TRUE(values_y.empty());

  std::vector<double> values_z = math::geometry::filterByAxis(points, math::geometry::Axis::Z);
  EXPECT_TRUE(values_z.empty());
}

TEST(Polygon, getMaxValue)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(5, 2, 3), makePoint(1, 4, 5), makePoint(-1, 2, -3)};
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::X), 5);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::Y), 4);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::Z), 5);
}

TEST(Polygon, getMaxValueEmptyVector)
{
  std::vector<geometry_msgs::msg::Point> points;
  EXPECT_THROW(
    math::geometry::getMaxValue(points, math::geometry::Axis::X), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMaxValue(points, math::geometry::Axis::Y), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMaxValue(points, math::geometry::Axis::Z), common::SimulationError);
}

TEST(Polygon, getMinValue)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(5, 2, 3), makePoint(1, 4, 5), makePoint(-1, 2, -3)};
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::X), -1);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::Y), 2);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::Z), -3);
}

TEST(Polygon, getMinValueEmptyVector)
{
  std::vector<geometry_msgs::msg::Point> points;
  EXPECT_THROW(
    math::geometry::getMinValue(points, math::geometry::Axis::X), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMinValue(points, math::geometry::Axis::Y), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMinValue(points, math::geometry::Axis::Z), common::SimulationError);
}

TEST(Polygon, get2DConvexHull)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(2, 2), makePoint(-2, 2), makePoint(-2, -2), makePoint(-1, 0)};
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_EQ(hull.size(), static_cast<size_t>(4));
  EXPECT_POINT_EQ(hull[0], points[2]);
  EXPECT_POINT_EQ(hull[1], points[1]);
  EXPECT_POINT_EQ(hull[2], points[0]);
  EXPECT_POINT_EQ(hull[3], points[2]);
}

TEST(Polygon, get2DConvexHullIdle)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(2, 2), makePoint(-2, 2), makePoint(-2, -2), makePoint(2, -2)};
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_EQ(hull.size(), static_cast<size_t>(5));
  EXPECT_POINT_EQ(hull[0], points[2]);
  EXPECT_POINT_EQ(hull[1], points[1]);
  EXPECT_POINT_EQ(hull[2], points[0]);
  EXPECT_POINT_EQ(hull[3], points[3]);
  EXPECT_POINT_EQ(hull[4], points[2]);
}

TEST(Polygon, get2DConvexHullEmpty)
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_TRUE(hull.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
