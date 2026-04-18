/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <gtest/gtest.h>

#include <stdexcept>

#include "frontier_exploration_ros2/qos_utils.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{
namespace
{

// Validates public QoS parsing/resolution helpers and startup autodetect state machine.

TEST(CoreDefaultsTests, TopicDefaultsAreRelative)
{
  FrontierExplorerCoreParams params;
  EXPECT_EQ(params.map_topic, "map");
  EXPECT_EQ(params.costmap_topic, "global_costmap/costmap");
  EXPECT_EQ(params.local_costmap_topic, "local_costmap/costmap");
  EXPECT_EQ(params.frontier_marker_topic, "explore/frontiers");
}

TEST(QosUtilsTests, ResolveProfilesParsesAndInheritsLocalCostmap)
{
  const auto profiles = resolve_topic_qos_profiles(
    "transient_local",
    "reliable",
    1,
    "best_effort",
    7,
    "inherit",
    -1);

  EXPECT_EQ(profiles.map_durability, rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(profiles.map_reliability, rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(profiles.map_depth, 1u);
  EXPECT_EQ(profiles.costmap_reliability, rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(profiles.costmap_depth, 7u);
  EXPECT_EQ(profiles.local_costmap_reliability, profiles.costmap_reliability);
  EXPECT_EQ(profiles.local_costmap_depth, profiles.costmap_depth);
  EXPECT_TRUE(profiles.local_costmap_reliability_inherited);
  EXPECT_TRUE(profiles.local_costmap_depth_inherited);

  const auto map_profile = profiles.make_map_qos().get_rmw_qos_profile();
  const auto local_profile = profiles.make_local_costmap_qos().get_rmw_qos_profile();
  EXPECT_EQ(map_profile.depth, 1u);
  EXPECT_EQ(map_profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_EQ(local_profile.depth, 7u);
  EXPECT_EQ(local_profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

// Invalid user-facing QoS values must fail fast with clear exceptions.
TEST(QosUtilsTests, InvalidProfileValueThrows)
{
  EXPECT_THROW(
    static_cast<void>(resolve_topic_qos_profiles(
      "invalid",
      "reliable",
      1,
      "reliable",
      10,
      "inherit",
      -1)),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(resolve_topic_qos_profiles(
      "volatile",
      "reliable",
      0,
      "reliable",
      10,
      "inherit",
      -1)),
    std::invalid_argument);
}

// Startup-only autodetect should switch at most once, then terminate.
TEST(QosUtilsTests, StartupAutodetectSwitchesDurabilityOnceThenStops)
{
  MapQosStartupAutodetect autodetect(true, rclcpp::DurabilityPolicy::TransientLocal);
  ASSERT_TRUE(autodetect.active());
  EXPECT_FALSE(autodetect.fallback_attempted());

  const auto first_switch = autodetect.on_timeout();
  ASSERT_TRUE(first_switch.has_value());
  EXPECT_EQ(*first_switch, rclcpp::DurabilityPolicy::Volatile);
  EXPECT_TRUE(autodetect.active());
  EXPECT_TRUE(autodetect.fallback_attempted());

  const auto second_switch = autodetect.on_timeout();
  EXPECT_FALSE(second_switch.has_value());
  EXPECT_FALSE(autodetect.active());
}

TEST(QosUtilsTests, StartupAutodetectWithoutFallbackStopsAfterTimeout)
{
  MapQosStartupAutodetect autodetect(true, rclcpp::DurabilityPolicy::SystemDefault);
  ASSERT_TRUE(autodetect.active());
  EXPECT_FALSE(autodetect.can_fallback());

  const auto switch_result = autodetect.on_timeout();
  EXPECT_FALSE(switch_result.has_value());
  EXPECT_FALSE(autodetect.active());
}

TEST(QosUtilsTests, StartupAutodetectStopsWhenMapArrives)
{
  MapQosStartupAutodetect autodetect(true, rclcpp::DurabilityPolicy::Volatile);
  ASSERT_TRUE(autodetect.active());

  autodetect.on_map_received();
  EXPECT_FALSE(autodetect.active());
  EXPECT_FALSE(autodetect.on_timeout().has_value());
}

}  // namespace
}  // namespace frontier_exploration_ros2
