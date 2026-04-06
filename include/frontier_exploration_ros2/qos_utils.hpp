#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace frontier_exploration_ros2
{

// Normalizes user-provided QoS tokens for case-insensitive parsing.
[[nodiscard]] inline std::string normalize_qos_token(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

// Converts reliability enum to stable config/log token.
[[nodiscard]] inline std::string reliability_policy_to_string(rclcpp::ReliabilityPolicy policy)
{
  switch (policy) {
    case rclcpp::ReliabilityPolicy::Reliable:
      return "reliable";
    case rclcpp::ReliabilityPolicy::BestEffort:
      return "best_effort";
    case rclcpp::ReliabilityPolicy::SystemDefault:
      return "system_default";
    default:
      return "unknown";
  }
}

// Converts durability enum to stable config/log token.
[[nodiscard]] inline std::string durability_policy_to_string(rclcpp::DurabilityPolicy policy)
{
  switch (policy) {
    case rclcpp::DurabilityPolicy::TransientLocal:
      return "transient_local";
    case rclcpp::DurabilityPolicy::Volatile:
      return "volatile";
    case rclcpp::DurabilityPolicy::SystemDefault:
      return "system_default";
    default:
      return "unknown";
  }
}

// Parses reliability policy parameter and validates allowed values.
[[nodiscard]] inline rclcpp::ReliabilityPolicy parse_reliability_policy(
  const std::string & value,
  const std::string & parameter_name)
{
  const std::string normalized = normalize_qos_token(value);
  if (normalized == "reliable") {
    return rclcpp::ReliabilityPolicy::Reliable;
  }
  if (normalized == "best_effort") {
    return rclcpp::ReliabilityPolicy::BestEffort;
  }
  if (normalized == "system_default") {
    return rclcpp::ReliabilityPolicy::SystemDefault;
  }
  throw std::invalid_argument(
          "Invalid value for '" + parameter_name +
          "': '" + value + "'. Expected one of [reliable, best_effort, system_default].");
}

// Parses durability policy parameter and validates allowed values.
[[nodiscard]] inline rclcpp::DurabilityPolicy parse_durability_policy(
  const std::string & value,
  const std::string & parameter_name)
{
  const std::string normalized = normalize_qos_token(value);
  if (normalized == "transient_local") {
    return rclcpp::DurabilityPolicy::TransientLocal;
  }
  if (normalized == "volatile") {
    return rclcpp::DurabilityPolicy::Volatile;
  }
  if (normalized == "system_default") {
    return rclcpp::DurabilityPolicy::SystemDefault;
  }
  throw std::invalid_argument(
          "Invalid value for '" + parameter_name +
          "': '" + value + "'. Expected one of [transient_local, volatile, system_default].");
}

// Parses QoS queue depth parameter with lower-bound validation.
[[nodiscard]] inline std::size_t parse_qos_depth(
  int64_t value,
  const std::string & parameter_name)
{
  if (value < 1) {
    throw std::invalid_argument(
            "Invalid value for '" + parameter_name + "': " + std::to_string(value) +
            ". Expected an integer >= 1.");
  }
  return static_cast<std::size_t>(value);
}

// Resolved QoS policies applied by frontier explorer subscriptions.
struct TopicQosProfiles
{
  rclcpp::DurabilityPolicy map_durability{rclcpp::DurabilityPolicy::TransientLocal};
  rclcpp::ReliabilityPolicy map_reliability{rclcpp::ReliabilityPolicy::Reliable};
  std::size_t map_depth{1};
  rclcpp::ReliabilityPolicy costmap_reliability{rclcpp::ReliabilityPolicy::Reliable};
  std::size_t costmap_depth{10};
  rclcpp::ReliabilityPolicy local_costmap_reliability{rclcpp::ReliabilityPolicy::Reliable};
  std::size_t local_costmap_depth{10};
  bool local_costmap_reliability_inherited{true};
  bool local_costmap_depth_inherited{true};

  // Map uses caller-selected durability to support startup autodetect switches.
  [[nodiscard]] rclcpp::QoS make_map_qos(
    std::optional<rclcpp::DurabilityPolicy> durability_override = std::nullopt) const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(map_depth)};
    qos.reliability(map_reliability);
    qos.durability(durability_override.value_or(map_durability));
    return qos;
  }

  // Costmaps intentionally remain volatile in the public API.
  [[nodiscard]] rclcpp::QoS make_costmap_qos() const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(costmap_depth)};
    qos.reliability(costmap_reliability);
    qos.durability_volatile();
    return qos;
  }

  // Local costmap defaults may inherit reliability/depth from global costmap.
  [[nodiscard]] rclcpp::QoS make_local_costmap_qos() const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(local_costmap_depth)};
    qos.reliability(local_costmap_reliability);
    qos.durability_volatile();
    return qos;
  }
};

// Resolves and validates all user-facing topic QoS parameters.
[[nodiscard]] inline TopicQosProfiles resolve_topic_qos_profiles(
  const std::string & map_qos_durability,
  const std::string & map_qos_reliability,
  int64_t map_qos_depth,
  const std::string & costmap_qos_reliability,
  int64_t costmap_qos_depth,
  const std::string & local_costmap_qos_reliability,
  int64_t local_costmap_qos_depth)
{
  TopicQosProfiles profiles;
  profiles.map_durability = parse_durability_policy(map_qos_durability, "map_qos_durability");
  profiles.map_reliability = parse_reliability_policy(map_qos_reliability, "map_qos_reliability");
  profiles.map_depth = parse_qos_depth(map_qos_depth, "map_qos_depth");
  profiles.costmap_reliability = parse_reliability_policy(
    costmap_qos_reliability,
    "costmap_qos_reliability");
  profiles.costmap_depth = parse_qos_depth(costmap_qos_depth, "costmap_qos_depth");

  const std::string normalized_local_reliability = normalize_qos_token(local_costmap_qos_reliability);
  if (normalized_local_reliability.empty() || normalized_local_reliability == "inherit") {
    profiles.local_costmap_reliability = profiles.costmap_reliability;
    profiles.local_costmap_reliability_inherited = true;
  } else {
    profiles.local_costmap_reliability = parse_reliability_policy(
      local_costmap_qos_reliability,
      "local_costmap_qos_reliability");
    profiles.local_costmap_reliability_inherited = false;
  }

  if (local_costmap_qos_depth < 0) {
    profiles.local_costmap_depth = profiles.costmap_depth;
    profiles.local_costmap_depth_inherited = true;
  } else {
    profiles.local_costmap_depth = parse_qos_depth(local_costmap_qos_depth, "local_costmap_qos_depth");
    profiles.local_costmap_depth_inherited = false;
  }

  return profiles;
}

class MapQosStartupAutodetect
{
public:
  MapQosStartupAutodetect(bool enabled, rclcpp::DurabilityPolicy configured_durability)
  : enabled_(enabled),
    configured_durability_(configured_durability),
    fallback_durability_(fallback_durability_for(configured_durability)),
    active_durability_(configured_durability)
  {
  }

  // True only during startup autodetect window.
  [[nodiscard]] bool active() const noexcept
  {
    return enabled_ && active_;
  }

  // Indicates whether fallback durability was already attempted once.
  [[nodiscard]] bool fallback_attempted() const noexcept
  {
    return fallback_attempted_;
  }

  // User-configured durability from parameters.
  [[nodiscard]] rclcpp::DurabilityPolicy configured_durability() const noexcept
  {
    return configured_durability_;
  }

  // Currently active durability used by the map subscription.
  [[nodiscard]] rclcpp::DurabilityPolicy active_durability() const noexcept
  {
    return active_durability_;
  }

  // Whether this configured durability has a defined opposite fallback.
  [[nodiscard]] bool can_fallback() const noexcept
  {
    return fallback_durability_.has_value();
  }

  // Handles one timeout tick: switch once to fallback, then terminate autodetect.
  [[nodiscard]] std::optional<rclcpp::DurabilityPolicy> on_timeout()
  {
    if (!active()) {
      return std::nullopt;
    }

    if (!fallback_attempted_ && fallback_durability_.has_value()) {
      fallback_attempted_ = true;
      active_durability_ = *fallback_durability_;
      return active_durability_;
    }

    active_ = false;
    return std::nullopt;
  }

  // Stops autodetect once a valid map message is received.
  void on_map_received()
  {
    active_ = false;
  }

private:
  static std::optional<rclcpp::DurabilityPolicy> fallback_durability_for(
    rclcpp::DurabilityPolicy durability)
  {
    if (durability == rclcpp::DurabilityPolicy::TransientLocal) {
      return rclcpp::DurabilityPolicy::Volatile;
    }
    if (durability == rclcpp::DurabilityPolicy::Volatile) {
      return rclcpp::DurabilityPolicy::TransientLocal;
    }
    return std::nullopt;
  }

  bool enabled_{false};
  bool active_{true};
  bool fallback_attempted_{false};
  rclcpp::DurabilityPolicy configured_durability_{rclcpp::DurabilityPolicy::TransientLocal};
  std::optional<rclcpp::DurabilityPolicy> fallback_durability_;
  rclcpp::DurabilityPolicy active_durability_{rclcpp::DurabilityPolicy::TransientLocal};
};

}  // namespace frontier_exploration_ros2
