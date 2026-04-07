# frontier_exploration_ros2

[![Contributors](https://img.shields.io/github/contributors/mertgulerx/frontier-exploration-ros2?style=flat-square)](https://github.com/mertgulerx/frontier-exploration-ros2/graphs/contributors)
[![Stars](https://img.shields.io/github/stars/mertgulerx/frontier-exploration-ros2?style=flat-square)](https://github.com/mertgulerx/frontier-exploration-ros2/stargazers)
[![Issues](https://img.shields.io/github/issues/mertgulerx/frontier-exploration-ros2?style=flat-square)](https://github.com/mertgulerx/frontier-exploration-ros2/issues)
[![License](https://img.shields.io/github/license/mertgulerx/frontier-exploration-ros2?style=flat-square)](https://github.com/mertgulerx/frontier-exploration-ros2/blob/main/LICENSE)

`frontier_exploration_ros2` is a modern C++ frontier exploration package for ROS 2 Jazzy and Nav2. It implements a Wavefront Frontier Detector (WFD) style exploration pipeline, exposes a ready-to-run `frontier_explorer` node, and ships a reusable C++ core library for teams that need tighter control over integration.

The package is written for ROS 2 Jazzy, but it is intentionally structured to keep the exploration logic clean, transport decisions explicit, and system-level integration straightforward. That makes it practical for Nav2 deployments, custom ROS 2 stacks, and future adaptations where the core exploration logic needs to be reused with minimal friction.

In internal comparisons against Python-based frontier exploration packages under similar workloads, the C++ implementation has typically shown about 40% lower runtime overhead.

This is not just a direct C++ port. It also includes impressive performance improvements with methods such as reusable caches, less repeated computations, and controlled memory usage for long-running explorations. With extra features such as preemption to improve overall effectiveness for autonomous mobile robots.

## Table of Contents

- [Overview](#overview)
- [Design Goals](#design-goals)
- [Status](#status)
- [Verified Environment](#verified-environment)
- [Architecture](#architecture)
- [Algorithm and Mathematics](#algorithm-and-mathematics)
- [Why This Package Is Different](#why-this-package-is-different)
- [Installation and Build](#installation-and-build)
- [Quick Start](#quick-start)
- [Integration Guide](#integration-guide)
- [QoS Configuration](#qos-configuration)
- [Launch File Reference](#launch-file-reference)
- [Parameter Reference](#parameter-reference)
- [TurtleBot3 Waffle Pi Example](#turtlebot3-waffle-pi-example)
- [Testing](#testing)
- [Contributing](#contributing)
- [License](#license)
- [Maintainer](#maintainer)

## Overview

This package solves autonomous frontier exploration for occupancy-grid-based mobile robots. It detects frontiers on the boundary between known free space and unknown space, selects a reachable frontier goal, sends that goal to Nav2, tracks map and costmap updates while the robot moves, and continues until frontier exhaustion.

The package is based on the WFD idea described in the paper "Frontier Based Exploration for Autonomous Robot" ([arXiv:1806.03581](https://arxiv.org/abs/1806.03581)). This implementation keeps the WFD-style core search model, then extends it with integration and runtime improvements that are useful in modern production stacks:

- global and local costmap filtering during frontier validation and goal selection
- startup-only map QoS autodetect for first-time integration
- frontier snapshot caching and deterministic frontier signatures
- post-goal settle logic to avoid immediate instability after a goal is reached
- explicit preemption controls for newly revealed frontiers and blocked goals, including optional gain-gated revealed preemption
- optional frontier suppression and suppressed-frontier waiting behavior
- optional completion-event publishing for external orchestration
- optional return-to-start behavior after frontier exhaustion
- reusable C++ library export for custom integration paths

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Design Goals

- Provide a C++ exploration package that is fast, predictable, and easy to integrate into ROS 2 Jazzy systems.
- Stay Nav2-first at the node boundary while keeping the decision logic separated from project-specific backends.
- Expose a clean parameter surface for topics, frames, QoS, goal behavior, and completion hooks.
- Make namespace-aware deployment and multi-robot integration practical.
- Keep the package public and universal. The package should be usable without assuming a specific robot, simulator, map saver, or private stack.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Status

This package is written and tested for ROS 2 Jazzy.

That target is explicit in the build, launch, and dependency surface. Future ports to other ROS 2 distributions can be considered on demand, but Jazzy is the primary supported target at this stage.

Even though the deliverable is a ROS 2 package, the implementation encourages reuse beyond a single ROS 2 application layout:

- the exploration logic lives in a reusable C++ core target
- transport details are kept at the node boundary
- completion handling is externalized instead of hard-coded
- QoS and topic contracts are explicit instead of being buried in code

In practice, that makes the package easier to reuse in custom ROS 2 stacks and lowers the cost of adapting the core ideas to non-ROS runtimes in later work.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Version History

| Version  | Summary                                                                              |
| -------- | ------------------------------------------------------------------------------------ |
| `v1.0.0` | First release                                                                        |
| `v1.1.0` | Added gain-gated preemption to reduce path complexity and optimize traveled distance |

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Verified Environment

The current implementation has been validated in a ROS 2 Jazzy exploration stack built around:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2
- Slam Toolbox
- TurtleBot3 Waffle Pi

This is not the only supported integration path, but it is a verified environment in which the package has successfully completed frontier exploration with the expected map, costmap, TF, and navigation flow.

LiDAR quality is especially important for this package because frontier extraction, blocked-goal checks, SLAM stability, and navigation progress all depend on the quality of the incoming scan data.

The verified TurtleBot3 Waffle Pi setup uses the ROBOTIS LDS-03 2D LiDAR. The LDS-03 provides 360-degree scanning and an official distance range of approximately `0.05 m` to `12.0 m`. In practice, exploration quality depends not only on the published sensor range, but also on mounting, scan cleanliness, environment reflectivity, filtering, and the usable range chosen by the navigation stack.

Exploration parameters can be strongly affected by LiDAR characteristics, so they should be tuned carefully for the actual sensor and environment.

The TurtleBot3 Waffle Pi is also a relatively small and slow robot, so parameter values may need to change on faster platforms.

![Frontier exploration demo with point-reveal preemption on a TurtleBot3 Waffle Pi](https://github.com/mertgulerx/readme-assets/blob/main/frontier-exploration/mertgulerx-frontier-exploration-point-reveal-preemption-example-on-a-turtlebot3-waffle-pi.gif)

> [!TIP]
> This demo utilizes aggressive preemption settings with a **point-based** method to accelerate the exploration flow in high-obstacle environments.

![Frontier exploration demo with gain-gate preemption on a TurtleBot3 Waffle Pi](https://github.com/mertgulerx/readme-assets/blob/main/frontier-exploration/mertgulerx-frontier-exploration-gain-gate-preemption-example-on-a-turtlebot3-waffle-pi.gif)

> [!TIP]
> This demo utilizes a smart **gain-gate preemption** method to reduce path & maneuver complexity and efficiency (Introduced in v1.1.0)

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Why This Package Is Different

- It targets ROS 2 Jazzy directly instead of being a legacy ROS 1 port with minimal adaptation.
- It is written in C++, which improves runtime efficiency and integration with performance-sensitive navigation stacks.
- It exports a reusable C++ core library instead of only shipping a monolithic node.
- It uses both global and local costmaps during frontier validation and goal selection.
- It implements startup-only QoS autodetect for map durability mismatches.
- It uses deterministic frontier signatures and snapshot caching to reduce repeated work.
- It includes settle logic and stable replacement checks to reduce unnecessary replanning.
- It supports both point/snapshot-based and target-pose visible-gain-gated revealed preemption.
- It keeps completion handling external and generic instead of coupling the package to project-specific post-processing.
- It enables IPO/LTO in `Release` and `RelWithDebInfo` builds when supported by the toolchain.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Architecture

The package ships with four main pieces:

- `frontier_explorer`: public executable that subscribes to map and costmap topics, queries TF, and talks to Nav2 `NavigateToPose`.
- `frontier_exploration_ros2::frontier_exploration_ros2_core`: reusable C++ core library that contains frontier search, selection, goal-state handling, settle logic, revealed/blocked-goal preemption policy, and suppression orchestration.
- `launch/frontier_explorer.launch.py`: package-owned example launch file.
- `config/params.yaml`: packaged baseline parameter file.

At runtime, the node expects:

- an occupancy map topic
- a global costmap topic
- a local costmap topic
- a TF transform from `global_frame` to `robot_base_frame`
- a Nav2 `navigate_to_pose` action server, or another action server exposed under the configured action name

The package can also publish a completion event through `std_msgs/msg/Empty`. This is intentionally optional and transport-light. The explorer only reports completion. Any map export, mission chaining, docking, or higher-level orchestration should be implemented outside the package.

When suppression is enabled, the core can temporarily exclude repeatedly failing frontier areas and optionally wait under a temporary return-to-start goal while all currently detected frontiers remain suppressed. That temporary return path is separate from normal exploration completion.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Algorithm and Mathematics

### Frontier Definition

The package follows the classical frontier idea: a frontier is an unknown cell that borders free space.

The eligibility test implemented in the search layer can be summarized as:

```text
frontier(p) is true if:
  map(p) = unknown
  and there exists q in N(p) such that map(q) = free
  and there does not exist q in N(p) blocked by the global or local costmap
```

`N(p)` is the local neighborhood around cell `p`. The implementation uses an 8-connected neighborhood plus the center cell for local scans.

### WFD-Style Two-Level Search

The core search is based on the WFD paper ([arXiv:1806.03581](https://arxiv.org/abs/1806.03581)). The code uses two BFS layers:

1. A map-space BFS expands through reachable map cells.
2. When a frontier cell is found, a frontier BFS grows that connected frontier cluster.

The search flow is:

```text
1. Project the robot pose into map coordinates.
2. If the robot starts in unknown or occupied space, recover the nearest free seed.
3. Expand a BFS over reachable map cells.
4. When a frontier cell is detected, grow the full connected frontier cluster.
5. Reject tiny clusters.
6. Build one frontier candidate from that cluster.
7. Repeat until the reachable map area is exhausted.
```

This preserves the core WFD idea while adapting it to modern ROS 2 exploration stacks that rely on both global and local costmaps.

### Centroid and Goal-Point Selection

Each frontier cluster is converted into a navigation candidate with:

- a centroid used for ranking and equivalence checks
- a goal point used for actual navigation
- a cluster size

The centroid is computed directly from frontier cells:

```text
cx = (1 / N) * sum(x_i)
cy = (1 / N) * sum(y_i)
```

The goal point is not chosen from unknown cells. Instead, the code searches the free and unblocked neighbors around frontier cells and selects a reachable navigation point close to the frontier centroid.

When minimum robot distance gating is active, goal selection follows this rule:

```text
choose g that minimizes ||g - c||^2
subject to:
  g is free
  g is not blocked in the global costmap
  g is not blocked in the local costmap
  and optionally ||g - r||^2 >= d_min^2
```

Where:

- `c` is the frontier centroid
- `g` is the candidate goal point
- `r` is the robot position
- `d_min` is `frontier_min_distance`

If no goal candidate satisfies the distance constraint, the implementation falls back to the best unconstrained reachable point.

### Costmap Blocking Policy

The package uses both global and local costmaps during search and during active-goal monitoring.

The blocking rule is intentionally strict:

```text
blocked(world_point) is true if any active costmap reports cost > OCC_THRESHOLD
```

This affects two stages:

- frontier eligibility: unknown cells next to blocked neighbors are not treated as valid frontiers
- goal-point materialization: free neighbors that are blocked in either costmap are rejected

This is one of the key extensions beyond a plain frontier-only implementation. It improves robustness in cluttered environments and reduces the number of goals that are technically frontier-adjacent but not operationally navigable.

### Frontier Selection Policy

After frontier extraction, the node selects a target according to the current policy:

- preferred frontier: closest frontier that still satisfies `frontier_min_distance`
- escape frontier: farthest candidate when no preferred frontier is available and escape mode is still active
- close-range fallback: farthest remaining candidate when the preferred path is unavailable

The escape path is deliberate. It helps the robot move away from a locally congested or trivial region until the first successful frontier goal is completed.
When a second frontier is available, the dispatched goal orientation is also biased toward that next frontier to reduce unnecessary reorientation after arrival.

### Deterministic Frontier Signatures

The implementation quantizes frontier reference points and goal points into a stable signature:

```text
q = max(frontier_visit_tolerance, 0.1)
signature(F) = sort(round(ref_x / q), round(ref_y / q), round(goal_x / q), round(goal_y / q))
```

This signature is used for:

- snapshot cache validation
- stable publish suppression for RViz markers
- post-goal settle checks

It reduces noise from small floating-point jitter and helps keep runtime behavior deterministic.

### Post-Goal Settle Logic

After a frontier goal succeeds, the node does not immediately send the next goal. Instead it waits until the map has settled.

The readiness rule is:

```text
ready if:
  map_updated
  and elapsed >= post_goal_min_settle
  and map_updates_seen >= post_goal_required_map_updates
  and stable_update_count >= post_goal_stable_updates
```

This is a practical improvement over simpler exploration loops that can re-goal too quickly while SLAM or costmaps are still converging.

### Preemption and Blocked-Goal Design

Two parameters are deliberately exposed as policy controls:

- `goal_preemption_on_frontier_revealed`
- `goal_preemption_on_blocked_goal`

They are design choices, not incidental flags.

`goal_preemption_on_frontier_revealed` allows the active frontier goal to be reconsidered when the frontier becomes revealed on a map update and a better target becomes available.

`goal_preemption_on_blocked_goal` allows the active frontier goal to be canceled or replaced when the target becomes blocked in the local or global costmap.

Revealed preemption has two practical modes:

- point/snapshot-based revealed preemption: the active frontier is rechecked against the refreshed frontier set, and reselection is allowed when that frontier no longer appears there
- gain-gated revealed preemption: before falling back to the same snapshot test, the target pose is checked with an occlusion-aware visible frontier gain estimate so the current goal can be kept while it still promises useful reveal on arrival

The current implementation also applies:

- a minimum time gap between frontier-revealed preemption attempts
- a distance gate that avoids switching when the robot is already close to the current target
- an optional completion distance that can treat a near-arrived frontier as finished before either revealed-preemption path evaluates further
- a stability requirement so replacement candidates must be observed repeatedly before a switch is made

Together, these controls help the explorer stay responsive without turning preemption into unstable goal switching.

`goal_preemption_complete_if_within_m` is especially useful when the visible-gain gate would otherwise be too aggressive near arrival, because it lets the system accept "close enough" as effectively complete and move on cleanly. It should still be used carefully on robots that prioritize conservative obstacle avoidance: if this threshold is set too large, the robot can mark a frontier complete before the intended sensing pose is meaningfully reached, which can in turn produce wrong exploration transitions.

### Frontier Suppression and Failure Memory

Suppression is an optional policy layer that temporarily removes repeatedly failing frontier goal areas from consideration without marking exploration as complete.

The implementation is reason-agnostic at the policy level. It does not try to infer whether a failure came from Nav2, another action backend, a planner limitation, or a temporary bring-up condition. Instead it tracks repeated failed attempts at the frontier goal point and promotes those failures into temporary square suppression regions.

The attempt key is quantized with the same tolerance family used elsewhere in the core:

```text
q = max(frontier_visit_tolerance, 0.1)
attempt_key(g) = (round(g_x / q), round(g_y / q))
```

Where `g` is the actual navigation goal point, not the frontier centroid.

After a configurable number of failed attempts, the implementation creates a square suppression region:

```text
center = g
side_length = frontier_suppression_base_size_m
```

If another matured failed goal lands in the outer expansion band of an existing region, the region grows instead of creating a second nearby fragment:

```text
inner square  = C x C
outer square  = (C + B) x (C + B)
if g is in outer - inner:
  new_center = midpoint(old_center, g)
  new_side_length = 2 * C
```

Where:

- `C` is the current region side length
- `B` is `frontier_suppression_expansion_size_m`

Membership is evaluated with the selected goal point. The suppression policy does not use the centroid for region filtering.

The suppression state is also bounded by design:

- failed attempt records are stored in a capped hash map
- suppression regions are stored in a capped vector
- both use TTL-based cleanup
- both use oldest-entry eviction if the configured hard cap is reached

That keeps the feature operationally safe for long-running deployments.

### No-Progress Detection

Suppression also includes a no-progress watchdog for accepted frontier goals.

The watchdog reads `distance_remaining` from the action feedback stream and tracks the best observed value. A goal is considered to have made meaningful progress only when the best distance improves by at least `frontier_suppression_progress_epsilon_m` since the last meaningful progress point.

The rule can be summarized as:

```text
meaningful_progress if:
  distance_at_last_progress - best_distance_remaining >= progress_epsilon
```

The timeout rule is:

```text
no_progress if:
  now - last_meaningful_progress_time >= no_progress_timeout
```

There is no separate time window attached to `frontier_suppression_progress_epsilon_m`. The expected interpretation is:

- achieve at least that much meaningful improvement
- before `frontier_suppression_no_progress_timeout_s` expires

If that does not happen, the active frontier goal is canceled and counted as a failed attempt.

### Startup Grace Period

Suppression is intentionally delayed during startup when `frontier_suppression_startup_grace_period_s` is greater than zero.

During that grace period:

- suppression filtering is not applied
- failed frontier attempts are not recorded
- no-progress timeout cancelation is not active

This protects first integration and real-world bring-up scenarios where Nav2, TF, costmaps, or another navigation backend may become fully ready slightly after the explorer starts.

When the grace period elapses, suppression becomes active and the node reports that transition in the logs. This startup delay is separate from map QoS autodetect, but the two features are complementary: QoS autodetect helps the explorer receive the map correctly, while startup grace prevents early navigation instability from poisoning suppression memory.

### Suppressed-Frontier Waiting Policy

`All currently detected frontiers are temporarily suppressed` is not the same condition as `No more frontiers found`.

The first means:

- frontier candidates still exist in the map
- but every currently detected candidate is temporarily excluded by suppression

The second means:

- the current frontier search found no remaining frontier candidates

When all current frontier candidates are suppressed, the package uses `all_frontiers_suppressed_behavior`:

- `stay`: remain in place and wait for new map or costmap changes
- `return_to_start`: temporarily navigate back to the recorded start pose while waiting

This temporary return-to-start path is not the same as `return_to_start_on_complete`:

- it does not mean exploration is finished
- it does not mark the mission complete
- it does not replace normal frontier exhaustion handling
- it can be canceled automatically if usable frontier candidates appear again

If frontiers become available again while the robot is moving under this temporary return goal, the explorer cancels that return goal and resumes frontier exploration.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Installation and Build

### Dependencies

Runtime and build dependencies include:

- ROS 2 Jazzy
- Nav2
- TF2
- `geometry_msgs`
- `nav_msgs`
- `nav2_msgs`
- `visualization_msgs`
- `action_msgs`

### Clone

```bash
cd <your_workspace>/src
git clone https://github.com/mertgulerx/frontier-exploration-ros2.git
```

### Install Dependencies

```bash
cd <your_workspace>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
cd <your_workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select frontier_exploration_ros2
source install/setup.bash
```

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Quick Start

> [!WARNING]
> WFD-based frontier extraction tends to select points close to obstacles. It is strongly recommended to enable both `goal_preemption_on_frontier_revealed` and `goal_preemption_on_blocked_goal` in your runtime configuration. Default packaged config comes with disabled params for compatibility reasons.

Launch with the packaged parameter file:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py
```

Override the parameter file:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  params_file:=/absolute/path/to/params.yaml
```

Run under a namespace:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  namespace:=robot1
```

Enable simulation time:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true
```

Override startup map durability:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  map_qos_durability:=volatile
```

Enable startup-only map QoS autodetect:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  map_qos_autodetect_on_startup:=true \
  map_qos_autodetect_timeout_s:=2.0
```

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Integration Guide

### Required Interfaces

The node expects the following interfaces to exist in the running system:

| Interface             | Default                  | Purpose                                 |
| --------------------- | ------------------------ | --------------------------------------- |
| Occupancy grid topic  | `map`                    | Frontier extraction                     |
| Global costmap topic  | `global_costmap/costmap` | Reachability and blocked-goal filtering |
| Local costmap topic   | `local_costmap/costmap`  | Near-field blocked-goal filtering       |
| Nav2 action           | `navigate_to_pose`       | Goal execution                          |
| TF transform          | `map -> base_footprint`  | Robot pose lookup                       |
| Frontier marker topic | `explore/frontiers`      | Visualization                           |

### Topic and Frame Mapping

For most integrations, only these fields need to be remapped:

- `map_topic`
- `costmap_topic`
- `local_costmap_topic`
- `navigate_to_pose_action_name`
- `global_frame`
- `robot_base_frame`
- `frontier_marker_topic`

All defaults are relative topic names inside the package-owned baseline config. That keeps the package namespace-friendly in multi-robot deployments. If your system uses absolute topic names, provide them explicitly in your parameter file.

### Nav2 Integration

The public node is designed around `nav2_msgs/action/NavigateToPose`.

The explorer:

- waits for the action server with a bounded timeout
- dispatches one frontier goal at a time
- receives feedback and result callbacks
- can request cancelation during preemption flows

If your stack wraps Nav2 behind a namespace or a remapped action name, update `navigate_to_pose_action_name` accordingly.

If suppression is enabled, repeated rejected goals, aborted goals, or no-progress timeout cancelations can temporarily remove a frontier area from selection.

### Multi-Robot and Namespace Use

The launch file accepts a `namespace` argument and all packaged topic defaults are relative. That makes the package suitable for:

- one explorer per robot namespace
- one Nav2 stack per robot namespace
- one map and costmap graph per robot namespace

Example:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  namespace:=robot1 \
  params_file:=/absolute/path/to/robot1_frontier.yaml
```

### Completion Event Consumption

When `completion_event_enabled` is `true`, the explorer publishes one `std_msgs/msg/Empty` message when frontier exhaustion is observed.

Completion event QoS:

- durability: `transient_local`
- reliability: `reliable`
- depth: `1`

Design intent:

- the explorer reports completion
- external systems decide what to do next
- late-joining subscribers can still see the latest completion event while the explorer node remains alive

Typical consumers include:

- map export services
- docking or return-home supervisors
- mission sequencers
- test automation scripts

Suppressed return-to-start is different. If `all_frontiers_suppressed_behavior=return_to_start`, the robot may temporarily go back to the start pose while waiting for new frontier opportunities, but that does not publish a completion event and does not mean exploration has finished.

### Reusing the C++ Core Library

The package exports a reusable C++ target:

```cmake
find_package(frontier_exploration_ros2 REQUIRED)

add_executable(my_explorer src/my_explorer.cpp)
target_link_libraries(my_explorer
  frontier_exploration_ros2::frontier_exploration_ros2_core
)
```

The core keeps the action-transport logic, frontier search, and policy flow separated well enough for advanced users who want to embed the exploration logic in a custom ROS 2 node.

### Suppression Behavior in Practice

Suppression is most useful in systems where the same frontier can fail repeatedly for operational reasons that are not visible in the occupancy map alone. Common examples include:

- a navigation stack that comes up later than the explorer
- a planner that keeps rejecting a narrow or unstable goal area
- a controller that stalls near clutter without making meaningful progress
- a temporary map or costmap inconsistency during bring-up

In those cases, suppression adds three controls:

- startup grace delays suppression until the initial system bring-up window has passed
- failure memory temporarily removes repeatedly failing goal areas from reselection
- optional temporary return-to-start behavior avoids treating the situation as exploration complete

If the frontier set changes and a usable candidate appears again, the temporary return-to-start goal is canceled and frontier exploration resumes automatically.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## QoS Configuration

### Supported User-Facing Tokens

Accepted durability values:

- `transient_local`
- `volatile`
- `system_default`

Accepted reliability values:

- `reliable`
- `best_effort`
- `system_default`

### Default QoS Profiles

| Channel          | Default durability | Default reliability           | Default depth                 |
| ---------------- | ------------------ | ----------------------------- | ----------------------------- |
| Map              | `transient_local`  | `reliable`                    | `1`                           |
| Global costmap   | `volatile`         | `reliable`                    | `10`                          |
| Local costmap    | `volatile`         | `inherit` from global costmap | `inherit` from global costmap |
| Completion event | `transient_local`  | `reliable`                    | `1`                           |

The local costmap subscriber always uses volatile durability. Reliability and depth can either inherit the global costmap settings or be overridden explicitly.

### Startup-Only Map Durability Autodetect

If the correct map durability is not known during first integration, the package can probe it at startup.

Behavior:

1. Start with the configured map durability.
2. Wait for `map_qos_autodetect_timeout_s`.
3. If no map is received and the configured durability is `transient_local` or `volatile`, switch once to the opposite durability.
4. If a map arrives, lock that choice and stop autodetect.
5. If neither attempt succeeds, stop autodetect and report failure.

This process is strictly startup-only. It does not keep switching at runtime.

### Autodetect Log Meanings

The node emits machine-parsable logs with a fixed prefix:

```text
[qos-autodetect] START selected=<durability> timeout=<seconds>
[qos-autodetect] SWITCH selected=<durability> elapsed=<seconds>
[qos-autodetect] COMPLETE result=<initial|fallback|failed> selected=<durability> elapsed=<seconds>
```

Interpretation:

- `result=initial`: the first configured durability worked
- `result=fallback`: the fallback durability worked
- `result=failed`: neither attempt received a map within the startup window

Recommended integration sequence:

1. Enable autodetect once during bring-up.
2. Read the `COMPLETE` log.
3. Copy the working durability into your permanent parameter file.
4. Disable autodetect for regular deployments.

Suppression does not define its own QoS policy, but it is still relevant during first integration. If map QoS is wrong or the navigation stack comes up late, the explorer can observe unstable early behavior. The recommended pattern is:

1. fix map QoS first
2. use startup grace while validating navigation bring-up timing
3. then tune suppression thresholds only after the transport layer is stable

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Launch File Reference

Launch file: `launch/frontier_explorer.launch.py`

| Argument                        | Default                       | Effect                                        | Overrides YAML    |
| ------------------------------- | ----------------------------- | --------------------------------------------- | ----------------- |
| `namespace`                     | `""`                          | Runs the node inside a ROS namespace          | No                |
| `params_file`                   | packaged `config/params.yaml` | Selects the parameter file                    | Replaces the file |
| `use_sim_time`                  | `false`                       | Passes standard ROS simulation time parameter | Yes               |
| `log_level`                     | `info`                        | Sets node log severity                        | No                |
| `map_qos_durability`            | `transient_local`             | Overrides map durability                      | Yes               |
| `map_qos_autodetect_on_startup` | `false`                       | Enables startup autodetect                    | Yes               |
| `map_qos_autodetect_timeout_s`  | `2.0`                         | Sets timeout per autodetect attempt           | Yes               |
| `costmap_qos_reliability`       | `reliable`                    | Overrides global costmap reliability          | Yes               |

Notes:

- `params_file` controls the full YAML source for the node.
- the launch file only overrides the four QoS-related parameters listed above, plus `use_sim_time`
- all other node behavior is defined by the selected parameter file
- suppression behavior, startup grace, and suppressed-frontier waiting policy are configured in YAML, not through dedicated launch arguments

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Parameter Reference

Parameters are declared in `frontier_explorer_node.cpp`.

The packaged launch path uses `config/params.yaml` as its baseline parameter file, so launched behavior may differ from the declared defaults where that YAML intentionally overrides them.

### Topics and Frames

| Parameter                      | Type     | Default                  | Description                                       | Notes                                                 |
| ------------------------------ | -------- | ------------------------ | ------------------------------------------------- | ----------------------------------------------------- |
| `map_topic`                    | `string` | `map`                    | Occupancy grid topic used for frontier extraction | Relative by default for namespace-friendly deployment |
| `costmap_topic`                | `string` | `global_costmap/costmap` | Global costmap topic                              | Used during search and blocked-goal checks            |
| `local_costmap_topic`          | `string` | `local_costmap/costmap`  | Local costmap topic                               | Used for near-field blocking checks                   |
| `navigate_to_pose_action_name` | `string` | `navigate_to_pose`       | Action name used for navigation goals             | Must resolve to a `NavigateToPose` action server      |
| `global_frame`                 | `string` | `map`                    | Global frame used for goals and TF lookups        | Must exist in TF                                      |
| `robot_base_frame`             | `string` | `base_footprint`         | Robot body frame used for TF lookups              | Must exist in TF                                      |
| `frontier_marker_topic`        | `string` | `explore/frontiers`      | Marker topic for frontier visualization           | Publishes goal-point markers                          |

### Visualization

| Parameter               | Type     | Default | Description                                   | Notes                             |
| ----------------------- | -------- | ------- | --------------------------------------------- | --------------------------------- |
| `frontier_marker_scale` | `double` | `0.15`  | Sphere marker size for frontier visualization | Used by the RViz marker publisher |

### QoS

| Parameter                       | Type     | Default           | Description                                    | Notes                                                    |
| ------------------------------- | -------- | ----------------- | ---------------------------------------------- | -------------------------------------------------------- |
| `map_qos_durability`            | `string` | `transient_local` | Map durability policy                          | Allowed: `transient_local`, `volatile`, `system_default` |
| `map_qos_reliability`           | `string` | `reliable`        | Map reliability policy                         | Allowed: `reliable`, `best_effort`, `system_default`     |
| `map_qos_depth`                 | `int`    | `1`               | Map subscription queue depth                   | Must be `>= 1`                                           |
| `map_qos_autodetect_on_startup` | `bool`   | `false`           | Enables startup-only map durability autodetect | Switches at most once, then stops                        |
| `map_qos_autodetect_timeout_s`  | `double` | `2.0`             | Timeout per autodetect attempt in seconds      | Internally clamped to at least `0.2`                     |
| `costmap_qos_reliability`       | `string` | `reliable`        | Global costmap reliability policy              | Allowed: `reliable`, `best_effort`, `system_default`     |
| `costmap_qos_depth`             | `int`    | `10`              | Global costmap queue depth                     | Must be `>= 1`                                           |
| `local_costmap_qos_reliability` | `string` | `inherit`         | Local costmap reliability policy               | `inherit` copies the global costmap reliability          |
| `local_costmap_qos_depth`       | `int`    | `-1`              | Local costmap queue depth                      | Negative values mean inherit from global costmap         |

### Exploration Behavior

| Parameter                                            | Type     | Default | Description                                                                                    | Notes                                                                                                                                                                                                               |
| ---------------------------------------------------- | -------- | ------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `frontier_min_distance`                              | `double` | `0.5`   | Minimum robot-to-goal distance preferred during frontier selection                             | If no far-enough goal exists, the code falls back to the best reachable candidate                                                                                                                                   |
| `frontier_visit_tolerance`                           | `double` | `0.30`  | Tolerance used for frontier equivalence and already-visited checks                             | Also drives quantized frontier signatures                                                                                                                                                                           |
| `goal_preemption_on_frontier_revealed`               | `bool`   | `false` | Allows reselection when the active frontier becomes revealed while a frontier goal is active   | Deliberate design choice for opportunistic replanning                                                                                                                                                               |
| `goal_preemption_on_blocked_goal`                    | `bool`   | `false` | Allows cancelation or replacement when the active goal becomes blocked                         | Deliberate design choice for safety and progress                                                                                                                                                                    |
| `goal_preemption_min_interval_s`                     | `double` | `2.0`   | Minimum time between frontier-revealed preemption attempts                                     | Helps prevent unstable re-goaling                                                                                                                                                                                   |
| `goal_preemption_skip_if_within_m`                   | `double` | `0.75`  | Skip frontier-revealed preemption if the robot is already close to the active target           | Does not disable blocked-goal checks                                                                                                                                                                                |
| `goal_preemption_complete_if_within_m`               | `double` | `0.0`   | When frontier-revealed preemption is enabled, treat a near-arrived active frontier as complete | `0.0` disables this shortcut; applies to both revealed-preemption paths and is most useful for softening visible-gain behavior near arrival, but large values can cause premature and wrong exploration transitions |
| `goal_preemption_visible_gain_gate_enabled`          | `bool`   | `false` | Enables target-pose visible frontier gain gating on the revealed-preemption path               | When disabled, revealed preemption stays point/snapshot-based                                                                                                                                                       |
| `goal_preemption_visible_gain_range_m`               | `double` | `12.0`  | Range used by the target-pose visible frontier gain estimate                                   | Sensor-model parameter for the map-based ray-cast                                                                                                                                                                   |
| `goal_preemption_visible_gain_fov_deg`               | `double` | `360.0` | Field of view used by the target-pose visible frontier gain estimate                           | Use values below `360` for directional sensors                                                                                                                                                                      |
| `goal_preemption_visible_gain_ray_step_deg`          | `double` | `1.0`   | Angular sampling step used by the target-pose visible frontier gain estimate                   | Smaller steps cost more CPU but resolve narrow structure better                                                                                                                                                     |
| `goal_preemption_visible_gain_min_frontier_length_m` | `double` | `0.5`   | Minimum visible frontier length required to keep the active goal in gain-gated mode            | Below this threshold, the flow falls back to the refreshed frontier snapshot decision                                                                                                                               |
| `goal_preemption_visible_gain_yaw_offset_deg`        | `double` | `0.0`   | Additional yaw offset applied to the target-pose visible gain sensor heading                   | Useful when the effective sensing direction differs from the goal heading model                                                                                                                                     |
| `escape_enabled`                                     | `bool`   | `true`  | Enables escape mode until the first successful frontier goal                                   | Allows a farthest-frontier fallback before normal preferred selection becomes available                                                                                                                             |
| `post_goal_min_settle`                               | `double` | `0.80`  | Minimum time to wait after a successful frontier goal                                          | Part of settle readiness                                                                                                                                                                                            |
| `post_goal_required_map_updates`                     | `int`    | `3`     | Required number of update events before sending the next frontier goal                         | Internally clamped to at least `1`                                                                                                                                                                                  |
| `post_goal_stable_updates`                           | `int`    | `2`     | Required number of stable signature observations before continuing                             | Internally clamped to at least `1` and never above the required update count                                                                                                                                        |
| `return_to_start_on_complete`                        | `bool`   | `true`  | Returns to the recorded start pose after frontier exhaustion                                   | Sends a regular navigation goal back to the saved start pose                                                                                                                                                        |
| `all_frontiers_suppressed_behavior`                  | `string` | `stay`  | Behavior used when frontiers exist but all detected candidates are temporarily suppressed      | Supported values: `stay`, `return_to_start`; other values are normalized to `stay`                                                                                                                                  |

### Frontier Suppression

| Parameter                                     | Type     | Default | Description                                                                  | Notes                                                                                                         |
| --------------------------------------------- | -------- | ------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `frontier_suppression_enabled`                | `bool`   | `false` | Enables temporary frontier suppression                                       | When disabled, suppression state is not allocated                                                             |
| `frontier_suppression_attempt_threshold`      | `int`    | `3`     | Failed attempts required before a frontier area is suppressed                | Applied to quantized goal-point attempt records                                                               |
| `frontier_suppression_base_size_m`            | `double` | `1.0`   | Initial side length of a new square suppression region                       | Used when a matured failed attempt is first promoted into a region                                            |
| `frontier_suppression_expansion_size_m`       | `double` | `0.5`   | Outer ring width used to detect nearby repeated failures and grow a region   | A matured failure in this band doubles the current square size                                                |
| `frontier_suppression_timeout_s`              | `double` | `90.0`  | Lifetime of suppression records in seconds                                   | Shared TTL for both attempt records and active suppression regions                                            |
| `frontier_suppression_no_progress_timeout_s`  | `double` | `20.0`  | Maximum allowed time without meaningful progress for an active frontier goal | If exceeded, the goal is canceled and counted as a failed attempt                                             |
| `frontier_suppression_progress_epsilon_m`     | `double` | `0.05`  | Minimum `distance_remaining` improvement required to count as progress       | There is no separate epsilon time window; this improvement must happen before the no-progress timeout expires |
| `frontier_suppression_startup_grace_period_s` | `double` | `15.0`  | Startup delay before suppression becomes active                              | During grace, suppression filtering, failed-attempt recording, and no-progress timeout are disabled           |
| `frontier_suppression_max_attempt_records`    | `int`    | `256`   | Maximum number of live failed-attempt records kept in memory                 | Hard cap for bounded-memory attempt tracking                                                                  |
| `frontier_suppression_max_regions`            | `int`    | `64`    | Maximum number of active suppression regions kept in memory                  | Hard cap for bounded-memory region tracking                                                                   |

### Integration Hooks

| Parameter                  | Type     | Default                | Description                         | Notes                                                      |
| -------------------------- | -------- | ---------------------- | ----------------------------------- | ---------------------------------------------------------- |
| `completion_event_enabled` | `bool`   | `false`                | Enables completion-event publishing | Publishes once per node lifetime after frontier exhaustion |
| `completion_event_topic`   | `string` | `exploration_complete` | Topic used for the completion event | Must be non-empty if completion events are enabled         |

Notes:

- `No more frontiers found` and `All currently detected frontiers are temporarily suppressed` describe different states.
- `return_to_start_on_complete` only applies after frontier exhaustion.
- `all_frontiers_suppressed_behavior=return_to_start` only applies to the temporary suppressed-frontier state.
- a temporary suppressed return-to-start does not publish a completion event and does not mean mission completion.
- revealed preemption can run either as a plain refreshed-frontier membership check or as a visible-gain-gated check before that same snapshot decision.

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## TurtleBot3 Waffle Pi Example

The following example is a clean TurtleBot3 Waffle Pi exploration profile derived from a real Nav2 integration. It enables both revealed-preemption paths: the normal refreshed-frontier check and the target-pose visible-gain gate.

### Example Parameter File

```yaml
frontier_explorer:
  ros__parameters:
    map_topic: /map
    costmap_topic: /global_costmap/costmap
    local_costmap_topic: /local_costmap/costmap
    navigate_to_pose_action_name: navigate_to_pose

    global_frame: map
    robot_base_frame: base_footprint

    frontier_marker_topic: /explore/frontiers
    frontier_marker_scale: 0.15

    map_qos_durability: transient_local
    map_qos_reliability: reliable
    map_qos_depth: 1
    map_qos_autodetect_on_startup: false
    map_qos_autodetect_timeout_s: 2.0

    costmap_qos_reliability: reliable
    costmap_qos_depth: 10
    local_costmap_qos_reliability: inherit
    local_costmap_qos_depth: -1

    frontier_min_distance: 0.5
    frontier_visit_tolerance: 0.30
    goal_preemption_on_frontier_revealed: true
    goal_preemption_on_blocked_goal: true
    goal_preemption_min_interval_s: 2.0
    goal_preemption_skip_if_within_m: 0.75
    goal_preemption_complete_if_within_m: 0.25
    goal_preemption_visible_gain_gate_enabled: true
    goal_preemption_visible_gain_range_m: 12.0
    goal_preemption_visible_gain_fov_deg: 360.0
    goal_preemption_visible_gain_ray_step_deg: 1.0
    goal_preemption_visible_gain_min_frontier_length_m: 0.5
    goal_preemption_visible_gain_yaw_offset_deg: 0.0
    escape_enabled: true
    post_goal_min_settle: 0.80
    post_goal_required_map_updates: 3
    post_goal_stable_updates: 2
    return_to_start_on_complete: true
    all_frontiers_suppressed_behavior: return_to_start

    frontier_suppression_enabled: false
    frontier_suppression_attempt_threshold: 1
    frontier_suppression_base_size_m: 1.0
    frontier_suppression_expansion_size_m: 0.5
    frontier_suppression_timeout_s: 90.0
    frontier_suppression_no_progress_timeout_s: 20.0
    frontier_suppression_progress_epsilon_m: 0.05
    frontier_suppression_startup_grace_period_s: 15.0
    frontier_suppression_max_attempt_records: 256
    frontier_suppression_max_regions: 64

    completion_event_enabled: true
    completion_event_topic: exploration_complete
```

### Example Launch

Assuming TurtleBot3, SLAM, and Nav2 are already running:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  params_file:=/absolute/path/to/tb3_waffle_pi_frontier.yaml \
  use_sim_time:=true
```

For a namespaced robot:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  namespace:=tb3 \
  params_file:=/absolute/path/to/tb3_waffle_pi_frontier.yaml \
  use_sim_time:=true
```

### What To Verify During Bring-Up

- `/map` is available and uses the expected QoS profile
- `/global_costmap/costmap` is available
- `/local_costmap/costmap` is available
- `map -> base_footprint` exists in TF
- `navigate_to_pose` is reachable
- the LiDAR scan quality is stable enough for SLAM and costmap updates
- the robot can receive and complete Nav2 goals before exploration is started
- the completion event topic is subscribed by an external consumer if post-processing is needed

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Testing

Run package tests with:

```bash
cd <your_workspace>
source /opt/ros/jazzy/setup.bash
colcon test --packages-select frontier_exploration_ros2
colcon test-result --verbose
```

Current test coverage includes:

- frontier extraction behavior
- snapshot cache behavior
- deterministic frontier results
- marker publish deduplication
- preemption and cancelation flow
- frontier suppression, no-progress timeout, startup grace, and temporary return-to-start behavior
- QoS parsing and startup autodetect behavior

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Contributing

This package is currently maintained by a single developer. Contributions are welcome and useful, especially in areas that improve portability, performance, test coverage, documentation quality, and integration breadth.

### What Helps Most

- bug reports with reproducible logs and parameter files
- integration reports from different robots, maps, and Nav2 setups
- performance measurements with clear test conditions
- tests for edge cases in frontier extraction, QoS handling, and preemption flow
- documentation improvements that make the package easier to adopt

### Before Opening an Issue

Please include:

- ROS 2 distribution
- operating system version
- robot model or simulation stack
- LiDAR model
- scan topic name
- LiDAR configured min/max range
- approximate practical usable range in your environment if it differs from the configured range
- scan rate or angular resolution if you changed the default sensor setup
- any scan filtering, cropping, downsampling, or obstacle-layer preprocessing applied before navigation
- relevant topic names
- TF frame names
- QoS settings if they differ from defaults
- the parameter file you used
- logs that show the problem

If the issue is QoS-related, include the output of:

```bash
ros2 topic info -v /map
ros2 topic info -v /global_costmap/costmap
ros2 topic info -v /local_costmap/costmap
```

### Before Opening a Pull Request

Please make sure the change:

- is focused and reviewable
- keeps the package universal and free of project-specific assumptions
- includes tests when behavior changes
- updates the README when the public integration surface changes
- keeps parameter names and examples aligned with the code

### Suggested Contribution Workflow

```bash
git checkout -b feature/my-change
source /opt/ros/jazzy/setup.bash
colcon build --packages-select frontier_exploration_ros2
colcon test --packages-select frontier_exploration_ros2
```

Then open a pull request with:

- a short problem statement
- the technical approach
- any parameter or QoS changes
- test evidence
- migration notes if existing users must update configs

### Pull Request Checklist

- [ ] code builds on ROS 2 Jazzy
- [ ] tests pass
- [ ] parameter defaults remain intentional
- [ ] public docs are updated when needed
- [ ] no project-specific references were introduced
- [ ] QoS and integration behavior are clearly documented

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## License

This project is released under the Apache-2.0 License. See [LICENSE](https://github.com/mertgulerx/frontier-exploration-ros2/blob/main/LICENSE).

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>

## Maintainer

Maintainer: `mertgulerx`  
Support Email: `support.mertgulerx@gmail.com`

<p align="right"><a href="#frontier_exploration_ros2">back to top</a></p>
