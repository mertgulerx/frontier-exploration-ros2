---
name: Bug report
about: Create a report to help us improve
title: ''
labels: ''
assignees: ''

---

## Summary

Provide a short and clear description of the issue.

## Expected Behavior

Describe what you expected to happen.

## Actual Behavior

Describe what actually happened.

## Reproduction Steps

1. Describe the initial setup.
2. List the command(s) you ran.
3. Explain when the problem appears.
4. Include whether the issue is consistent or intermittent.

## Environment

- ROS 2 distribution:
- Operating system:
- Robot platform or simulator:
- Navigation stack:
- Mapping/localization stack:
- Sensor setup:
- `use_sim_time`:
- Workspace / package branch or commit:

## Interfaces and Configuration

- Map topic:
- Global costmap topic:
- Local costmap topic:
- Scan topic:
- Navigation action name:
- Global frame:
- Robot base frame:
- Completion event topic:
- Namespacing used:
- Relevant QoS settings:
- Relevant parameter file:

## Relevant Parameters

Paste only the parameters that are likely related to the issue.

```yaml
frontier_explorer:
  ros__parameters:
    map_topic: /map
    costmap_topic: /global_costmap/costmap
    local_costmap_topic: /local_costmap/costmap
    navigate_to_pose_action_name: navigate_to_pose
    global_frame: map
    robot_base_frame: base_footprint
    goal_preemption_on_frontier_revealed: true
    goal_preemption_on_blocked_goal: true
```

## Logs and Evidence

Paste relevant logs, warnings, stack traces, screenshots, or RViz observations here.

```text
Paste logs here
```

If the issue may be QoS-related, include:

```bash
ros2 topic info -v /map
ros2 topic info -v /global_costmap/costmap
ros2 topic info -v /local_costmap/costmap
```

## What You Already Tried

- [ ] Verified required topics are available
- [ ] Verified TF frames exist
- [ ] Verified Nav2 action server is reachable
- [ ] Checked QoS compatibility
- [ ] Tested with the packaged parameter file
- [ ] Tested with simplified / default settings
- [ ] Reviewed logs for startup autodetect, suppression, or preemption behavior

## Impact

Describe how this issue affects usage.

- [ ] Blocks package usage completely
- [ ] Causes incorrect exploration behavior
- [ ] Causes unstable or inconsistent behavior
- [ ] Documentation is unclear or incomplete
- [ ] Minor issue / improvement request

## Additional Context

Add any other context that may help diagnose the issue.
