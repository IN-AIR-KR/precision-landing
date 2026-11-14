# pl_viz

Common demand-driven camera/control HUD: `ros2 run pl_viz monitor`.
See [stage 4](../../docs/monitoring_stage4.md) for topics, configuration and tests.

The optional Gazebo marker node runs with `ros2 run pl_viz tag_pose_visualizer`.
On the simulation host, install `python3-gz-transport13` and `python3-gz-msgs10` from the Gazebo apt repository. Use `enable_gazebo_viz:=false` to disable it in the simulation launch.
