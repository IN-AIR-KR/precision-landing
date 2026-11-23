# pl_viz

Common demand-driven camera/control HUD: `ros2 run pl_viz monitor`.
See the [node structure](../../README.md#3-노드-구조) for topics and the [simulation profile](../pl_bringup/config/sim.yaml) for HUD settings.

The optional Gazebo marker node runs with `ros2 run pl_viz tag_pose_visualizer`.
On the simulation host, install `python3-gz-transport13` and `python3-gz-msgs10` from the Gazebo apt repository. Use `enable_gazebo_viz:=false` to disable it in the simulation launch.
