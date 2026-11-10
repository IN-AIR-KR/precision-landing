# pl_viz

Common demand-driven camera/control HUD: `ros2 run pl_viz monitor`.
See [stage 4](../../docs/monitoring_stage4.md) for topics, configuration and tests.

The optional Gazebo marker executable is built only with `-DPL_VIZ_GAZEBO=ON`.
`scripts/build.sh sim` enables it; `scripts/build.sh hardware` disables it.
