# AWS RoboMaker Small Warehouse assets

The files in `models/` are derived from the `ros2` branch of
[`aws-robotics/aws-robomaker-small-warehouse-world`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world),
commit `ee0af733315e78432408c3cd98d378ecee5f767c`.

They are distributed under the included MIT No Attribution (`MIT-0`) license.
The original ground and roof models contained inertia tensors rejected by
modern SDFormat. Their inertial blocks were removed because both models are
static. `../aws_small_warehouse.sdf` adapts the original world for Gazebo Sim 8
and replaces Gazebo Classic's dynamic wrapper models with top-level static
includes.
