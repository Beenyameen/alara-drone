# indoor_drone_sim

A minimal ROS 2 Jazzy indoor drone simulator (2.5D: planar motion + altitude), intended for quick validation of:

- manual control via `/cmd_vel`
- odometry + TF (`/odom` and `odom -> base_link`)
- depth camera outputs (`/depth/image_raw`, `/depth/image_viz`, `/depth/camera_info`, `/depth/points`)
- radiation sensing (`/geiger_count`)
- world visualization markers (`/world_geometry`)
- live radiation grid mapping (`radmapper_node`)
