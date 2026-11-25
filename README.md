# mola_warehouse_pf_tutorial

Repository for the localization-only demo using MOLA-built maps and particle filtering,
as described in: https://docs.mola-slam.org/latest/localization.html

## Regenerating the map

The map in `maps/` must match the current `demo_warehouse.world.xml` of
[MVSim](https://github.com/MRPT/mvsim). Its frame is the robot initial pose in
that world, so the particle filter can start from its default initial pose.
To rebuild it after changes to the world:

1. Record a dataset while driving the robot around the warehouse (e.g. with a
   joystick or the keyboard in the MVSim GUI):

   ```bash
   MVSIM_SAVE_RAWLOG_FILENAME=/tmp/warehouse.rawlog \
     mvsim launch $(ros2 pkg prefix mvsim)/share/mvsim/mvsim_tutorial/demo_warehouse.world.xml
   ```

2. Build a simple map with MOLA LiDAR odometry:

   ```bash
   MOLA_LIDAR_NAME=lidar1 mola-lo-cli-rawlog /tmp/warehouse.r1.rawlog \
     --output-simplemap /tmp/warehouse.simplemap
   ```

3. Convert it into the metric map used for localization:

   ```bash
   sm2mm -i /tmp/warehouse.simplemap -o maps/mvsim-warehouse01.mm \
     -p sm2mm-config/sm2mm_pipeline.yaml
   ```
