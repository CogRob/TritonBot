Before running this, first stop the pure-localization mode cartographer.

```
source ~/catkin_ws/devel/setup.bash
rosservice call /finish_trajectory "trajectory_id: 0"
rosservice call /write_state "filename: 'maps/[DATA-TIME].cart'"
rosrun map_server map_saver map:=map_cart -f /root/.ros/maps/[DATE-TIME]
```
