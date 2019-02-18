# Watch Tower mini-WAMV Tracking

### - Please follow the following steps to run this node

1. Run watch tower tracking code

```
$ rosrun lidar_perception watch_tower
```
2. Play rosbag of point cloud data

3. Press 'SPACE' to pause the rosbag

4. Open Rviz and select following topics to visualize:
- /boundary_marker_point
- /boundary_marker
- /path_a (please select path color for robot A)
- /path_b (try to select different color from robot A)
- /wamvs
- /bamboobotb/velodyne_points
- Axes

5. Press "2D Pose Estimate" button and select the region that the boat will navigate in

6. Press 'SPACE' to play the rosbag

7. If you only see one robot path instead of two, please call rosservice "/clear_path" to reset the path

8. Once you run new rosbag without restartig the tracking node, please call rosservice "/clear_path" to reset the path