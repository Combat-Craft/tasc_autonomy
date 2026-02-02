# artemis branch

## DATE: 2025/02/02 Monday 

I cloned the entire main branch into a workspace, then worked inside ``` ~/tasc_autonomy_ws/src/tasc_autonomy ```

Overall:
 * I made a new launch file test.launch.py, as the original was not working.
 * seperated the one giskard_base.urdf into several xacro files with robot.urdf.xacro being the one that combines them all
   * i edited some dimensions for the sensors, but ultimately left most of it alone
   * I added the orbbec, and used the depth_camera gazbo plugin that was in the original giskard_base.urdf
 * replaced rviz file with test.rviz

### Steps
1) cloned the entire main branch into a workspace i.e. ```$ mkdir -p ~/YOUR_WORKSPACE_NAME_HERE/src```
2) cd and then worked inside ```$  ~/YOUR_WORKSPACE_NAME_HERE/src/tasc_autonomy ```
3) Replace the main branch's ``` giskard_description ``` folder with this one's - ONLY THE FOLDER/PACKAGE giskard_description; ignore the others inside artemis branch.
4) Source, build, source workspace. You can then launch test.launch.py to see it in both rviz and gazebo
```
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/local_setup.bash
$ ros2 launch giskard_description test.launch.py
```

### Extras if we have time
 * could make a seperate materials.xacro then include them to futher modular-ize
 * clean up simple inertials to use only the macros inside inertial.xacro, include them as needed (i.e. not ones ported from CAD files, just boxes and cylinders)
