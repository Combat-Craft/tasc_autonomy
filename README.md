# artemis branch

## DATE: 2025/03/31 Tuesday 

Upon reading my notes below and comparing packages, it looks I succeeded before because I removed many depreciated/troublesome packages. These are:
  - asimov_description
    - this thing got replaced by giskard_description anyways
  - rviz
    - also out of date
  - package.xml
    - removing it, as in main, apparently fixed many issues
   
Fixing some other packages from main branch:
  - giskard_gazebo
    - edited giskard_gazebo/CMakeLists.txt, line 13 removed the non-existing folders like ```include``` and ```src```
   
Regardless, the mix of CMAke and Python ROS2 packages is mildly infuriating. I will keep giskard_description and giskar_gazbo, and remake a new autonomy_sensors pacge with CMake.

The other packages can go *bleep*, but I will make empty packages with their names and fill them with "working" code as we get to it.

## DATE: 2025/03/30 Monday 

During work session the *bleep* package would not build. My current task is now to ensur emy branch entiew thing can build, with instructions onto how.

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
