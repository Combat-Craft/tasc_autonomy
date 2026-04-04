# artemis branch

## Steps
1) cloned this branch (artemis) into a workspace folder i.e. ```$ mkdir -p ~/YOUR_WORKSPACE_NAME_HERE/src```
2) ```source /opt/ros/humble/setup.bash```
3) ``` colcon build```
4) You can CD and then worked inside ```YOUR_WORKSPACE_NAME_HERE/src/tasc_autonomy ```, but while testing and debugging, I rec you open another terminal, source humble (step 2), then go to  ```YOUR_WORKSPACE_NAME_HERE/src/tasc_autonomy```
5) source workspace ```source install/local_setup.bash```
6) You can ```ros2 launch giskard_description test.launch.py ```to see the model and sensors in both rviz and gazebo 
7) You can test out the imu gps code with ```ros2 launch autonomy_sensors sensors.launch.py```. Simulated data has been given if the ESP32 isn't connected and running, but real/live data needs to be tested!!!!


### GPS IMU node

```ros2 run autonomy_sensors gps_node ``` 
```ros2 run autonomy_sensors imu_node ``` 
```ros2 launch autonomy_sensors sensors.launch.py ``` for both at once

Errors you may face:
  - ```attributeError: module 'serial' has no attribute 'SerialException'```
    -  system has the wrong serial module, so reinstall the right one
    -  ```pip uninstall serial```
    -  ```pip install pyserial```
  -  Check the USB port is correct! Should be USB0, but might change to USB1 or whatever.

NOTE: currently don't have the ESP32 with GPS and IMU with me. So right now I can't test the rest of the code as i reach the error ```RuntimeError: Failed to open serial port /dev/ttyUSB0: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'```.

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

will be following the recomended [structure](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html):

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

Regardless, the mix of CMAke and Python ROS2 packages is mildly infuriating, but ROS2 themselves say its ok... I will keep giskard_description and giskar_gazbo. Since we use Python for our code, including custom nodes, I'll make our new packages in Python rather than cmake


Ultimately, the structure should be as follows in your VM (replace ```YOUR_WORKSPACE_NAME_HERE``` with whatever folder name you use):

```
YOUR_WORKSPACE_NAME_HERE/
    src/
      autonomy_sensors/
          package.xml
          resource/autonomy_sensors
          setup.cfg
          setup.py
          autonomy_sensors/
          test/
          firmware/

      giskard_description/
          CMakeLists.txt
          include/giskard_description/
          package.xml
          src/
          config/
          launch/
          meshes/orbbec_meshes/
          rviz/

      giskard_gazebo/
          CMakeLists.txt
          include/giskard_gazebo/
          package.xml
          src/
          launch/
          models/moon/
          worlds/
```

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



### Extras if we have time
 * could make a seperate materials.xacro then include them to futher modular-ize
 * clean up simple inertials to use only the macros inside inertial.xacro, include them as needed (i.e. not ones ported from CAD files, just boxes and cylinders)
