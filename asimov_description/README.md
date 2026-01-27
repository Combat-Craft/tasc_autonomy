# NOTES
I placed  **ros2_control.xacro** here as it is a xacro file, but **ros2controls_controllers.yaml** most likely belongs somewhere else; However, not sure where else to place for now.

<br>

### 27/01/2027 Notes:

 * Seperated URDF into sperate XACRO files for each sensor, leaving out ASIMOV as it looks we are moving away.
 * All were missing collision, added them by copying the geometry from visual, except for complicated ones like Orbbec.
 * All also missing <inertial> within the <link>, but not sure if they're mandatory?
 * No RGB camera found?

