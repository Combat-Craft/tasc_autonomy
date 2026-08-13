# Overall notes
Frankly, we're the only subteam that didn't have to do any debugging or repairs. 
Everyone did incredible well to ensure their code worked both on their PC's and on the Jetson with close to real situations - thank you all for coming in for longs hours and often suddenly for testing.


# Cameras and GUI
### What Worked
- consistently good FPS and quality
- the SRT streams immediately picked up whenever connection got lost due to line-of-sight to antenna was lost
- the HowToRun.md document made it easy to start the Jetson side and the Client side GUI
- Foxglove triggers for panorama, top camera panning, and morse code made them easy to use in the field
  - **would like a button** even more, so can be improved! 
- panorama code worked excellently.
  - simple but organized folder structure made it easy to download and create the panoramas
- morse code was not used dueing heist task, but during some quick testing
  - using IP camera meant did not have to run via ROS2 on Jetson i.e. less CPU/RAM usage
  - can use OpenCV window to see what the code saw
- GPS logger works very well, made the file of points and was able to plot
- top camera helps a lot to give a 'wider' and long range POV of the rover and where it needed to go
- IP cameras in general were nice in that they avoided using resources of the Jetson, not requiring software encoding and having built in onvif/networking
 

### Issues
1. IP cameras did not auto restart as RTSP, had to restart GUI
   - we forgot to add a button to manually kill and restart any Gstreamer pipelines on the GUI end.
   - Required for non-SRT, which has the keep-listening option to ensure they restart automatically
2. GUI often crashed,
   - for one error crash code, Mina suspects it might be because my laptop has only 8GB of RAM -> we need to test with another person's laptop
   - another error was segmentation fault, which means one of our arrays/dictionaries is probably not working cleanly.
3. the docks are horrendous to move around for my laptop
   - Mina never suffered, might be a low RAM or even a Windows vs Ubuntu thing
4. weird issue for the front camera dock only: can never resize the dock smaller after it goes big then small
5. lack of screen space. Even if we had integrated all of Foxglove into the GUI, there would not have been enough space to see everything we needed at once
   - genuinely need a operator monitor setup or more laptops/users to handle all the GUI we have
6. GPS logger worked... but I forgot to that i needed to plot the points on a map. Had to scramble and google a site to plot points
   - **next time, read task carefully and plan out each step required**. I forgot in the name of panorama i realize, but that was careless
   - sites like these exist, but each has their own formatting requirements. I had to convert our txt to CSV
     - https://www.mapcustomizer.com/# https://www.batchgeo.com/ https://www.gpsvisualizer.com/map_input?form=data
   - can also make code to do this, but ahsjkdhska.
   - **foxglove wasn't showing history of plot point like i asked it too, however**, so may need to fix or figure out why
7. As I already noted, the panorama/top camera's attachment to the servo is extremely weak.
   - Should have used a 2:3 gear set to offset the camera to rotate on a much more strong and stable axle, and give us 360deg
   - prevented us from using it to look around in refreshment delivery, as I was afraid it would still pop off with the 'roof' so i taped it down
8. In snack run, the LED signs were flashing bc our camera FPS was too low.
   - get some better cameras / use the max FPS available.
     - Orbbec has 60FPS
     - ARM probably needs 60FPS as well
     - THIS NEEDS TESTING TO SEE HOW HIGH FPS WE NEED TO PROPERLY SEE LED SCREENS
9. WEATHER PROOFING DEAR ****
   - the risk of heavy rain caused me to panic.
     - our cameras are not weather proof, and all areas where cables enter the camera body are points where water can drip inside
     - our sensors are not weather proof, but were easier to do since I could fully enclose them with cling wrap.
   - we should try to design weatherproofing into all of our cameras and sensor mounts, or buy those that are rated weatherproofed by default
10. current IP cameras have very thick and rigid cables, which meant they do not work well for panorama movement. actively fought it
    - may want to replace TOP camera with another one, and use current IP cameras for static position OR figure out a better way to move their chunky cables
    - need to be careful when buying cameras as well, see if we can find info on cables. tho, IP cameras with PTZ (i..e pan tilt zoon) built in likely better off
11. We were too slow on implementing ONVIF
    - it is extremely well covered API that can do a lot for us, but we researched and implemented late. Need to do full integration, especially if we plan on adding more.
    - compared to the USB gstreamer cameras, not as well implemented
12. similarily, did not implement certain camera controls such as exposure, brightness, contrast, etc into GUI. or even rotate camera view, zoom, local resize, etc.
    - we have time now for both 11 and 12


# Extra Sensor Ideas
### Arm: Laser Distance Measuring Tool
- another team, i think husky, had a laser measuring tool on their arm to help judge distance. Great idea to help with the arm camera
  - way cheaper than a depth camera that some industries use on their arms
- we can use a time of flight laser sensor
  - more affordable than a true 1d lidar
  - and Laser Measuring Tools, while more rugged and tested don't have wired capabilities (some offer bluetooth tho)
- https://www.youtube.com/watch?v=oVxZ0UAHKXQ  https://www.adafruit.com/product/3317 for implementation ideas

### Actually get Nav2 and SLAM
- this will help the operators 'see' the world around the rover, as the rover sees it
- built in planners and behavoir trees can help operator drive and avoid objects

### Overall, how to more easily determine if a rover can drive over that spot

**can make custom costmaps layer plugin and behaviour trees**
- convert pointcloud into traversability / semantic grid to classify
  - separate ground from above ground
  - for plants/shurbs/other 'soft' objects
    - determine density/occupancy of points (denser likely more 'hard')
    - height above ground (taller trends towards hard)
    - consistency of the object over time (if it shifts alot, likely a plant that is 'soft' but can be hard due to sensor noise)
  - thus, each cell has
    - **max_height_above_ground**,
    - **num_points_in_band[k]** for bands like 0–2 cm, 2–10 cm, 10–30 cm, etc.,
    - **point_density / occupancy per band** (normalize by area/visibility),
    - **vertical structure statistics** e.g., how many distinct height layers exist)
  - apply rule based mapping, tuned to rover and sensors and terrain
    -   If ``max_height < H_soft`` and occupancy in low bands is high but higher bands are low -> treat as soft (allowed)
    -   If ``max_height > H_hard`` > avoid
    -   If occupancy becomes “solid-looking” (dense points in mid/high bands) -> avoid
    -   If the structure is wide but shallow, potentially treat it as soft if suspension/wheels can handle it.

**Better observation of IMU and any encoder/motor feedback**
To help see how well the rover traverse an areas, observe:
- wheel slip increase
- motor current spikes -> **also applies to arm motors!**
- robot deceleration / pitch changes
- relative orientation of the robot (tilted sharply -> more careful driving down/up, needs more power, etc)

**bumper/contact pressure sensors for rover or just ARM**
- can help tell if the rover is directly touching something, and depending on sensitivity how hard it is touching it
- arm:
  - gripper strength?
  - if arm is actually touching a button or the like
  - auger is actually touching ground
- 

 overall for depth camera and LIDAR position, probably best to mount them in the front to prioritize traversability / costmaps


### Cameras
- as seen in RD, front orbbec camera had good quality but due to its position, lack of practice driving with cameras, and generally the different in camera vs human eyes, we could not accurately determe how 'easy' it was to drive certain places
- solution
  - as mentioned above, get Nav2/SLAM in and a custom costmap based on LIDAR and depth camera to help create a map of the world around the over. combined with camera feed, gives drive a better idea of what is actually traversability or not
- get main front camera and top camera calibrated to optimize coverage
  - use largest/wider orbbec color video feed
  - use the larger/wider IP camera feeds as well
- generally, in a car, we see a bit in front of the car towards the ground and forward.
  - depth camera has some limitations as priority is costmaps
  - maybe instead of a temp ground camera for auger, perma add another front camera angled towards the ground, maybe with servo to adjust tilt   

Overall, hard to say how much genuine driving practice with cameras would have helped in operators figureing our traversability. 
But definitely should do it more, as the lack of driving practice was really damn obvious in the field.






