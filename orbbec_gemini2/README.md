# Notes on this combined package

## Package Structure

```
orbbec_gemini2/
# --> package info, configuration, and compilation
├── CMakeLists.txt
├── package.xml
# --> Python stuff
├── orbbec_gemini2
│   ├── __init__.py
│   └── module_to_import.py
├── scripts
│   └── py_node.py  # python node code goes here!
# --> Cpp stuff
├── include
│   └── orbbec_gemini2
│       └── header_files_from_src_here.hpp
└── src
    └── cpp_node.cpp
```

## Running and Build

Go cd to your main workspace folder, it holds src, build, install, etc

```bash
cd ~autonav_ws/
```

Colcon build

```bash
colcon build --packages-select orbbec_gemini2
```

To run the Orbbec Code, which is external to ROS2 for now...

## Build

```bash
cd build
cmake ..
make
```

## Run

```bash
./gemini2_color_only
./gemini2_color_gstreamer
```

- `gemini2_color_only`: starts the color-only stream
- `gemini2_color_gstreamer`: uses OrbbecSDK to get color frames, OpenCV to decode MJPG, and GStreamer to display the video

If no camera is connected, a no-device error is expected.

## Goals

Clean up orbbec code to actually use gstreamer and remove any headers from orbbec that aint required

### FROM ORBBEC: The utils functions and classes used in the examples code

**_utils_**

It contains functions to wait for a key press with an optional timeout, get the current time in milliseconds, and parse the input option from a key press event.This is done with C++.

**_utils_c_**

It contains functions to get the current system timestamp and wait for keystrokes from the user, as well as a macro to check for and handle errors. These capabilities can be used in scenarios such as time measurement, user interaction, and error handling. This is done with C.

**_utils_opencv_**

The CVWindow class includes the following main functionalities:

The CVWindow class leverages OpenCV to create a flexible and customizable graphical interface for displaying and managing camera frames.

_Window Creation:_

- The constructor accepts the window name, width, and height as parameters, along with an optional - arrangement mode (ArrangeMode).
- The arrangement modes include single frame display, displaying multiple frames in a row, displaying multiple frames in a column, grid display, and overlay display.

  _Image Frame Display and Processing:_

- The pushFramesToView method is used to push a set or multiple sets of image frames to the window.
- There is an internal thread processThread\_ for processing image frames.
- The arrangeFrames method arranges the image frames based on the selected arrangement mode.
- The visualize and drawInfo methods are used to draw additional information on the images.

  _User Interaction:_

- The setKeyPressedCallback method sets a key press callback function.
- The setKeyPrompt method sets a prompt message.
- The addLog method adds a log message.

  _Configuration and State Management:_

- The setShowInfo method controls whether frame information should be displayed.
- The setAlpha set alpha for OVERLAY render mode
- The resize method adjusts the window size.
- The close method closes the window.
- The reset method clears the cached frames and image matrices.

### The Error Handling in the examples code

If an error occurs, the SDK reports the error by throwing an exception of type ob::Error. The ob::Error exception class typically contains detailed information about the error, which can help developers diagnose the problem.
The example uses a 'try' block to wrap the entire main function. If an exception of type ob::Error is thrown, the program will catch it and print the error message to the console.
Here is the information that can be obtained from an ob::Error:

**Function Name (getFunction()):**
Indicates the name of the function where the exception was thrown. This helps pinpoint the location of the error within the code.

**Arguments (getArgs()):**
Provides information about the arguments passed to the function when the exception occurred. This context can be useful for understanding the specific conditions under which the error happened.

**Error Message (what()):**
Returns a string describing the nature of the error. This is often the most important piece of information, as it directly explains what went wrong.

**Exception Type (getExceptionType()):**
Specifies the type of the exception. This can help categorize the error and determine appropriate handling strategies. Read the comments of the OBExceptionType enum in the libobsensor/h/ObTypes.h file for more information.

**Example Code** in C++

```cpp
catch(ob::Error &e) {
    std::cerr << "function: " << e.getFunction() << std::endl;
    std::cerr << "args: " << e.getArgs() << std::endl;
    std::cerr << "message: " << e.what() << std::endl;
    std::cerr << "type: " << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
```

Eventually, going to combine this with autonomy vision. but that's later, since I want to orbbec SDK dev and gstreamer/camera general stuff to work simulataneously withouth merge conflicts mucking around
