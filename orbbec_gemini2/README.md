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

if you need any depencies, use rosdep

## Goals

Eventually, going to combine this with autonomy vision. but that's later, since I want to orbbec SDK dev and gstreamer/camera general stuff to work simulataneously withouth merge conflicts mucking around

