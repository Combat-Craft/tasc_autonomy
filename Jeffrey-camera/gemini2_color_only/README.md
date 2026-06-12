# Gemini 2 Color-Only Test

C++ test for the Orbbec Gemini 2 using OrbbecSDK v2.

It requests only the regular color stream:

```cpp
config->enableVideoStream(OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_MJPG);
```

No depth, IR, IMU, or other streams are enabled.

## Setup

Run the OrbbecSDK v2.8.6 Linux x86_64 setup:

```bash
cd ~/Downloads/OrbbecSDK_v2.8.6_202604271452_6399409_linux_x86_64
chmod +x setup.sh
./setup.sh
```

Install dependencies:

```bash
sudo apt update
sudo apt install libopencv-dev gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
```

## Build

```bash
cd ~/gemini2_color_only
mkdir -p build
cd build
cmake ..
make
```

## Run

```bash
./gemini2_probe
./gemini2_color_only
./gemini2_color_gstreamer
```

- `gemini2_probe`: checks for connected Orbbec devices
- `gemini2_color_only`: starts the color-only stream
- `gemini2_color_gstreamer`: uses OrbbecSDK to get color frames, OpenCV to decode MJPG, and GStreamer to display the video

If no camera is connected, a no-device error is expected.

## VMware

If using VMware, connect the camera to the VM:

```text
VM -> Removable Devices -> Orbbec/Gemini camera -> Connect
```

Then check:

```bash
lsusb
```

## Status

Build-tested without the physical Gemini 2. Real camera runtime testing still needed.
