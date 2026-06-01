# gps_tracker
hi

Subscribes to `/gps/fix` and logs an accumulated path to `/gps/path`.

For real GPS data, use `gps_imu_broadcaster` from `autonomy_sensors` it handles the ESP32 serial connection and publishes `/gps/fix` + `/imu/data_raw`

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select gps_tracker
source install/setup.bash
```

---

## Modes

### fake — hardcoded Toronto waypoints
```bash
ros2 launch gps_tracker gps_tracker.launch.py mode:=fake
```

### ip — your approximate location via IP lookup
```bash
ros2 launch gps_tracker gps_tracker.launch.py mode:=ip
```

### real — live ESP32 GPS via gps_imu_broadcaster
Run `gps_imu_broadcaster` from `autonomy_sensors` separately, then:
```bash
ros2 launch gps_tracker gps_tracker.launch.py mode:=real port:=/dev/ttyUSB0
```

---

## Options


`mode` |`fake`, `ip`, or `real` 
`loop` |Loop waypoints 
`walk_step_m` Step size in metres (ip mode) 
`foxglove`| Launch Foxglove bridge 
`port`  Serial port for ESP32
`baudrate` | Must match ESP32  

---

## Topics


| `/gps/fix`  Raw GPS fix (input) 
| `/gps/path` Accumulated path in local x/y metres (output) 

---

## Foxglove

Connect to `ws://localhost:8765` then:

- **Map panel** → `/gps/fix` — plots position on a real map
- **3D panel** → `/gps/path`, fixed frame `map` — shows local x/y trail
- **Raw Messages** → `/gps/fix` — live numbers

```bash
# Disable bridge
ros2 launch gps_tracker gps_tracker.launch.py foxglove:=false
```

---

## Verify

```bash
ros2 topic echo /gps/fix
ros2 topic echo /gps/path
ros2 topic hz /gps/fix
```