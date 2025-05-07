# nav_msgs_custom

`nav_msgs_custom` is a standalone ROS 2 interface package that defines custom messages specifically tailored for navigation-related tasks. This package provides reusable and modular message definitions that can be shared across projects, making it easier to integrate navigation data structures across different systems (robots, ground stations, Android apps, etc.).

---

## Current Messages

### `GPSWaypointArray.msg`

This message allows publishing **multiple GPS waypoints** in a single message. Each waypoint includes:

- **Latitude**
- **Longitude**
- **Altitude**
- **Heading**

It uses an array of `std_msgs/Float64MultiArray`, where each waypoint is encoded as:

```yaml
data: [latitude, longitude, altitude, heading]
```

### Example message:

```
waypoints:
  - data: [12.34, 56.78, 100.0, 90.0]
  - data: [23.45, 67.89, 110.0, 180.0]
```

---
## Usage

### 1. Build the package

```
colcon build --packages-select nav_msgs_custom
source install/setup.bash

```

### 2. Echo the message
```
ros2 topic echo /gps_waypoints nav_msgs_custom/msg/GPSWaypointArray
```

### Publish test data

```
ros2 topic pub /gps_waypoints nav_msgs_custom/msg/GPSWaypointArray \
'{waypoints: [{data: [12.34, 56.78, 100.0, 90.0]}, {data: [23.45, 67.89, 110.0, 180.0]}]}'
```