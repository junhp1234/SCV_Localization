# Map Origin Transform Node

## Overview

The `map_origin_transform_node` is a new component of the robot_localization package that provides map -> odom coordinate frame transformation based on YAML configuration. This implementation is similar to the approach used in the Global_path_planner, allowing for easy configuration of map origin parameters through YAML files.

## Features

- **YAML-based Configuration**: All map origin parameters are configurable via YAML files
- **Static and Dynamic Transforms**: Supports both static (one-time) and dynamic (periodic) transform broadcasting
- **UTM Integration**: Handles GPS to UTM coordinate conversions
- **Flexible Frame Assignment**: Configurable frame IDs for map and odom frames
- **Transform Offsets**: Optional translation and rotation offsets

## Configuration

### Parameters

The node reads configuration from a YAML file with the following structure:

```yaml
map_origin_transform_node:
  ros__parameters:
    # Map origin GPS coordinates (decimal degrees)
    map_origin:
      gps:
        latitude: 37.5665    # Example: Seoul, Korea
        longitude: 126.9780
        altitude: 0.0
      
      # Map origin UTM coordinates
      utm:
        easting: 323000.0    # UTM easting in meters
        northing: 4157000.0  # UTM northing in meters
        zone: "52N"          # UTM zone
    
    # Transform broadcasting settings
    broadcast_transform: true    # Enable/disable transform broadcasting
    static_transform: true       # Static (true) or dynamic (false) transform
    frequency: 10.0             # Update frequency for dynamic transform (Hz)
    
    # Frame IDs
    map_frame: "map"
    odom_frame: "odom"
    
    # Optional transform offsets
    transform_offset:
      translation:
        x: 0.0
        y: 0.0
        z: 0.0
      rotation:
        roll: 0.0
        pitch: 0.0
        yaw: 0.0
```

### Key Parameters

- **map_origin.gps**: GPS coordinates of the map origin point
- **map_origin.utm**: Corresponding UTM coordinates of the map origin
- **broadcast_transform**: Whether to broadcast the transform
- **static_transform**: If true, broadcasts static transform once; if false, broadcasts at specified frequency
- **transform_offset**: Optional offset adjustments for fine-tuning

## Usage

### Available Launch Files

Several launch files are provided for different use cases:

#### 1. Map Origin Transform Only
```bash
ros2 launch robot_localization map_origin_transform.launch.py
```
Launches only the map origin transform node for broadcasting map -> odom TF.

#### 2. EKF with Map Origin Transform (Integrated)
```bash
ros2 launch robot_localization ekf_with_map_origin.launch.py
```
Complete localization setup including EKF nodes, NavSat transform, and map origin transform.

#### 3. EKF without Map Origin Transform
```bash
ros2 launch robot_localization ekf_without_map_origin.launch.py
```
Standard robot_localization functionality without map -> odom transform.

#### 4. Basic EKF Only
```bash
ros2 launch robot_localization basic_ekf.launch.py
```
Minimal setup with just the odom-frame EKF for simple odometry fusion.

#### 5. Complete Robot Localization (Configurable)
```bash
# Without map origin transform (default)
ros2 launch robot_localization robot_localization_complete.launch.py

# With map origin transform enabled
ros2 launch robot_localization robot_localization_complete.launch.py use_map_origin_transform:=true
```
Full-featured launch file with all options configurable via parameters.

### Custom Parameters

Launch with custom parameter files:

```bash
ros2 launch robot_localization map_origin_transform.launch.py params_file:=/path/to/your/params.yaml
ros2 launch robot_localization ekf_with_map_origin.launch.py ekf_params_file:=/path/to/ekf.yaml map_origin_params_file:=/path/to/map_origin.yaml
```

## Implementation Details

### Transform Calculation

The node calculates the map -> odom transform by:

1. Reading map origin coordinates from YAML configuration
2. Computing the translation as the negative of UTM coordinates (to center odom at map origin)
3. Applying any configured offsets
4. Broadcasting the transform using tf2

### Coordinate Frame Relationship

```
map (UTM-based global frame)
  └── odom (local odometry frame, centered at map origin)
      └── base_link (robot body frame)
```

## Files Created/Modified

### New Files
- `include/robot_localization/map_origin_transform.hpp` - Header file
- `src/map_origin_transform.cpp` - Implementation
- `src/map_origin_transform_node.cpp` - Node executable
- `params/map_origin_transform.yaml` - Default configuration
- `launch/map_origin_transform.launch.py` - Map origin transform only
- `launch/ekf_with_map_origin.launch.py` - Integrated with map origin transform
- `launch/ekf_without_map_origin.launch.py` - Standard EKF without map origin transform
- `launch/basic_ekf.launch.py` - Minimal EKF setup
- `launch/robot_localization_complete.launch.py` - Complete configurable launch file

### Modified Files
- `CMakeLists.txt` - Added new node to build system

## Comparison with Global_path_planner

This implementation provides similar functionality to the Global_path_planner's TF system but with key improvements:

1. **Configuration-driven**: All parameters come from YAML instead of hardcoded values
2. **Standalone node**: Can be used independently or integrated with existing localization
3. **Flexible operation**: Supports both static and dynamic transform broadcasting
4. **Standard integration**: Uses robot_localization package conventions

## Testing

After building the package, you can test the node:

```bash
# Source the workspace
source /home/ros2/nav2_ws/install/setup.bash

# Launch the node
ros2 launch robot_localization map_origin_transform.launch.py

# Check transforms
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo map odom
```

## Troubleshooting

1. **Build errors**: Ensure all dependencies are installed and the workspace is properly sourced

2. **Transform not appearing**: 
   - Check that `broadcast_transform` is set to true in the map origin YAML file
   - For EKF nodes, verify `publish_tf` parameter is correctly set

3. **Missing odom -> base_link transform**:
   - Ensure the odom frame EKF node has `publish_tf: true` in its configuration
   - The map frame EKF node should have `publish_tf: false` to avoid conflicts

4. **Incorrect coordinates**: Verify that UTM and GPS coordinates correspond to the same location

5. **Frame conflicts**: Ensure no other nodes are broadcasting conflicting transforms:
   - Only one node should publish map -> odom transform
   - Only one node should publish odom -> base_link transform

6. **EKF Configuration Issues**:
   - Check that `world_frame` parameter matches the intended coordinate frame
   - Odom frame EKF: `world_frame: odom`, `publish_tf: true`
   - Map frame EKF: `world_frame: map`, `publish_tf: false`

### Common Configuration Patterns

**For complete localization with map origin transform:**
```yaml
ekf_filter_node_odom:
  ros__parameters:
    world_frame: odom
    publish_tf: true    # Publishes odom -> base_link

ekf_filter_node_map:
  ros__parameters:
    world_frame: map
    publish_tf: false   # Avoids conflict, map -> odom handled by map_origin_transform_node

map_origin_transform_node:
  ros__parameters:
    broadcast_transform: true  # Publishes map -> odom
```