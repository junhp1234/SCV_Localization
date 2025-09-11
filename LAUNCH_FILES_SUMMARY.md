# Robot Localization Launch Files Summary

## Available Launch Files

### 1. `map_origin_transform.launch.py`
**Purpose**: Map origin transform node only  
**Usage**: `ros2 launch robot_localization map_origin_transform.launch.py`  
**Description**: Launches only the map origin transform node for broadcasting map -> odom TF transformation based on YAML configuration.

### 2. `ekf_with_map_origin.launch.py` 
**Purpose**: Complete localization with map origin transform  
**Usage**: `ros2 launch robot_localization ekf_with_map_origin.launch.py`  
**Description**: Integrated setup including:
- EKF node (odom frame)
- EKF node (map frame) 
- NavSat transform node
- Map origin transform node

### 3. `ekf_without_map_origin.launch.py`
**Purpose**: Standard robot_localization without map origin transform  
**Usage**: `ros2 launch robot_localization ekf_without_map_origin.launch.py`  
**Description**: Standard robot_localization functionality without map -> odom transform. Includes:
- EKF node (odom frame)
- EKF node (map frame) - optional
- NavSat transform node - optional

**Parameters**:
- `use_map_ekf`: Enable/disable map frame EKF (default: true)
- `use_navsat_transform`: Enable/disable NavSat transform (default: true)

### 4. `basic_ekf.launch.py`
**Purpose**: Minimal EKF setup  
**Usage**: `ros2 launch robot_localization basic_ekf.launch.py`  
**Description**: Minimal setup with just the odom-frame EKF for simple odometry fusion without GPS or map frame integration.

### 5. `robot_localization_complete.launch.py`
**Purpose**: Complete configurable robot localization  
**Usage**: 
```bash
# Default (without map origin transform)
ros2 launch robot_localization robot_localization_complete.launch.py

# With map origin transform enabled
ros2 launch robot_localization robot_localization_complete.launch.py use_map_origin_transform:=true
```

**Description**: Full-featured launch file with all robot_localization components, each configurable via parameters.

**Parameters**:
- `use_odom_ekf`: Enable/disable odom frame EKF (default: true)
- `use_map_ekf`: Enable/disable map frame EKF (default: true)
- `use_navsat_transform`: Enable/disable NavSat transform (default: true)
- `use_map_origin_transform`: Enable/disable map origin transform (default: false)

## Use Case Recommendations

- **Just need map -> odom TF**: Use `map_origin_transform.launch.py`
- **Full localization with map origin**: Use `ekf_with_map_origin.launch.py`
- **Standard localization (no custom map origin)**: Use `ekf_without_map_origin.launch.py`
- **Simple odometry fusion only**: Use `basic_ekf.launch.py`
- **Maximum flexibility**: Use `robot_localization_complete.launch.py` with appropriate parameters

## Common Launch Arguments

All launch files support these common arguments:
- `output`: Output destination for logs (default: "screen")
- `ekf_params_file`: Path to EKF parameters YAML file
- `map_origin_params_file`: Path to map origin parameters YAML file (where applicable)