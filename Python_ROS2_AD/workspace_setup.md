# CATALYST Workspace Setup Guide

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 20.04/22.04, Windows 10/11 with WSL2, or macOS
- **ROS2 Distribution**: Humble, Iron, or Rolling
- **Python**: 3.8 or later
- **Memory**: 4GB RAM minimum, 8GB recommended
- **Storage**: 5GB free space

### Software Dependencies

#### 1. Install ROS2

**Ubuntu:**
```bash
# Set up sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete
```

**Windows (WSL2):**
```powershell
# Install WSL2 and Ubuntu 20.04
wsl --install -d Ubuntu-20.04
# Then follow Ubuntu instructions above
```

#### 2. Install Python Dependencies
```bash
# Core scientific computing
pip3 install numpy scipy matplotlib

# ROS2 Python packages
pip3 install rclpy

# Additional packages
pip3 install pyyaml shapely
```

#### 3. Install Development Tools
```bash
# ROS2 build tools
sudo apt install python3-colcon-common-extensions

# Development tools
sudo apt install git vim code
```

## Workspace Setup

### 1. Clone and Initialize Workspace

```bash
# Clone the repository
git clone <repository-url> catalyst_ws
cd catalyst_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build the Workspace

```bash
# Build all packages
colcon build

# Build with specific options for development
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Build specific packages only
colcon build --packages-select catalyst_core catalyst_interfaces
```

### 3. Source the Workspace

```bash
# Source the workspace
source install/setup.bash

# Add to bashrc for automatic sourcing
echo "source ~/catalyst_ws/install/setup.bash" >> ~/.bashrc
```

## Configuration Setup

### 1. Default Configuration

The system comes with default configuration in `src/catalyst_core/config/default_config.yaml`. 

### 2. Custom Configuration

Create your own configuration file:

```bash
# Copy default config
cp src/catalyst_core/config/default_config.yaml my_config.yaml

# Edit configuration
nano my_config.yaml
```

Example custom configuration:
```yaml
platform:
  name: "CATALYST_CUSTOM"
  log_level: "DEBUG"

simulation:
  real_time: false
  time_step: 0.05

algorithms:
  default_planner: "astar"
  planning_timeout: 45.0

environment:
  distribution_center:
    length: 400.0  # Custom warehouse size
    width: 250.0

path_planning:
  astar:
    heuristic_weight: 3.0  # More aggressive heuristic
    max_iterations: 15000
```

## Running the System

### 1. Quick Start - Full System

```bash
# Launch complete system with default configuration
ros2 launch catalyst_launch catalyst_system.launch.py

# Launch with custom configuration
ros2 launch catalyst_launch catalyst_system.launch.py \
  config_file:=/path/to/my_config.yaml

# Launch with visualization disabled (headless)
ros2 launch catalyst_launch catalyst_system.launch.py \
  enable_visualization:=false

# Launch with debug logging
ros2 launch catalyst_launch catalyst_system.launch.py \
  log_level:=debug
```

### 2. Individual Components

**Platform Manager (Core):**
```bash
ros2 run catalyst_core platform_manager --ros-args --log-level debug
```

**A* Path Planner:**
```bash
ros2 run catalyst_algorithms astar_plugin
```

**Environment Model:**
```bash
ros2 run catalyst_models environment_model
```

**Visualization:**
```bash
ros2 run catalyst_visualization path_visualizer
```

### 3. Testing Individual Services

**Path Planning Service:**
```bash
# Test path planning service
ros2 service call /catalyst/plan_path catalyst_interfaces/srv/PlanPath "{
  start_state: {x: 10.0, y: 10.0, theta: 0.0},
  goal_state: {x: 100.0, y: 100.0, theta: 0.0},
  obstacle_map: {length_dc: 328.0, width_dc: 200.0}
}"
```

**Plugin Loading Service:**
```bash
# Load a new plugin
ros2 service call /catalyst/load_plugin catalyst_interfaces/srv/LoadPlugin "{
  plugin_name: 'astar_pathplanner',
  plugin_type: 'algorithm'
}"
```

## Development Setup

### 1. Development Environment

**VS Code Setup:**
```bash
# Install VS Code extensions
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools

# Open workspace in VS Code
code .
```

**Recommended VS Code settings (`.vscode/settings.json`):**
```json
{
    "python.defaultInterpreterPath": "/usr/bin/python3",
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ]
}
```

### 2. Testing Setup

**Install test dependencies:**
```bash
pip3 install pytest pytest-cov
sudo apt install ros-humble-ament-cmake-pytest
```

**Run tests:**
```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select catalyst_core

# Run tests with coverage
colcon test --coverage-report-path coverage_report

# Run Python unit tests directly
cd src/catalyst_core
python3 -m pytest tests/ -v
```

### 3. Debugging

**Enable debug logging:**
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch catalyst_launch catalyst_system.launch.py log_level:=debug
```

**Debug with GDB:**
```bash
# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run with GDB
ros2 run --prefix 'gdb -ex run --args' catalyst_core platform_manager
```

**Performance profiling:**
```bash
# Install profiling tools
pip3 install py-spy

# Profile a running node
py-spy top --pid $(pgrep -f platform_manager)
```

## Monitoring and Visualization

### 1. System Monitoring

**Monitor plugin status:**
```bash
# View all plugin status
ros2 topic echo /catalyst/plugin_status/platform_manager

# View system topics
ros2 topic list | grep catalyst

# Monitor system performance
ros2 run catalyst_visualization system_monitor
```

### 2. RViz2 Visualization

**Start RViz2 with CATALYST configuration:**
```bash
rviz2 -d src/catalyst_visualization/rviz/catalyst_system.rviz
```

**Manual RViz2 setup:**
1. Add topics:
   - `/catalyst/path_plan` (visualization_msgs/MarkerArray)
   - `/catalyst/obstacle_map` (nav_msgs/OccupancyGrid)
   - `/catalyst/vehicle_state` (geometry_msgs/PoseStamped)

2. Set fixed frame to `map`

3. Save configuration for future use

### 3. Data Recording and Playback

**Record simulation data:**
```bash
# Record all CATALYST topics
ros2 bag record /catalyst/*

# Record specific topics
ros2 bag record /catalyst/path_plan /catalyst/vehicle_state
```

**Playback recorded data:**
```bash
# Playback at normal speed
ros2 bag play rosbag2_<timestamp>

# Playback at 2x speed
ros2 bag play rosbag2_<timestamp> --rate 2.0
```

## Troubleshooting

### Common Issues

**1. Build Failures**
```bash
# Clean build
rm -rf build install log
colcon build

# Check dependencies
rosdep check --from-paths src --ignore-src
```

**2. Import Errors**
```bash
# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"

# Source workspace
source install/setup.bash
```

**3. Plugin Loading Failures**
```bash
# Check plugin discovery
ros2 run catalyst_core plugin_registry

# Verify package installation
ros2 pkg list | grep catalyst
```

**4. Performance Issues**
```bash
# Monitor CPU usage
htop

# Check ROS2 performance
ros2 run performance_test perf_test --topic-name /catalyst/path_plan
```

### Getting Help

- **Documentation**: Check package README files
- **Logs**: Look in `~/.ros/log/` for detailed error logs
- **Forums**: ROS2 Discourse, Stack Overflow
- **Issues**: Create GitHub issues for bugs

---

## Next Steps

After successful setup:

1. **Validation**: Run validation tests against original MATLAB code
2. **Customization**: Modify configuration for your specific use case  
3. **Extension**: Add new algorithm plugins
4. **Integration**: Connect with your sensor hardware or simulation environment

🎉 **You're ready to use CATALYST!**
