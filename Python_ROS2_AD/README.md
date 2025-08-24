# CATALYST ROS2 Workspace

## XX (work)-Inspired Layered Architecture for Autonomous Vehicle Path Planning

This workspace implements a complete autonomous vehicle obstacle avoidance system using ROS2, converting MATLAB algorithms to Python with a XX (work)-inspired layered architecture.

### 🏗️ Architecture Overview

The system follows XX (work)'s modular plugin architecture with four main layers:

```
┌─────────────────────────────────────────────────────────┐
│                 PLATFORM LAYER                          │
│              (catalyst_core)                            │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ • Platform Manager (System Orchestration)          │ │
│  │ • Plugin Registry (Plugin Lifecycle)               │ │
│  │ • Configuration Manager (System Config)            │ │
│  │ • Performance Monitoring                           │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│               INTERFACE LAYER                           │
│             (catalyst_interfaces)                       │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ • Message Definitions (VehicleState, PathPlan)     │ │
│  │ • Service Definitions (PlanPath, LoadPlugin)       │ │
│  │ • Data Interceptors & Adapters                     │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│              ALGORITHM LAYER                            │
│            (catalyst_algorithms)                        │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ • A* Path Planner (from Pathplanning_Astar.m)      │ │
│  │ • Cost Calculators (from g_cost.m, h_cost.m)       │ │
│  │ • Collision Detection (from staticobs_check.m)     │ │
│  │ • Future: RRT*, Hybrid A*, etc.                    │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                MODEL LAYER                              │
│              (catalyst_models)                          │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ • Environment Model (from DPDscenario.m)           │ │
│  │ • Vehicle Dynamics Model                           │ │
│  │ • Motion Primitive Library                         │ │
│  │ • Sensor Models                                    │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### 📦 Package Structure

```
catalyst_ws/
├── src/
│   ├── catalyst_core/              # 🏛️ Platform Layer
│   │   ├── catalyst_core/
│   │   │   ├── platform_manager.py      # Main orchestrator
│   │   │   ├── plugin_registry.py       # Plugin management
│   │   │   ├── configuration_manager.py # Config handling
│   │   │   ├── base_plugin.py           # Plugin base class
│   │   │   └── __init__.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── catalyst_interfaces/        # 🔌 Interface Layer
│   │   ├── msg/
│   │   │   ├── VehicleState.msg         # Vehicle state representation
│   │   │   ├── ObstacleMap.msg          # Environment obstacles
│   │   │   ├── PathPlan.msg             # Generated path plans
│   │   │   ├── MotionPrimitive.msg      # Motion primitive data
│   │   │   ├── AlgorithmConfig.msg      # Plugin configuration
│   │   │   └── PluginStatus.msg         # Plugin monitoring
│   │   ├── srv/
│   │   │   ├── PlanPath.srv             # Path planning service
│   │   │   └── LoadPlugin.srv           # Plugin loading service
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── catalyst_algorithms/        # 🧠 Algorithm Layer
│   │   ├── catalyst_algorithms/
│   │   │   ├── astar_plugin.py          # A* implementation (from .m)
│   │   │   ├── cost_calculators.py      # Cost functions (from .m)
│   │   │   ├── collision_detection.py   # Collision checking (from .m)
│   │   │   └── __init__.py
│   │   └── package.xml
│   │
│   ├── catalyst_models/            # 🚗 Model Layer  
│   │   ├── catalyst_models/
│   │   │   ├── environment_model.py     # DC environment (from .m)
│   │   │   ├── vehicle_model.py         # Vehicle dynamics
│   │   │   ├── motion_primitives.py     # Motion primitive library
│   │   │   └── __init__.py
│   │   └── package.xml
│   │
│   ├── catalyst_sensors/           # 📡 Sensor Layer
│   │   ├── catalyst_sensors/
│   │   │   ├── sensor_interceptors.py   # Sensor data adapters
│   │   │   └── __init__.py
│   │   └── package.xml
│   │
│   ├── catalyst_visualization/     # 📊 Visualization Layer
│   │   ├── catalyst_visualization/
│   │   │   ├── path_visualizer.py       # RViz path display
│   │   │   └── __init__.py
│   │   └── package.xml
│   │
│   └── catalyst_launch/            # 🚀 Launch Configurations
│       ├── launch/
│       │   └── catalyst_system.launch.py  # Complete system launch
│       └── package.xml
│
├── README.md                       # This file
└── workspace_setup.md             # Setup instructions
```

### 🔄 MATLAB to Python Conversion

This system converts the following MATLAB components:

| MATLAB File | Python Component | Package | Description |
|-------------|------------------|---------|-------------|
| `Pathplanning_Astar.m` | `astar_plugin.py` | catalyst_algorithms | A* path planning algorithm |
| `g_cost.m` | `cost_calculators.py` | catalyst_algorithms | G-cost calculation |
| `h_cost.m` | `cost_calculators.py` | catalyst_algorithms | Heuristic cost calculation |
| `staticobs_check.m` | `collision_detection.py` | catalyst_algorithms | Static obstacle collision checking |
| `DPDscenario.m` | `environment_model.py` | catalyst_models | Distribution center environment |
| `Plot_pathgenerated.m` | `path_visualizer.py` | catalyst_visualization | Path visualization |
| Motion primitive `.mat` files | `motion_primitives.py` | catalyst_models | Motion primitive library |

### 🚀 Getting Started

#### Prerequisites
- ROS2 Humble/Iron/Rolling
- Python 3.8+
- NumPy, SciPy, Matplotlib

#### Build and Run

1. **Clone and build the workspace:**
   ```bash
   cd catalyst_ws
   colcon build
   source install/setup.bash
   ```

2. **Launch the complete system:**
   ```bash
   ros2 launch catalyst_launch catalyst_system.launch.py
   ```

3. **Launch with custom configuration:**
   ```bash
   ros2 launch catalyst_launch catalyst_system.launch.py \
     config_file:=/path/to/custom_config.yaml \
     enable_visualization:=true \
     log_level:=debug
   ```

#### Individual Components

- **Platform Manager:**
  ```bash
  ros2 run catalyst_core platform_manager
  ```

- **A* Path Planner:**
  ```bash
  ros2 run catalyst_algorithms astar_plugin
  ```

- **Environment Model:**
  ```bash
  ros2 run catalyst_models environment_model
  ```

### 🔧 Configuration

System configuration follows XX (work)'s configuration pattern with YAML files:

```yaml
platform:
  name: "CATALYST"
  version: "1.0.0"
  max_plugins: 50

algorithms:
  default_planner: "astar"
  planning_timeout: 30.0

environment:
  distribution_center:
    length: 328.0  # meters (from MATLAB)
    width: 200.0   # meters (from MATLAB)

vehicle:
  articulated_vehicle:
    trailer_wheelbase: 8.475    # L_1f (from MATLAB)
    tractor_wheelbase: 3.8      # L_0f (from MATLAB)
    trailer_width: 2.5          # w_1 (from MATLAB)

path_planning:
  astar:
    heuristic_weight: 2.5       # From h_cost.m
    penalty_factor: 1000.0      # From h_cost.m
    max_iterations: 10000
```

### 🧪 Testing and Validation

The system includes comprehensive testing to validate MATLAB-to-Python conversion:

- **Unit tests** for each converted algorithm
- **Integration tests** for plugin interactions  
- **Validation tests** comparing Python results with original MATLAB
- **Performance benchmarks** for real-time operation

### 📊 Monitoring and Performance

XX (work)-inspired monitoring provides:

- **Plugin status monitoring** via `/catalyst/plugin_status/*`
- **Performance metrics** (execution time, success rate)
- **System health monitoring** 
- **Real-time visualization** in RViz2

### 🔌 Plugin Development

Adding new algorithms follows the XX (work) plugin pattern:

```python
from catalyst_core.base_plugin import CatalystPlugin

class MyAlgorithmPlugin(CatalystPlugin):
    def __init__(self):
        super().__init__("my_algorithm", "algorithm")
    
    def initialize(self, config):
        # Plugin initialization
        return True
    
    def execute(self, input_data):
        # Algorithm implementation
        return result
    
    def cleanup(self):
        # Resource cleanup
        pass
```

### 🎯 Key Benefits

✅ **Modular Architecture**: Easy to extend with new algorithms
✅ **Real-time Capable**: ROS2 enables real-time operation
✅ **Testable**: Clear separation enables comprehensive testing  
✅ **Maintainable**: Modular-inspired design patterns
✅ **Scalable**: Distributed execution across multiple machines
✅ **Compatible**: Seamless integration with existing ROS2 systems

### 🚧 Current Status

- ✅ **Phase 1**: ROS2 workspace structure created
- ⏳ **Phase 2**: MATLAB to Python conversion in progress
- ⏳ **Phase 3**: Plugin integration and testing
- ⏳ **Phase 4**: Performance optimization and validation


