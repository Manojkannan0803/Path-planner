# Software Engineering Assistant: Help in simulating the system (autonomous vehicle - whole software stack)

Do NOT generate code before reading the instructions of the applicable topic or file mentioned in the context. If the task involves changing a file, you MUST read the instructions for that file first.
You are helping an engineer with their software development. You read instructions and guidelines to provide accurate and helpful answers. Before answering, ensure you understand the question. If unsure, ask for clarification. Before generating code, ensure you understand the requirements. If unsure, ask for clarification. If you do not know the answer or are unsure of the answer, you should tell the engineer to ask for advice.

# Project Goals
- Developing Autonomous valet parking system in MATLAB Simulink, focusing on environment perception, path planning inclusive global and local strategies/planner, and vehicle control.
- Global planner - lattice based path planner with A* search algorithm
- Reference to global planner is available in the repository which was developed long back and it is monolithic
- One of the project goal and start point is to refactor that monolithic code. and it focuses on articulated vehicle.
- Apply layered architecture and model based development to that monolithic code. and, this project is to focus on car with three DOFs (x-position, y-position, orientation angle)
- Then optimize that code for better performance.
- Followed by how it could be extended - considering environmental perception simulation, path planning - behavioral planning, motion planning, trajectory generation.
- Existing vehicle controller to be replaced with Model Predictive Controller
- Have to be generic to any constrained environment

# Coding Standards
- Python: PEP8, type hints, modular packages, pytest for testing
- MATLAB: Functions > scripts, use OOP when appropriate, clear function signatures
- Documentation: Module-level docstrings + inline comments

# Architecture principles
This section provides architectural guidelines for developing an autonomous valet parking system using MATLAB/Simulink, following the proven modular architecture pattern demonstrated.

## 1. Core Architectural Principles

### 1.1 Component-Based Modular Design
- Each major subsystem should be organized as a MATLAB package (folder with "+" prefix)
- Each component should have clear interfaces and minimal coupling
- Hierarchical model structure with main system models and component sub-models

### 1.2 Multi-Configuration Support
- Support different vehicle platforms and parking environments
- Configurable system parameters through JSON/CSV files
- Separate configurations for different operational modes

### 1.3 Model-Based Systems Engineering
- Use Simulink as primary modeling environment
- Implement digital twin concepts for system validation
- Maintain separation between plant models and control logic

## 2. Recommended System Components Structure

### 2.1 Core Vehicle Subsystems
```
+VehicleDynamics/          - Vehicle kinematics and dynamics models
+Powertrain/               - Electric/hybrid powertrain models
+SteeringSystem/           - Steering actuator and feedback models
+BrakeSystem/              - Braking system and ABS models
+Suspension/               - Suspension dynamics and comfort models
```

### 2.2 Perception and Sensing
```
+CameraSensor/             - Camera-based perception systems
+LidarSensor/              - LiDAR point cloud processing
+RadarSensor/              - Radar object detection
+UltrasonicSensor/         - Ultrasonic proximity sensors
+IMUSensor/                - Inertial measurement unit
+GNSSSensor/               - GPS/GNSS positioning
+OdometrySensor/           - Wheel odometry and dead reckoning
```

### 2.3 Planning and Control
```
+PathPlanning/             - Global and local path planning algorithms
+MotionPlanning/           - Trajectory generation and optimization
+VehicleController/        - Low-level vehicle control (steering, throttle, brake)
+ParkingController/        - Parking-specific maneuvers and behaviors
+BehaviorPlanner/          - High-level decision making and state machine
```

### 2.4 Communication and Infrastructure
```
+V2XCommunication/         - Vehicle-to-everything communication
+ParkingInfrastructure/    - Parking lot sensors and communication
+CloudConnectivity/        - Cloud services and remote monitoring
+HMInterface/              - Human-machine interface (mobile app, displays)
```

### 2.5 Environment and Simulation
```
+ParkingEnvironment/       - Parking lot models and scenarios
+TrafficSimulation/        - Other vehicles and pedestrian models
+WeatherConditions/        - Environmental conditions (rain, snow, lighting)
+ObstacleModels/           - Static and dynamic obstacle representations
```

## 3. Main System Models

### 3.1 System-Level Models
```
MainValetParking.slx       - Complete autonomous valet parking system
MainSimulation.slx         - Simulation environment with scenarios
MainHIL.slx               - Hardware-in-the-loop testing model
MainCommon.slx            - Shared components and utilities
```

### 3.2 Scenario-Specific Models
```
ParkingLotA_Standard.slx   - Standard parking lot configuration
ParkingGarage_Multi.slx    - Multi-level parking garage
StreetParking_Urban.slx    - Urban street parking scenarios
ValetDropoff_Curbside.slx  - Curbside dropoff/pickup scenarios
```

## 4. Configuration Management

### 4.1 Vehicle Configurations
```
config/
 vehicle_sedan_standard.json     - Standard sedan configuration
 vehicle_suv_large.json          - Large SUV configuration  
 vehicle_compact_city.json       - Compact city car configuration
 sensors_premium.json            - Premium sensor suite
 sensors_basic.json              - Basic sensor configuration
 parking_environments/
     mall_parking.json           - Shopping mall parking lot
     airport_garage.json         - Multi-level airport garage
     residential_street.json     - Residential street parking
```

### 4.2 Parameter Data Files
```
config/data/
 vehicle_parameters.csv          - Vehicle physical parameters
 sensor_specifications.csv       - Sensor performance characteristics
 parking_space_dimensions.csv    - Standard parking space sizes
 traffic_patterns.csv           - Traffic flow and timing data
```

## 5. Testing and Validation Framework

### 5.1 Test Structure
```
Test/
 VehicleDynamicsTests.m          - Vehicle model validation
 SensorFusionTests.m             - Sensor integration testing
 PathPlanningTests.m             - Path planning algorithm tests
 ParkingManeuverTests.m          - Parking scenario validation
 SafetySystemTests.m             - Safety-critical system tests
 Harnesses/                      - Test harnesses and fixtures
    SensorTestHarness.slx
    ControllerTestHarness.slx
    ScenarioTestHarness.slx
 TestData/                       - Reference data and scenarios
     parking_scenarios/
     sensor_data_logs/
     reference_trajectories/
```

### 5.2 Automated Testing Framework
```
automation/
 testing/
    AutomatedParkingTestFramework.m
 documentation/
    SystemDocumentationGenerator.m
 reports/
     test_results/
```

## 6. Development Workflow

### 6.1 Component Development Process
1. **Define Component Interface**: Establish clear input/output specifications
2. **Implement Component Model**: Create Simulink model with proper masking
3. **Unit Testing**: Develop component-specific tests
4. **Integration Testing**: Test component within system context
5. **Documentation**: Auto-generate component documentation

### 6.2 System Integration Process
1. **Component Assembly**: Integrate components into system models
2. **Configuration Management**: Apply appropriate system configurations
3. **Scenario Testing**: Validate against parking scenarios
4. **Performance Testing**: Evaluate system performance metrics
5. **Safety Validation**: Verify safety-critical requirements

### 6.3 Continuous Integration Pipeline
```
Jenkinsfile or CI Configuration:
1. Code Checkout
2. Model Compilation Check
3. Unit Test Execution
4. Integration Test Suite
5. Scenario-Based Validation
6. Performance Benchmarking
7. Safety Requirements Verification
8. Documentation Generation
9. Artifact Publishing
```

## 7. Key Utility Functions

### 7.1 System Utilities
```
parking_system_root.m              - Root path configuration
parking_system_setpath.m           - Path setup for development
parking_system_test_all.m          - Execute all test suites
parking_system_config_loader.m     - Configuration file management
```

### 7.2 Simulation and Analysis
```
run_parking_scenario.m             - Execute specific parking scenarios
analyze_parking_performance.m      - Performance metrics analysis
generate_test_reports.m            - Automated report generation
parking_scenario_generator.m       - Procedural scenario generation
```

## 8. Model Development Guidelines

### 8.1 Simulink Model Standards
- **Naming Convention**: Use descriptive, hierarchical naming
- **Model Masking**: Create masked subsystems for reusable components
- **Parameter Management**: Use Model Workspace and Data Dictionary
- **Signal Naming**: Clear, descriptive signal names throughout
- **Documentation**: Comprehensive model documentation and annotations

### 8.2 Code Generation Considerations
- **Real-Time Compatibility**: Ensure models are real-time code generation ready
- **Memory Management**: Optimize for embedded target memory constraints
- **Computational Efficiency**: Consider algorithm complexity for real-time performance
- **Safety Standards**: Follow automotive safety standards (ISO 26262)

## 9. Safety and Validation Requirements

### 9.1 Safety-Critical Components
- **Emergency Braking System**: Fail-safe braking mechanisms
- **Collision Avoidance**: Real-time obstacle detection and avoidance
- **Human Override**: Manual control takeover capabilities
- **System Monitoring**: Health monitoring and fault detection

### 9.2 Validation Scenarios
```
Safety Test Scenarios:
 obstacle_sudden_appearance.slx    - Sudden obstacle scenarios
 sensor_failure_modes.slx          - Sensor degradation/failure
 communication_loss.slx            - V2X communication failures  
 weather_adverse.slx               - Adverse weather conditions
 emergency_situations.slx          - Emergency response scenarios
```

## 10. Deployment and Production Considerations

### 10.1 Target Hardware Platforms
- **Automotive ECUs**: Production-grade automotive computers
- **Development Platforms**: dSPACE, Speedgoat for HIL testing
- **Simulation Clusters**: High-performance computing for scenario testing
- **Edge Computing**: Local processing capabilities

### 10.2 Production Pipeline
1. **Model Validation**: Comprehensive testing and validation
2. **Code Generation**: Automatic C/C++ code generation
3. **Integration Testing**: Hardware-in-the-loop validation
4. **Calibration**: Parameter tuning and optimization
5. **Deployment**: Production system deployment
6. **Monitoring**: Operational performance monitoring

## 11. Performance Metrics and KPIs

### 11.1 System Performance Indicators
- **Parking Success Rate**: Percentage of successful parking attempts
- **Parking Time**: Average time to complete parking maneuver
- **Path Efficiency**: Optimality of chosen parking paths
- **Safety Margins**: Minimum distances maintained from obstacles
- **Energy Consumption**: Efficiency of parking maneuvers

### 11.2 Component Performance Metrics
- **Sensor Accuracy**: Detection and measurement accuracy
- **Control Response**: Actuator response times and accuracy
- **Planning Efficiency**: Computational performance of algorithms
- **Communication Latency**: V2X and cloud communication delays

## 12. Implementation Phases

### Phase 1: Foundation Setup
- [ ] Create project structure following component architecture
- [ ] Set up MATLAB/Simulink development environment
- [ ] Initialize version control with appropriate .gitignore
- [ ] Create basic vehicle dynamics model
- [ ] Implement simple parking scenario

### Phase 2: Core Components Development
- [ ] Develop sensor simulation models (Camera, LiDAR, Ultrasonic)
- [ ] Implement path planning algorithms
- [ ] Create vehicle control systems
- [ ] Build parking environment models
- [ ] Establish component interfaces

### Phase 3: Integration and Testing
- [ ] Integrate components into system models
- [ ] Set up automated testing framework
- [ ] Create test scenarios and harnesses
- [ ] Implement safety systems
- [ ] Validate parking maneuvers

### Phase 4: Advanced Features
- [ ] Add multi-vehicle scenarios
- [ ] Implement V2X communication
- [ ] Create advanced weather conditions
- [ ] Add machine learning components
- [ ] Optimize for real-time performance

### Phase 5: Production Preparation
- [ ] Hardware-in-the-loop testing
- [ ] Code generation optimization
- [ ] Safety validation and certification
- [ ] Performance benchmarking
- [ ] Documentation and deployment

---

## Getting Started Checklist

- [ ] Set up MATLAB/Simulink development environment
- [ ] Create project folder structure following component architecture
- [ ] Initialize version control (Git) with appropriate .gitignore
- [ ] Set up configuration management system
- [ ] Implement basic vehicle dynamics model
- [ ] Develop sensor simulation models
- [ ] Create simple parking scenario for initial testing
- [ ] Establish automated testing framework
- [ ] Set up continuous integration pipeline
- [ ] Document system architecture and component interfaces

## Additional Resources

### MATLAB/Simulink Toolboxes Required
- **Automated Driving Toolbox**: Vehicle dynamics and sensor models
- **Robotics System Toolbox**: Path planning and navigation
- **Computer Vision Toolbox**: Image processing and object detection
- **Sensor Fusion and Tracking Toolbox**: Multi-sensor data fusion
- **Vehicle Dynamics Blockset**: Detailed vehicle modeling
- **Simulink Real-Time**: Real-time simulation and testing

### Standards and Guidelines
- **ISO 26262**: Functional safety for automotive systems
- **SAE J3016**: Levels of driving automation
- **ISO 14229**: Unified diagnostic services
- **SAE J2735**: Dedicated short range communications message set

This architecture provides a scalable, maintainable foundation for autonomous valet parking system development while leveraging proven patterns from complex systems engineering projects.

