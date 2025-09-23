# Stakeholder Requirements Gathering (Autonomous Valet Parking)

**Purpose:** Systematic collection of stakeholder requirements to finalize system specifications before detailed architecture design and implementation.

**Role:** Software Architect conducting requirements elicitation  
**Stakeholder:** System Owner/User (Autonomous Vehicle System Integrator)  
**Scope:** Motion planning subsystem for 3-DOF autonomous valet parking

---

## 1. Operational Context & Use Cases

### 1.1 Environment & Scenarios
**Q1.1:** What types of parking environments will the system operate in?
- [ ] Private parking lots (shopping malls, offices)
- [ ] Public street parking (parallel, angled)
- [ ] Multi-level parking garages
- [ ] Mixed-use facilities (drop-off zones + parking)
- [ ] Other: _______________
Answer from stakeholder: I would like to kick start this project with private parking lot i.e., offices. But it has to be made scalable for other environments like public street parking, multi-level parking garages eventually in the future.

**Q1.2:** What are the typical parking space dimensions and constraints?
- Standard space width: _____ m (e.g., 2.4-3.0m)
- Standard space length: _____ m (e.g., 5.0-6.0m)
- Minimum clearance from obstacles: _____ m (e.g., 0.25-0.5m)
- Maximum acceptable turning radius: _____ m

Answer from stakeholder: 
- Standard space width: 2.30 - 2.50 m 
- Standard space length: 4.80 - 5.00 m 
- Minimum clearance from obstacles: 0.25 - 0.50 m
- Maximum acceptable turning radius: ~6.0 m

**Q1.3:** What dynamic obstacles must the system handle?
- [ ] Pedestrians (walking speed ~1.5 m/s)
- [ ] Other vehicles (parking lot speeds ~5-10 km/h)
- [ ] Shopping carts, wheelchairs
- [ ] Service vehicles (delivery, maintenance)
- [ ] Static obstacles appearing/disappearing (parked cars)

Answer from stakeholder:  Yes, all dynamic obstacles listed above

**Q1.4:** What maneuver complexity is required?
- [ ] Simple forward parking (drive-in spots)
- [ ] Parallel parking (reverse required)
- [ ] Perpendicular parking with tight approach angles
- [ ] Multi-point turns in confined spaces
- [ ] Backing out of spaces with limited visibility

Answer from stakeholder: Yes, all maneuver complexity required

### 1.2 Mission Profiles
**Q1.5:** What is the typical planning distance and complexity?
- Average parking search area: _____ m x _____ m
- Maximum number of obstacles in scene: _____ (e.g., 50-300)
- Expected number of reverse maneuvers per parking: _____ (e.g., 0-3)

Answer from stakeholder: 
- Average parking search area: 250 m x 250 m
- Maximum number of obstacles in scene: depends - start with low complexity, test and increase it gradually
- Expected number of reverse maneuvers per parking: depends on the use case

**Q1.6:** How frequently will replanning occur?
- Expected dynamic obstacle encounters per parking attempt: _____
- Acceptable number of replans per mission: _____ (cost consideration)

Answer from stakeholder: 
- Expected dynamic obstacle encounters per parking attempt: depends - start with low complexity, test and increase it gradually
- Acceptable number of replans per mission: depends on the use case; but try to keep it minimal as possible

---

## 2. Vehicle & Platform Constraints

### 2.1 Vehicle Specifications
**Q2.1:** What vehicle platform(s) must be supported?
- Wheelbase: _____ m (e.g., 2.85m sedan, 3.2m SUV)
- Overall length: _____ m
- Overall width: _____ m  
- Minimum turning radius: _____ m
- [ ] Single vehicle type initially
- [ ] Multiple vehicle classes (sedan, SUV, truck)

Answer from stakeholder:
- Wheelbase: 2.55 m
- Overall length: 4.07 m
- Overall width: 1.75 m  
- Minimum turning radius: 5.3 m


**Q2.2:** What are the kinematic limits?
- Maximum steering angle: 44 degrees
- Maximum steering rate: ~400 - 600 deg/s
- Maximum forward speed (parking): ~ 2 - 3 m/s 
- Maximum reverse speed: ~1 - 1.5 m/s

**Q2.3:** Are there articulated vehicle requirements?
- [ ] No - standard passenger vehicles only
- [ ] Yes - trailer combinations (articulation angle γ required)
- If yes, maximum articulation angle: _____ degrees

Answer from stakeholder: we will focus on standard passenger vehicles.

### 2.2 Sensing & Actuation
**Q2.4:** What sensors provide environmental data?
- [ ] Camera (free space estimation)
- [ ] Radar (moving object detection)
- [ ] Ultrasonic (near-field obstacles)
- [ ] Lidar (precise localization & mapping)
- [ ] Other: _______________

Answer from stakeholder: yes, as mentioned above!

**Q2.5:** What is the sensor detection range and update rate?
- Forward detection range: _____ m
- Rear detection range: _____ m
- Lateral detection range: _____ m
- Sensor fusion update rate: _____ Hz

Answer from stakeholder: 
Forward detection range: 30–50 m (AVP use case)
Rear detection range: 10–15 m
Lateral detection range: 5–10 m
Sensor fusion update rate: 10–20 Hz (AVP)

---

## 3. Performance Requirements

### 3.1 Timing & Responsiveness
**Q3.1:** What are the acceptable planning latencies?
- Initial path planning (cold start): _____ ms (e.g., 500-2000ms)
- Replanning after obstacle detection: _____ ms (e.g., 100-500ms)
- Emergency stop reaction time: _____ ms (e.g., 50-150ms)

Answer from stakeholder:
- Initial path planning (cold start): 500-2000ms (Longer is acceptable since it's done once at the start)
- Replanning after obstacle detection:100-300ms (Must be quick to avoid collisions)
- Emergency stop reaction time: 50-150ms (critical for safety)

**Q3.2:** What execution rates are required?
Answer from stakeholder:
- Global planner update: 1Hz or event-driven
- Local path adaptation: 10-20Hz
- Trajectory generation: 20-50Hz
- Safety monitoring: 50Hz-100Hz (Higher frequency ensures timely fault detection)

**Q3.3:** What computational resources are available?
- Target CPU platform: _____ (e.g., automotive ECU, embedded x86, GPU)
- Available memory for planning: _____ MB
- Real-time OS requirements: [ ] Yes [ ] No
- Code generation required: [ ] Yes [ ] No (MATLAB/C++)

Answer from stakeholder:
- Target CPU platform: Automotive-grade ECU or embeddedx86
- Available memory for planning: 512 - 1024 MB(planning stack only; full system needs more)
- Real-time OS requirements: Yes for deterministic execution
- Code generation required: Yes


### 3.2 Path Quality & Comfort
**Q3.4:** What path quality metrics are important?
- [ ] Minimum path length (fuel efficiency)
- [ ] Minimum maneuvering time
- [ ] Passenger comfort (smooth acceleration/steering)
- [ ] Minimize reverse distance (driver preference)
- Priority ranking: 1. _____ 2. _____ 3. _____

Answer from stakeholder:
1. Minimum path length 2. Minimum maneuvering time 

**Q3.5:** What are the comfort constraints?
- Maximum longitudinal acceleration: _____ m/s²
- Maximum longitudinal jerk: _____ m/s³
- Maximum steering rate: _____ deg/s
- Dwell time at direction changes: _____ s (gearbox settling)

Answer from stakeholder:
- Maximum longitudinal acceleration: 2.0 - 3.0 m/s²
- Maximum longitudinal jerk: 1.0 - 2.0 m/s³
- Maximum steering rate: 30 - 40 deg/s
- Dwell time at direction changes: 0.5 - 1.0 s (gearbox settling)
---

## 4. Safety & Reliability Requirements

### 4.1 Collision Avoidance
**Q4.1:** What safety margins are required?
- Minimum clearance to static obstacles: _____ m
- Minimum clearance to moving obstacles: _____ m
- Safety factor for sensor uncertainty: _____ (e.g., 1.2-1.5x)

Answer from stakeholder:
- Minimum clearance to static obstacles: 0.5 - 1.0 m
- Minimum clearance to moving obstacles: 1.0 - 2.0 m
- Safety factor for sensor uncertainty: 1.2-1.5x

**Q4.2:** What failure response is expected?
- [ ] Graceful degradation (conservative planning)
- [ ] Safe stop and alert human operator
- [ ] Handover to remote operator
- [ ] Multiple strategies based on failure type

Answer from stakeholder:
safe stop 

**Q4.3:** Are there regulatory or certification requirements?
- [ ] ISO 26262 ASIL rating required: _____ (A/B/C/D)
- [ ] Regional safety standards (EU, US DOT, etc.)
- [ ] Insurance/liability requirements
- [ ] Internal company safety standards

Answer from stakeholder: Not yet decided - but follow regional safety standards defined by EU

### 4.2 Robustness & Edge Cases
**Q4.4:** How should the system handle edge cases?
- No valid path to goal: [ ] Report failure [ ] Suggest alternative goal
- Sensor degradation: [ ] Reduce speed [ ] Stop safely [ ] Manual override
- Dynamic obstacle blocking path: [ ] Wait [ ] Replan [ ] Request assistance
- Computational overload: [ ] Degrade resolution [ ] Fallback algorithm [ ] Stop

Answer from stakeholder: consider all edge cases
---

## 5. Integration & Deployment

### 5.1 System Architecture Context  
**Q5.1:** How does this subsystem integrate with the larger system?
- Receives parking goal from: _____ (e.g., mission planner, user interface)
- Sends trajectory commands to: _____ (e.g., vehicle controller, CAN bus)
- Interfaces with perception via: _____ (e.g., ROS topics, shared memory, custom API)

Answer from stakeholder: 
- Receives parking goal from user interface
- sends trajectory commands to vehicle controller
- interfaces with perception via: ROS topics at the moment we can placeholder. but if you know other approach, provide them!

**Q5.2:** What data formats and protocols are required?
- Input map format: [ ] Occupancy grid [ ] Vector polygons [ ] HD map [ ] Custom
- Trajectory output format: [ ] Waypoints [ ] Spline coefficients [ ] Time-stamped states
- Communication protocol: [ ] ROS/ROS2 [ ] Custom middleware [ ] Direct function calls

- Input map format: Occupancy grid
- Trajectory output format: waypoints
- communication protocol: ROS topics at the moment we can placeholder. but if you know other approach, provide them!

**Q5.3:** What simulation and testing environments are needed?
- [ ] MATLAB/Simulink SIL (Software-in-Loop)
- [ ] CarSim/IPG vehicle dynamics integration
- [ ] Unity/Unreal visualization
- [ ] Hardware-in-Loop (HIL) test bench
- [ ] Real vehicle testing capability

Answer from stakeholder: for now, we will focus on matlab/simulink environment alone

### 5.2 Development & Maintenance
**Q5.4:** What are the development and timeline constraints?
- Target completion date: _____
- Available development resources: _____ person-months
- Budget constraints affecting algorithm complexity: [ ] Yes [ ] No
- Preference for proven algorithms vs. cutting-edge research: _____

Answer from stakeholder: Quarter Q4

**Q5.5:** What maintainability and extensibility requirements exist?
- [ ] Support multiple vehicle platforms without code changes
- [ ] Tunable parameters via configuration files
- [ ] Scenario-based regression testing capability
- [ ] Performance monitoring and diagnostics in production
- [ ] Remote parameter updates (over-the-air)

Answer from stakeholder:
at the moment, i could think of Support multiple vehicle platforms without code changes and Scenario-based regression testing capability

---

## 6. Success Criteria & Acceptance

### 6.1 Performance Metrics
**Q6.1:** What quantitative success criteria define project completion?
- Planning success rate: ≥ ____% (e.g., 95-99%)
- Average planning time: ≤ _____ ms
- Collision avoidance rate: ≥ ____% (e.g., 99.9%+)
- Path optimality vs. theoretical minimum: ≥ ____% (e.g., 85-95%)

Answer from stakeholder: try to achieve as much as possible

**Q6.2:** What test scenarios must be passed?
- Number of regression test cases: _____ (e.g., 50-200)
- Environmental conditions: [ ] Day/night [ ] Weather [ ] Lighting variations
- Stress testing: _____ obstacle density, _____ dynamic objects

Answer from stakeholder: consider as much as test scenarios

### 6.3 Validation Approach
**Q6.3:** How will requirements be verified?
- [ ] Simulation-based verification (SIL)
- [ ] HIL testing with real sensors
- [ ] Closed-course real vehicle testing  
- [ ] Public road testing (if applicable)
- [ ] Statistical analysis over large scenario datasets

Answer from stakeholder: we will be performing only MIL and SIL in matlab/simulink environment
---

## 7. Risk Tolerance & Trade-offs

### 7.1 Acceptable Trade-offs
**Q7.1:** When performance conflicts arise, what is the priority order?
1. Safety (collision avoidance)
2. _____
3. _____
4. _____
(Options: speed, comfort, path optimality, computational efficiency)

**Q7.2:** What level of planning complexity is acceptable?
- Computational budget per planning cycle: _____ ms
- Memory footprint limit: _____ MB
- Preference for deterministic vs. optimal solutions: _____

### 7.2 Future Evolution
**Q7.3:** What future capabilities should the architecture accommodate?
- [ ] Dynamic obstacle trajectory prediction
- [ ] Multi-vehicle coordination (avoid other autonomous cars)
- [ ] Learning from driver behavior patterns
- [ ] Cloud-based route optimization
- [ ] Integration with smart parking infrastructure (V2I)

Answer from stakeholder: Dynamic obstacle trajectory prediction, Multi-vehicle coordination (avoid other autonomous cars), Learning from driver behavior patterns
---

## 8. Documentation & Compliance

**Q8.1:** What documentation deliverables are required?
- [ ] Requirements traceability matrix
- [ ] Safety analysis (HAZOP, FMEA)
- [ ] Verification & validation reports
- [ ] User/operator manuals
- [ ] Software design documentation

Answer: all

**Q8.2:** What standards and processes must be followed?
- [ ] ASPICE (Automotive SPICE) compliance
- [ ] ISO 26262 functional safety
- [ ] Company-specific engineering processes
- [ ] Version control and change management requirements

answer: all

---

## Next Steps After Requirements Gathering

1. **Requirement Prioritization:** Rank requirements by criticality (must-have, should-have, nice-to-have)
2. **Requirement Validation:** Review with stakeholders for completeness and feasibility
3. **System Architecture Update:** Refine planning_architecture.md based on clarified requirements
4. **Verification Strategy:** Define test cases and acceptance criteria for each requirement
5. **Risk Assessment:** Identify technical risks and mitigation strategies
6. **Implementation Planning:** Update development phases based on stakeholder priorities

---

**Template Usage:** Present this questionnaire to stakeholders, collect responses, and use answers to finalize the system requirements before proceeding with detailed architectural design and developer implementation guidance.