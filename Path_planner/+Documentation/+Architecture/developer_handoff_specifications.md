# Requirements Traceability Matrix & Developer Handoff

**Date:** 2025-09-23  
**Status:** READY FOR IMPLEMENTATION  
**Phase:** 1 (Office Parking Lots - Q4 2025)

---

## Requirements Traceability Matrix

| Req ID | Stakeholder Requirement | Design Allocation | Implementation Target | Verification Method |
|--------|------------------------|-------------------|----------------------|-------------------|
| **REQ-ENV-001** | Office parking lots (2.3-2.5m spaces) | GlobalPlanner + collision checker | Hybrid A* with vehicle footprint | MIL scenario suite |
| **REQ-ENV-002** | 25-50 obstacles max (Phase 1) | Search space + memory allocation | Preallocated arrays (50 obstacles) | Performance profiling |
| **REQ-ENV-003** | 0-2 pedestrians, 0-1 moving vehicle | DynamicObstacle handler | Constant velocity prediction | SIL dynamic scenarios |
| **REQ-VEH-001** | Vehicle: 2.55m wheelbase, 4.07m length, 1.75m width | VehicleParams struct | Configuration YAML loader | Unit test validation |
| **REQ-VEH-002** | Min turning radius: 5.3m | Motion primitive generator | Curvature-bounded primitives | Primitive validation |
| **REQ-VEH-003** | Max steering: 44°, rate: 30-40°/s | Kinematic limits | TrajectoryGenerator constraints | Limit enforcement tests |
| **REQ-PERF-001** | Planning time: 500-2000ms initial | GlobalPlanner timeout | Hybrid A* with timeout guard | Timing benchmarks |
| **REQ-PERF-002** | Replan time: 100-300ms | LocalPathAdapter cycle | Incremental search optimization | Replan latency tests |
| **REQ-PERF-003** | Emergency stop: 50-150ms | CollisionMonitor | Real-time collision checking | Safety response tests |
| **REQ-SAFE-001** | Clearance: 0.5-1.0m static, 1.0-2.0m dynamic | Collision checker margins | Inflated obstacle boundaries | Clearance validation |
| **REQ-SAFE-002** | Safe stop on failure | Emergency handler | Graceful degradation logic | Failure injection tests |
| **REQ-PATH-001** | Priority: Length > Time > Comfort > Efficiency | Cost function weights | Multi-objective optimization | Path quality metrics |
| **REQ-PATH-002** | Max reverse maneuvers: 0-2 per parking | Direction change penalty | Reverse sequence limiting | Maneuver counting |
| **REQ-INT-001** | MATLAB/Simulink native interface | Bus structures | Simulink block implementation | Integration tests |
| **REQ-EDGE-001** | No path → report failure + alternatives | Failure handler | Goal suggestion logic | Edge case scenarios |
| **REQ-EDGE-002** | Sensor degraded → reduce speed 50% | Safety monitor | Adaptive speed limits | Sensor fault simulation |

---

## Developer Implementation Priorities

### **Phase 1A (Weeks 1-3): Foundation**
1. **VehicleParams & Configuration System**
   - Load YAML config with finalized vehicle specs
   - Implement parameter validation and bounds checking
   - Create unit tests for all parameter ranges

2. **Bus Structures & Interfaces**
   - Define all Simulink bus structures (MapBus, PathBus, TrajectoryBus, etc.)
   - Implement bus validation functions
   - Create bus initialization utilities

3. **Motion Primitive Infrastructure**  
   - **Phase 1A:** Use existing ACADO primitives as reference baseline
   - **Phase 1B:** Develop CasADi Python generator with crucial considerations:
     * Vehicle-specific constraints (2.55m wheelbase, 5.3m turning radius)
     * Kinematic feasibility validation (curvature continuity)
     * Endpoint accuracy requirements (±0.1m position, ±2° heading)
     * Computational efficiency (generate offline, load at runtime)
   - Implement primitive validation framework (curvature bounds, collision-free)
   - Create primitive comparison tools (ACADO vs CasADi quality metrics)

### **Phase 1B (Weeks 4-6): Core Planning**
4. **GlobalPlanner (Hybrid A*)**
   - Priority queue implementation (binary heap)
   - State hashing with finalized discretization (0.25m spatial, 7.5° angular)
   - Basic Euclidean heuristic (Reeds-Shepp upgrade in Phase 2)
   - Timeout handling (1250ms initial, 200ms replan)

5. **Collision Detection**
   - Vehicle footprint definition (4.07m × 1.75m rectangle)
   - Polygon-based obstacle checking
   - Clearance margin enforcement (0.75m static, 1.5m dynamic)
   - Multi-sensor uncertainty handling (1.35× safety factor)

### **Phase 1C (Weeks 7-9): Integration & Validation**
6. **TrajectoryGenerator**
   - S-curve velocity profiles with CasADi optimization integration
   - Direction change handling (0.75s dwell time)
   - Kinematic limit enforcement (2.5 m/s² accel, 1.5 m/s³ jerk)
   - CasADi Python bridge for real-time trajectory refinement

7. **Safety & Monitoring**
   - Emergency stop logic (<100ms response)
   - Edge case handlers (confirmed strategies)
   - Performance monitoring (timing, memory usage)

8. **MIL Test Suite**
   - 10 basic scenarios (forward parking, simple obstacles)
   - 15 dynamic scenarios (pedestrians, moving vehicles)
   - 10 edge cases (tight spaces, no path, sensor degradation)

---

## Critical Implementation Notes

### **Architecture Decisions Made:**
- ✅ **MATLAB/Simulink Native** (no ROS bridge Phase 1)
- ✅ **Hybrid A* primary** (lattice A* fallback for determinism)
- ✅ **Phase 1 scope:** 25-50 obstacles, office parking only
- ✅ **Multi-objective cost:** Length > Time > Comfort > Efficiency

### **Technical Constraints:**
- Memory budget: 256MB for planning stack
- Real-time OS required for deterministic execution
- Code generation compatibility (no dynamic figures, preallocated arrays)
- Sensor precision challenge accepted (tight 0.275m per side margins)

### **Performance Targets:**
- Planning success rate: ≥95% (Phase 1)
- Average planning time: ≤800ms
- Collision avoidance: ≥99.5%
- Path optimality: ≥85% vs theoretical minimum

---

## Developer Questions - PRE-ANSWERED

Based on developer's original questions, here are the architect-approved answers:

### **Q1: Primitive Generation Strategy**
**Answer:** Phase 1A: Use existing ACADO primitives as reference baseline. Phase 1B: Implement CasADi Python generator with critical considerations: vehicle-specific dynamics, curvature continuity, endpoint precision, and offline generation for runtime efficiency. Create validation framework to ensure CasADi primitives meet or exceed ACADO quality.

### **Q2: State Discretization**
**Answer:** θ discretization = 7.5° (confirmed in config). Ignore articulation γ for 3-DOF case.

### **Q3: Collision Detection**
**Answer:** Phase 1 = rectangular vehicle footprint (4.07m × 1.75m). Swept volume is Phase 2 enhancement.

### **Q4: Real-time Constraints**
**Answer:** Implement timeout guards in each block. Use Simulink execution framework, add diagnostic timing outputs.

### **Q5: Code Generation**
**Answer:** Target MATLAB R2023a+ with Simulink Coder. Preallocate for 50 obstacles, 1000 nodes max.

### **Q6: Integration Strategy**
**Answer:** Create new files alongside existing. Gradually replace starting with `hybridAStar.m` function. Maintain backward compatibility.

### **Q7: Testing Priorities**
**Answer:** Focus on functional correctness first, then performance benchmarks. Determinism validation included in test suite.

### **Q8: Configuration**
**Answer:** Static YAML loading at startup. Runtime parameter updates not required Phase 1.

---

## SUCCESS CRITERIA

### **Phase 1 Acceptance Tests:**
1. **Functional:** Park in 90% of test scenarios (25 baseline cases)
2. **Performance:** Average planning <800ms, replan <200ms
3. **Safety:** Zero collisions in test suite, proper clearance margins
4. **Integration:** Clean Simulink block interface, no code generation errors

### **Demo Scenarios for Q4:**
1. Simple forward parking (empty lot)
2. Parallel parking with 1 static obstacle
3. Dynamic pedestrian avoidance during approach
4. Tight space maneuvering (2.3m width space)
5. No path scenario (blocked goal, suggest alternative)

---

## READY FOR DEVELOPMENT ✅

**All requirements finalized, architecture updated, priorities clear.**  
**Developer can proceed with Phase 1A implementation immediately.**

**Estimated Timeline:** 9 weeks for Phase 1 completion (perfect for Q4 2025 target)

---

**Next Steps:**
1. Developer begins with VehicleParams & Bus structures
2. Weekly progress reviews with architecture team  
3. Phase 1B planning review after Week 3
4. Demo preparation starts Week 7