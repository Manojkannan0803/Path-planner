# CasADi Motion Primitive Generation Specification

**Date:** 2025-09-23  
**Target:** Motion Primitive Developer  
**Phase:** 1B Implementation (Weeks 4-6)  
**Reference:** Existing ACADO primitive library

---

## 1. Current ACADO System Analysis

### **Existing Structure Reference:**
```
Motion primitives library/
├── FINAL_MP_PP_100/           # Current ACADO primitives
│   ├── thetatrailer_0_gamma_*/ # Discrete heading angles
│   └── (Multiple gamma values: -30° to +30°)
├── Add_MP_toworkspace.m       # Primitive loader
└── MP_plotfile.m             # Visualization tool
```

### **Current ACADO Characteristics to Preserve:**
- **Heading Discretization:** 48 bins (7.5° increments, 0° to 352.5°)
- **Steering Angles:** γ ∈ {-30°, -27°, ..., +27°, +30°} (21 values)
- **Arc Lengths:** Variable based on curvature and endpoint requirements
- **Endpoint Accuracy:** ±0.15m position, ±3.75° heading (current tolerance)

### **ACADO Limitations to Address:**
1. **Fixed time horizons** → Need variable-length primitives
2. **Limited curvature variety** → Expand steering resolution
3. **No reverse primitives** → Add backward motion capabilities  
4. **Endpoint drift accumulation** → Improve terminal constraints

---

## 2. CasADi Python Generator Requirements

### **Core Optimization Problem:**
```python
# Vehicle Model (Bicycle/Ackermann)
dx/dt = v * cos(theta + beta)
dy/dt = v * sin(theta + beta) 
dtheta/dt = (v/L) * tan(delta)

# Where:
# L = 2.55m (wheelbase)
# beta = atan((lr/L) * tan(delta)) # rear axle slip angle
# lr = 1.53m (distance to rear axle)
# delta = steering angle [-44°, +44°]
```

### **Crucial Generation Considerations:**

#### **2.1 Vehicle-Specific Dynamics**
- **Minimum Turning Radius:** 5.3m (validates against δ_max = 44°)
- **Maximum Curvature:** κ_max = 1/5.3 = 0.189 m⁻¹
- **Wheelbase Integration:** Proper bicycle model (not point mass)
- **Center of Mass:** Account for vehicle dimensions in primitive endpoints

#### **2.2 Kinematic Feasibility**
- **Curvature Continuity:** C¹ continuous curvature (no instantaneous steering jumps)
- **Steering Rate Limits:** |dδ/dt| ≤ 35°/s (realistic actuator dynamics)
- **Velocity Profiles:** Coordinate steering with speed for stability
- **Path Smoothness:** C² continuous paths (jerk-bounded trajectories)

#### **2.3 Endpoint Precision**
- **Target Accuracy:** ±0.1m position, ±2° heading (improvement over ACADO)
- **Terminal Constraints:** Hard constraints on (x_f, y_f, θ_f)
- **Boundary Value Problem:** Two-point boundary value problem (TPBVP)
- **Convergence Criteria:** Newton-Raphson tolerance < 1e-6

#### **2.4 Computational Efficiency**
- **Offline Generation:** Pre-compute all primitives, store as lookup tables
- **Runtime Loading:** Fast memory-mapped access in MATLAB
- **Storage Format:** Binary .mat files with metadata
- **Compression:** Remove redundant/symmetric primitives

---

## 3. CasADi Implementation Architecture

### **3.1 Python Generation Script: `casadi_primitive_generator.py`**
```python
import casadi as ca
import numpy as np
from scipy.spatial.distance import cdist
import yaml

class MotionPrimitiveGenerator:
    def __init__(self, vehicle_params, generation_config):
        self.L = vehicle_params['wheelbase']  # 2.55m
        self.lr = vehicle_params['rear_axle_distance']  # 1.53m  
        self.delta_max = np.radians(vehicle_params['max_steering'])  # 44°
        
        # Discretization parameters
        self.theta_bins = generation_config['theta_bins']  # 48
        self.steering_resolution = generation_config['steering_resolution']  # 1°
        self.primitive_length = generation_config['primitive_length']  # 2-6m
        
    def generate_primitive_set(self):
        """Generate complete primitive library"""
        primitives = {}
        
        for start_theta in self.get_theta_discretization():
            for end_theta in self.get_theta_discretization():
                for steering_profile in self.get_steering_profiles():
                    primitive = self.solve_boundary_value_problem(
                        start_theta, end_theta, steering_profile
                    )
                    if self.validate_primitive(primitive):
                        key = self.get_primitive_key(start_theta, end_theta, primitive)
                        primitives[key] = primitive
                        
        return primitives
    
    def solve_boundary_value_problem(self, theta_start, theta_end, steering_profile):
        """CasADi optimal control problem"""
        # State variables: [x, y, theta]
        # Control: [delta, v] (steering angle, velocity)
        
        # Create CasADi optimization problem
        opti = ca.Opti()
        
        # Time discretization
        N = 50  # Control intervals
        T = opti.variable()  # Free end time
        dt = T / N
        
        # State trajectories
        X = opti.variable(3, N+1)  # [x, y, theta]
        U = opti.variable(2, N)    # [delta, v]
        
        # Vehicle dynamics constraints
        for k in range(N):
            x_k = X[:, k]
            u_k = U[:, k]
            
            # Bicycle model dynamics
            beta = ca.atan((self.lr/self.L) * ca.tan(u_k[0]))
            x_dot = u_k[1] * ca.cos(x_k[2] + beta)
            y_dot = u_k[1] * ca.sin(x_k[2] + beta)
            theta_dot = (u_k[1]/self.L) * ca.tan(u_k[0])
            
            # Euler integration
            x_next = x_k + ca.vertcat(x_dot, y_dot, theta_dot) * dt
            opti.subject_to(X[:, k+1] == x_next)
        
        # Boundary conditions
        opti.subject_to(X[0, 0] == 0)      # Start x = 0
        opti.subject_to(X[1, 0] == 0)      # Start y = 0  
        opti.subject_to(X[2, 0] == theta_start)
        
        # Terminal constraints (endpoint precision)
        opti.subject_to(X[2, -1] == theta_end)  # End heading
        
        # Physical constraints
        opti.subject_to(opti.bounded(-self.delta_max, U[0, :], self.delta_max))  # Steering
        opti.subject_to(opti.bounded(0.5, U[1, :], 8.0))  # Velocity bounds
        opti.subject_to(opti.bounded(1.0, T, 10.0))  # Time bounds
        
        # Objective: minimize path length + smoothness
        path_length = ca.sum1(U[1, :] * dt)
        steering_smoothness = ca.sum1(ca.diff(U[0, :]).T @ ca.diff(U[0, :]))
        
        opti.minimize(path_length + 0.1 * steering_smoothness)
        
        # Solver configuration
        opts = {'ipopt.print_level': 0, 'ipopt.tol': 1e-6}
        opti.solver('ipopt', opts)
        
        try:
            sol = opti.solve()
            return self.extract_primitive(sol, X, U, T)
        except:
            return None  # Failed to converge
```

### **3.2 Primitive Validation Framework:**
```python
def validate_primitive(self, primitive):
    """Comprehensive primitive validation"""
    checks = [
        self.check_curvature_bounds(primitive),
        self.check_endpoint_accuracy(primitive), 
        self.check_steering_rate_limits(primitive),
        self.check_collision_clearance(primitive),
        self.check_smoothness_criteria(primitive)
    ]
    return all(checks)

def check_curvature_bounds(self, primitive):
    """Validate maximum curvature constraint"""
    curvatures = self.compute_curvature_profile(primitive)
    return np.all(np.abs(curvatures) <= 0.189)  # 5.3m turning radius

def check_endpoint_accuracy(self, primitive):
    """Validate terminal constraint satisfaction"""
    pos_error = np.linalg.norm(primitive['endpoint'][:2] - primitive['target'][:2])
    angle_error = np.abs(primitive['endpoint'][2] - primitive['target'][2])
    return (pos_error < 0.1) and (angle_error < np.radians(2))
```

### **3.3 MATLAB Integration Bridge:**
```matlab
function primitiveLib = loadCasADiPrimitives(configFile)
    % Load CasADi-generated primitives into MATLAB workspace
    
    % Python interface
    pyrun("import sys; sys.path.append('./casadi_generator')");
    pyrun("from primitive_loader import PrimitiveLoader");
    
    % Load primitive data
    primitiveData = pyrunfile("load_primitives.py", "primitives", config=configFile);
    
    % Convert to MATLAB structure
    primitiveLib = struct();
    primitiveLib.theta_discretization = double(primitiveData{'theta_bins'});
    primitiveLib.primitives = cell(48, 48); % Start_theta x End_theta
    
    % Populate primitive matrix
    for i = 1:length(primitiveData{'primitive_list'})
        prim = primitiveData{'primitive_list'}{i};
        start_idx = prim{'start_theta_idx'} + 1;  % MATLAB 1-indexing
        end_idx = prim{'end_theta_idx'} + 1;
        
        primitiveLib.primitives{start_idx, end_idx} = struct(...
            'trajectory', double(prim{'trajectory'}), ...
            'curvature', double(prim{'curvature'}), ...
            'length', double(prim{'length'}), ...
            'cost', double(prim{'cost'}) ...
        );
    end
end
```

---

## 4. Critical Considerations During Generation

### **4.1 Numerical Stability**
- **Initialization Strategy:** Use ACADO primitives as warm-start for CasADi solver
- **Multiple Shooting:** Use direct multiple shooting for better convergence
- **Regularization:** Add small regularization terms to avoid singular solutions
- **Scaling:** Proper variable scaling (distances in meters, angles in radians)

### **4.2 Completeness Verification**
- **Reachability Analysis:** Ensure primitive set covers full state space
- **Coverage Metrics:** Quantify reachable set completeness  
- **Gap Detection:** Identify and fill coverage gaps in primitive library
- **Redundancy Removal:** Eliminate near-duplicate primitives

### **4.3 Quality Assurance**
- **ACADO Comparison:** Benchmark CasADi primitives against ACADO baseline
- **Path Length Optimality:** Verify primitives achieve near-optimal path lengths
- **Execution Time:** Ensure generation completes within reasonable time (< 2 hours)
- **Memory Footprint:** Validate primitive library size (< 50MB)

### **4.4 Integration Robustness**
- **Error Handling:** Graceful degradation when primitives unavailable
- **Fallback Strategy:** Use ACADO primitives if CasADi fails
- **Version Control:** Track primitive library versions and compatibility
- **Regression Testing:** Automated validation against test scenarios

---

## 5. Phase 1B Implementation Plan

### **Week 4: CasADi Setup & Basic Generation**
- Install CasADi Python environment
- Implement basic vehicle model and single primitive generation
- Validate against simple test cases (straight line, constant curvature)
- Create MATLAB-Python interface prototype

### **Week 5: Full Library Generation**  
- Generate complete primitive set (48×48 matrix)
- Implement validation framework and quality metrics
- Compare with ACADO baseline (path length, endpoint accuracy)
- Optimize generation performance and memory usage

### **Week 6: Integration & Testing**
- Integrate CasADi primitives into existing MATLAB path planner
- Create A/B testing framework (ACADO vs CasADi performance)
- Validate in realistic parking scenarios
- Document primitive library API and usage guidelines

---

## 6. Success Criteria

### **Technical Metrics:**
- **Generation Success Rate:** >95% primitive convergence
- **Endpoint Accuracy:** ±0.1m position, ±2° heading
- **Path Length Optimality:** <5% deviation from theoretical minimum
- **Library Completeness:** 100% state space coverage

### **Performance Metrics:**
- **Generation Time:** <2 hours for full library
- **Memory Usage:** <50MB for primitive storage
- **Runtime Loading:** <100ms for library initialization
- **Planning Quality:** ≥98% path success rate with CasADi primitives

### **Integration Validation:**
- **Backward Compatibility:** Seamless replacement of ACADO loader
- **API Consistency:** No changes to path planner interface
- **Robustness:** Graceful handling of edge cases and failures
- **Documentation:** Complete developer guide and troubleshooting

---

**✅ CasADi primitive generation provides the foundation for high-quality, vehicle-specific motion planning while preserving the proven ACADO reference architecture!**