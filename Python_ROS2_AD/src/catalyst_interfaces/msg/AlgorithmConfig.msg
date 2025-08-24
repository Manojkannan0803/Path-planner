# Algorithm configuration (XX (work)-inspired plugin configuration)
# Allows runtime configuration of path planning algorithms

string algorithm_name              # Name of algorithm (e.g., "astar", "rrt_star")
string plugin_id                  # Unique plugin identifier

# A* specific parameters (converted from MATLAB hardcoded values)
float64 heuristic_weight          # Heuristic coefficient (2.5 from h_cost.m)
float64 penalty_factor            # Penalty for wrong orientation (1000 from h_cost.m)
bool use_rectangular_heuristic    # Whether to use rectangular heuristic

# General parameters
float64 max_planning_time         # Maximum planning time [seconds]
int32 max_iterations              # Maximum search iterations
float64 goal_tolerance            # Distance tolerance to goal [m]
float64 angle_tolerance           # Angular tolerance to goal [rad]

# Motion primitive settings
string motion_primitive_library   # Path to motion primitive files
bool enable_reverse_motion        # Allow reverse motion
float64 turning_radius_min        # Minimum turning radius [m]

# Plugin-specific parameters (key-value pairs for flexibility)
string[] parameter_names
string[] parameter_values

builtin_interfaces/Time stamp
