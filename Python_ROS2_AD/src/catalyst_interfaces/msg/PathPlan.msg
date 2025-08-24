# Path plan result from A* algorithm
# Represents the complete path from start to goal

# Path information
VehicleState start_state
VehicleState goal_state
VehicleState[] path_states          # Complete path as vehicle states
MotionPrimitive[] motion_primitives  # Motion primitives used

# Planning statistics
float64 planning_time               # Time taken to plan [seconds]
int32 nodes_explored               # Number of nodes explored
float64 total_path_cost            # Total cost of path
float64 path_length                # Total path length [m]

# Status
bool success                       # Whether planning succeeded
string error_message               # Error description if failed
int32 algorithm_id                 # Which algorithm was used

# Metadata
builtin_interfaces/Time stamp
string frame_id
