# Vehicle state representation (converted from MATLAB state structure)
# Corresponds to the 'state' struct in Pathplanning_Astar.m

# Vehicle position
float64 x          # X coordinate [m]
float64 y          # Y coordinate [m]
float64 xa         # Actual ending point X [m] 
float64 ya         # Actual ending point Y [m]
float64 theta      # Vehicle orientation [rad]
float64 gamma      # Articulation angle [rad]

# Path planning data
int32 predxy       # Index of predecessor config in tree
float64 g_cost     # Cost so far (from g_cost.m)
float64 h_cost     # Heuristic cost (from h_cost.m)
float64 f_cost     # Total cost (g_cost + h_cost)

# Motion primitive info
MotionPrimitive predecessor   # Previous motion primitive
int32 direction    # Direction of motion (0=forward, 1=reverse)

# Timestamps
builtin_interfaces/Time stamp
