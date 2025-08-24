# Motion primitive definition (converted from MATLAB mp structure)
# Corresponds to motion primitive structure in Pathplanning_Astar.m

# Motion primitive identification
float64 ti         # Theta initial [rad]
float64 gi         # Gamma initial [rad] 
float64 tf         # Theta final [rad]
float64 gf         # Gamma final [rad]

# Cost information
float64 gcost      # G-cost for this primitive
float64 hcost      # H-cost for this primitive
int32 direction    # Direction (0=forward, 1=reverse)

# Trajectory data
geometry_msgs/Point[] trajectory_points   # X,Y points along trajectory
float64[] theta_trajectory               # Orientation along trajectory
float64[] gamma_trajectory               # Articulation angle along trajectory
float64 path_length                      # Total length of primitive

# Metadata
string primitive_id                      # Unique identifier
builtin_interfaces/Time stamp
