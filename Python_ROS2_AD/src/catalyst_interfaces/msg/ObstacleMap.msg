# Obstacle map representation (converted from MATLAB DPDscenario.m)
# Represents static and dynamic obstacles in distribution center

# Environment dimensions
float64 length_dc    # Length of distribution center [m] (328m from MATLAB)
float64 width_dc     # Width of distribution center [m] (200m from MATLAB)
float64 scale        # Scale of the grid [m] (1m from MATLAB)

# Static obstacles (from DPDscenario.m)
geometry_msgs/Polygon[] static_obstacles

# Dynamic obstacles
geometry_msgs/Polygon[] dynamic_obstacles

# Grid-based representation for fast collision checking
int8[] occupancy_grid    # 0=free, 100=occupied, -1=unknown
int32 grid_width         # Grid width in cells
int32 grid_height        # Grid height in cells
float64 resolution       # Meters per cell

# Metadata
builtin_interfaces/Time stamp
string frame_id
