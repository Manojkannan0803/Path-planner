# Plugin status information (TwinSim-inspired plugin monitoring)
string plugin_id                  # Unique plugin identifier
string plugin_name                # Human-readable plugin name
string plugin_type                # Type: "algorithm", "model", "sensor", etc.

# Status information
uint8 status                      # Status code
uint8 STATUS_UNLOADED = 0
uint8 STATUS_LOADING = 1  
uint8 STATUS_READY = 2
uint8 STATUS_RUNNING = 3
uint8 STATUS_ERROR = 4
uint8 STATUS_STOPPING = 5

string status_message             # Detailed status message
float64 cpu_usage                 # CPU usage percentage
float64 memory_usage              # Memory usage [MB]
float64 execution_time_avg        # Average execution time [ms]
float64 execution_time_max        # Maximum execution time [ms]

# Performance metrics
int64 total_executions            # Total number of executions
int64 successful_executions       # Successful executions
int64 failed_executions           # Failed executions

builtin_interfaces/Time last_execution
builtin_interfaces/Time stamp
