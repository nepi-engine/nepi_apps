# Fake GPS App Status

# App enabled state (HilGPS injection on/off)
bool enabled

# Target mavros (mavlink) node selection
string[] available_mavros_nodes
string selected_mavros_node
bool mavros_connected

# Simulated GPS position (WGS84)
float64 current_latitude
float64 current_longitude
float64 current_altitude_m

# Configured start/home position (WGS84)
float64 start_latitude
float64 start_longitude
float64 start_altitude_m

# Simulated NED position relative to last reset (meters)
float32 current_point_x
float32 current_point_y
float32 current_point_z

# Simulated orientation (derived from direction of travel)
# current_heading_deg: degrees true north; current_yaw_deg: ENU yaw degrees
float32 current_heading_deg
float32 current_yaw_deg

# Active move state
bool moving

# Fake GPS publish settings
int32 satellites_visible
float32 gps_pub_rate_hz
