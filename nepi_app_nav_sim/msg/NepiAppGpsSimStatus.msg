# GPS Sim Instance Status
#
# What the instance REPORTS. Everything the operator ADJUSTS lives in this
# instance's control sets (gps_instances_<name>_{position,move,output}) and is
# read back from their own ControlsStatus, so it is deliberately not mirrored
# here.

# Master enable (GPS_INPUT injection and fix publishing on/off)
bool gps_sim_enabled

# Target mavros (mavlink) node. The discovered list is process-global and
# pushed down from the app node; the selection and the binding are per instance.
string[] available_mavros_nodes
string selected_mavros_node
bool mavros_connected

# Simulated GPS position (WGS84)
float64 current_latitude
float64 current_longitude
float64 current_altitude_m

# Simulated ENU position relative to the last reset (meters)
float32 current_point_x
float32 current_point_y
float32 current_point_z

# Simulated orientation, derived from the direction of travel.
# current_heading_deg: degrees true north; current_yaw_deg: ENU yaw degrees.
float32 current_heading_deg
float32 current_yaw_deg

# Active move state
bool moving

# Publish settings, reported read-only the way NepiAppFakeGpsStatus reported them
int32 satellites_visible
float32 gps_pub_rate_hz
