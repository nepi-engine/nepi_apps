# This message details the Nav Sim App Status Msg

# Simulator enable states
bool nmea_sim_enabled
bool hnav_sim_enabled
# Client connected to each TCP server
bool nmea_connected
bool hnav_connected
# Current simulated position (WGS84)
float64 latitude
float64 longitude
float32 altitude_m
float32 depth_m
# Current simulated orientation
float32 heading_deg
float32 roll_deg
float32 pitch_deg
# Dead-reckoning speed
float32 speed_ms
# TCP server ports
int32 nmea_port
int32 hnav_port
