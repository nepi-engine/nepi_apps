# Nav Sim App Status

bool nmea_sim_enabled
bool hnav_sim_enabled
bool nmea_connected
bool hnav_connected
int32 nmea_port
int32 hnav_port
# NMEA simulated nav (GPS + compass)
float64 nmea_latitude
float64 nmea_longitude
float32 nmea_altitude_m
float32 nmea_heading_deg
float32 nmea_speed_ms
# HNav simulated nav (full 6-DOF)
float64 hnav_latitude
float64 hnav_longitude
float32 hnav_altitude_m
float32 hnav_depth_m
float32 hnav_heading_deg
float32 hnav_roll_deg
float32 hnav_pitch_deg
float32 hnav_speed_ms
