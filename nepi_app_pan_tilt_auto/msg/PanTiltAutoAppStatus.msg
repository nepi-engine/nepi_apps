## Pan and Tilt App status msg
## All data provided in the current reverse pan and tilt frames
## Reverse axis conversions are handled by the TMP IF class

#####################
string[] available_pan_tilts
string selected_pan_tilt
bool pt_connected
string pt_connected_topic
nepi_interfaces/DevicePTXStatus pt_status_msg

#[pan_deg, tilt_deg], unset values are -999
float32[] current_position
float32[] goto_position


###################

bool pan_scanning
bool tilt_scanning

bool pan_tracking
bool tilt_tracking

bool pan_stabing
bool tilt_stabing

#################

bool scan_pan_enabled
bool scan_tilt_enabled

float32 scan_pan_min_deg
float32 scan_pan_max_deg
float32 scan_tilt_min_deg
float32 scan_tilt_max_deg

float32 scan_pan_sec
float32 scan_tilt_sec

###############

###################

bool track_pan_enabled
bool track_tilt_enabled

float32 track_pan_min_deg
float32 track_pan_max_deg
float32 track_tilt_min_deg
float32 track_tilt_max_deg

bool track_source_connected

nepi_interfaces/TrackingStatus tracking_status

float32 track_goal_deg
float32 track_move_deg
float32 track_move_ratio
float32 track_reset_time_sec

float32 track_pan_error
float32 track_tilt_error


###############
string[] available_stab_source_namespaces
string selected_stab_source
bool stab_source_connected

bool stab_pan_enabled
bool stab_tilt_enabled


#********

float32 stab_update_rate
float32 stab_num_avg
float32 stab_move_deg
float32 stab_move_ratio
float32 stab_reset_time_sec

float32 pan_speed_max
float32 pan_speed_dps
float32 tilt_speed_max
float32 tilt_speed_dps
float32 pantilt_avg_move_delay

float32 stab_roll_deg
float32 stab_roll_adj

float32 stab_pitch_deg
float32 stab_pitch_adj

float32 stab_heading_deg
float32 stab_heading_adj

float32 stab_pan_deg
float32 stab_pan_adj


float32 stab_tilt_deg
float32 stab_tilt_adj

#********

#########################

bool click_pan_enabled
bool click_tilt_enabled

########################
# Selected Image Topics
string[] image_topics
# Number of Windows to Dipslay
int32 num_windows
string[] image_priority_options


