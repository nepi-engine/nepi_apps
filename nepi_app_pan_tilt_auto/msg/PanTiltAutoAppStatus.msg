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



###################

bool pan_scanning
bool tilt_scanning

bool pan_tracking
bool tilt_tracking

bool pan_locked
bool tilt_locked

#################

bool scan_pan_enabled
bool sin_pan_enabled

bool scan_tilt_enabled
bool sin_tilt_enabled

float32 scan_pan_min_deg
float32 scan_pan_max_deg
float32 scan_tilt_min_deg
float32 scan_tilt_max_deg
float32 scan_speed_ratio

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

float32 track_move_ratio
float32 track_reset_time_sec

nepi_interfaces/TrackingStatus tracking_status



###############
string[] available_lock_source_namespaces
string[] lock_source_namespaces
string lock_source_namespace
bool lock_source_connected

bool lock_pan_enabled
bool lock_tilt_enabled

float32 lock_pan_min_deg
float32 lock_pan_max_deg
float32 lock_tilt_min_deg
float32 lock_tilt_max_deg


#########################

bool click_pan_enabled
bool click_tilt_enabled

########################
# Selected Image Topics
string[] image_topics
# Number of Windows to Dipslay
int32 num_windows
string[] image_priority_options


