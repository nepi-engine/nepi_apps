## Pan and Tilt App status msg
## All data provided in the current reverse pan and tilt frames
## Reverse axis conversions are handled by the TMP IF class

#####################
string[] available_pan_tilts
string selected_pan_tilt
bool pt_connected
string pt_connected_topic

#[pan_deg, tilt_deg], unset values are -999
float32[] current_position
float32 goto_position
float32 goto_min_error
float32 goto_error

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

string[] track_available_sources
string track_selected_source
bool track_source_connected

string[] track_available_classes
string[] track_selected_classes

string[] track_available_filters
string[] track_selected_filter


bool track_pan_enabled
bool track_tilt_enabled

float32 track_pan_min_deg
float32 track_pan_max_deg
float32 track_tilt_min_deg
float32 track_tilt_max_deg

float32 track_reset_time_sec


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


