## Pan and Tilt App status msg
## All data provided in the current reverse pan and tilt frames
## Reverse axis conversions are handled by the TMP IF class


string[] available_topics
string selected_topic
bool connected
string connected_topic

float32 scan_pan_min_deg
float32 scan_pan_max_deg
float32 scan_tilt_min_deg
float32 scan_tilt_max_deg
float32 scan_speed_ratio

bool has_auto_pan
bool auto_pan_enabled
bool sin_pan_enabled
bool track_pan_enabled

bool auto_pan_sec

bool has_auto_tilt
bool auto_tilt_enabled
bool sin_tilt_enabled
bool track_tilt_enabled

float32 auto_tilt_sec

string[] track_source_namespaces
string track_source_namespace
bool track_source_connected

#[pan_deg, tilt_deg], unset values are -999
float32[] current_position
float32 goto_position
float32 goto_min_error
float32 goto_error

bool click_pan_enabled
bool click_tilt_enabled

# Selected Image Topics
string[] image_topics
# Number of Windows to Dipslay
int32 num_windows

