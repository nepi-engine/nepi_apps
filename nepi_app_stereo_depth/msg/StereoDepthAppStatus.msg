# This message details the Stereo Depth App Status Msg

# Available and selected camera image topics
string[] available_image_topics
string selected_left_topic
string selected_right_topic
bool left_connected
bool right_connected

# Frame pairing (two USB cameras are not hardware synchronized)
float32 sync_tolerance_s
float32 last_pair_dt_s

# Calibration state
bool calibrated
string calibration_file
float32 baseline_m
int32 image_width
int32 image_height
float32 last_rms_reproj_error

# Calibration capture mode: idle, capturing, computing
string calib_state
int32 capture_count
int32 capture_target
int32 checkerboard_cols
int32 checkerboard_rows
float32 checkerboard_square_m

# Disparity / StereoSGBM parameters
int32 min_disparity
int32 num_disparities
int32 block_size
int32 uniqueness_ratio
int32 speckle_window_size
int32 speckle_range

# Depth range clip (meters)
float32 min_range_m
float32 max_range_m

# Run state
bool running
string status_message
