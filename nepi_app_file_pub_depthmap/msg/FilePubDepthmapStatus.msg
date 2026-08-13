string home_folder
string current_folder
string selected_folder
string[] current_folders
string[] supported_file_types
int32 collection_count
string current_collection

bool paused

bool set_random
bool set_overlay
float32[] min_max_rate
float32 set_rate
bool running

#################
# Field of View
#################
# The angular width and height the three published products declare. Consumers
# that reason about direction rather than distance derive their per-pixel
# bearings from these, so they must match the sensor the data came from
float32[] min_max_fov_deg
float32 set_width_deg
float32 set_height_deg

#################
# Folder Settings
#################
# Operator setting. When true, the current folder's nepi_collection_settings.yaml
# is read and applied on every folder selection and the moment the toggle is
# switched on. When false the file is never read
bool apply_folder_settings
# Read-only. True when a usable settings file was found in the current folder.
# False while apply_folder_settings is off, because nothing was looked for
bool folder_settings_found
# Read-only. What the last apply attempt did, or why it did nothing
string folder_settings_status

#################
# NavPose Source
#################
# This app publishes a navpose on <node>/navpose from one of two SOURCES, and the
# distinction is what the two blocks below are for. In 'system' mode the pose is
# navpose_mgr's, forwarded unchanged, and its frames are its author's declaration
# about it. In 'static' mode this app authors the pose from the operator's values,
# so the operator also declares the frames it is expressed in.
#
# The static pose fields and the three static frames below therefore apply ONLY
# while navpose_active_mode is 'static'. The RUI renders both blocks disabled when
# it is 'system' -- see file_pub_depthmap_app_node.py publishNavPoseCb()

# Operator setting: 'auto', 'system' or 'static'. 'auto' prefers the system source
# and falls back to static; the other two force one source
string navpose_source_mode
# Available modes (published so the RUI menu always matches the node)
string[] navpose_source_mode_options
# The source actually publishing right now: 'system' or 'static'. This, not
# navpose_source_mode, is what the RUI gates its static fields on
string navpose_active_mode
# True when a system navpose arrived within navpose_system_timeout_sec. This is
# what makes 'auto' resolve to 'system'
bool navpose_system_available
# Staleness window, in seconds, for the flag above
float32 navpose_system_timeout_sec
# Steady rate <node>/navpose is republished at, independent of set_rate above
float32 navpose_pub_rate

#################
# Static NavPose
#################
# The pose this app authors in 'static' mode. Each field is one key of the NEPI
# navpose dict (nepi_nav.BLANK_NAVPOSE_DICT), set on <node>/set_navpose_static_*
float32 navpose_static_latitude
float32 navpose_static_longitude
float32 navpose_static_heading_deg
float32 navpose_static_roll_deg
float32 navpose_static_pitch_deg
float32 navpose_static_yaw_deg
float32 navpose_static_x_m
float32 navpose_static_y_m
float32 navpose_static_z_m
float32 navpose_static_altitude_m
float32 navpose_static_depth_m

#################
# Static NavPose Frames
#################
# The frames the STATIC pose is declared in. Never applied to a forwarded system
# pose. Each is validated against the matching option list below before it is
# accepted; a value not in the list is rejected and the previous one kept
string navpose_static_frame_nav
string navpose_static_frame_altitude
string navpose_static_frame_depth
# Frame options, read from NavPoseIF so the RUI dropdowns cannot drift from what
# the interface will actually accept
string[] navpose_frame_nav_options
string[] navpose_frame_altitude_options
string[] navpose_frame_depth_options
