# nepi_apps — Developer Reference

## Purpose

`nepi_apps` is the application collection for the NEPI platform. Each app is an independent ROS package that adds a specific capability on top of the core engine. Apps are discovered and lifecycle-managed by `apps_mgr` in `nepi_engine`. The collection currently contains five apps covering automated hardware control, data publishing, visualization, and device management.

## Architecture

```
nepi_apps/
├── nepi_app_pan_tilt_auto/     # Automated pan-tilt scanning, tracking, and locking
├── nepi_app_file_pub_img/      # Publish images from filesystem to ROS
├── nepi_app_file_pub_vid/      # Publish video frames from filesystem to ROS
├── nepi_app_image_viewer/      # Multi-window ROS image topic viewer
└── nepi_app_onvif_mgr/         # ONVIF IP camera/PTZ discovery and management
```

Each app follows this internal structure:

```
nepi_app_{name}/
├── scripts/          # Python ROS node (main app logic)
├── msg/              # Custom ROS message definitions for this app
├── srv/              # Custom ROS service definitions (onvif_mgr only)
├── api/              # connect_app_*.py — external interface for other nodes
├── params/           # YAML: default parameters and app metadata (name, group, display name)
├── rui/              # React components for the RUI web interface
├── package.xml
├── CMakeLists.txt
├── LICENSE
└── deploy_nepi_app.sh
```

## How It Works

`apps_mgr` scans for installed apps on a 5-second polling interval, reads each app's `params/*.yaml` metadata file to get the app name, group, and display name, then launches the app's ROS node if it is enabled. Apps are enabled/disabled via the RUI or by direct ROS service calls to `apps_mgr`.

Each app node follows the same lifecycle:
1. `nepi_sdk.init_node()` initializes the ROS node
2. `NodeClassIF` (from `nepi_api`) loads parameters from the ROS param server
3. The app sets up publishers, subscribers, and service handlers
4. A 0.5–1.0 Hz timer callback publishes a latched status message containing all current state
5. All control input arrives via topic callbacks or service calls; no polling of external state

Apps are stateless across restarts unless they write to their YAML params file. The `config_mgr` persists the param server state to disk, which apps can use for save/restore.

### Apps in detail

**nepi_app_pan_tilt_auto** (group: AUTOMATION)
Provides autonomous scanning, tracking, and locking for pan-tilt (PTX) hardware. Scanning runs sinusoidal motion profiles over configurable pan/tilt ranges and speeds. Tracking subscribes to AI detection output or target messages and drives the pan-tilt to follow selected objects. Locking holds the pan-tilt at a pose derived from a base frame. All three modes are mutually exclusive.

**nepi_app_file_pub_img** (group: DATA)
Browses the filesystem for PNG/JPG images and publishes them as `sensor_msgs/Image` at a configurable rate (0.1–20 Hz). Supports resize, encoding selection (BGR8, RGB8, MONO8), text overlay, random selection, and step-forward/backward playback. Folder navigation commands arrive as ROS String topics.

**nepi_app_file_pub_vid** (group: DATA)
Same pattern as `file_pub_img` but for AVI video files. Extracts frames via OpenCV and publishes them. Includes frame-by-frame step control and reports FPS.

**nepi_app_image_viewer** (group: DATA)
Discovers all active `sensor_msgs/Image` topics via `nepi_sdk.find_topics_by_msg('Image')` and allows the RUI to select up to four topics to display simultaneously. The app itself subscribes to the selected topics and exposes them to the RUI frontend. Window count and topic selection are controlled via ROS String/Int32 topics.

**nepi_app_onvif_mgr** (group: SYSTEM)
Manages ONVIF-compliant IP cameras and PTZ devices. Uses WS-Discovery to find devices on the local network. Stores per-device configuration (UUID, credentials, IDX/PTX driver assignment) in YAML files at `/mnt/nepi_storage/user_cfg/`. Provides services for listing detected devices, updating/deleting device configs, querying available drivers, and synchronizing device clocks. Licensed under the Numurus Software License rather than BSD-3-Clause.

## ROS Interface

All app topics are rooted under the device namespace. Representative interfaces:

**nepi_app_pan_tilt_auto:**
- Published: `.../app_pan_tilt_auto/status` (`PanTiltAutoAppStatus`, latched)
- Subscribed: AI detection topics, target tracking topics (dynamic, configurable via status)

**nepi_app_file_pub_img:**
- Published: `.../app_file_pub_img/status` (`FilePubImgStatus`, latched); image data via `ColorImageIF`
- Subscribed: `.../app_file_pub_img/select_folder`, `.../home_folder`, `.../back_folder`, `.../set_size`, `.../set_encoding`, `.../set_rate`, `.../set_random`, `.../set_overlay`, `.../start_pub`, `.../stop_pub`, `.../pause_pub`, `.../step_forward`, `.../step_backward`

**nepi_app_file_pub_vid:**
- Same subscriber pattern as `file_pub_img` with AVI-specific controls
- Published: `.../app_file_pub_vid/status` (`FilePubVidStatus`, latched)

**nepi_app_image_viewer:**
- Published: `.../app_image_viewer/status` (`NepiAppImageViewerStatus`, latched)
- Subscribed: `.../app_image_viewer/set_topic_1` through `.../set_topic_4` (String), `.../set_num_windows` (Int32)

**nepi_app_onvif_mgr:**
- Published: `.../onvif_app/status` (`OnvifStatus`, latched)
- Subscribed: `.../onvif_app/allow_discovery_clearing` (Bool), `.../drivers_mgr/status` (`MgrDriversStatus`)
- Services: `.../onvif_app/device_list_query` (`OnvifDeviceListQuery`), `.../onvif_app/set_device_cfg` (`OnvifDeviceCfgUpdate`), `.../onvif_app/delete_device_cfg` (`OnvifDeviceCfgDelete`), `.../onvif_app/driver_list_query` (`OnvifDriverListQuery`), `.../onvif_app/resync_onvif_device_clocks` (`std_srvs/Empty`)

## Build and Dependencies

Built as part of the `nepi_engine_ws` catkin workspace. Each app declares its own `package.xml` with `nepi_sdk`, `nepi_api`, `nepi_interfaces`, `rospy`, and `std_msgs` as base dependencies. App-specific additions:

- `nepi_app_file_pub_img`, `nepi_app_file_pub_vid`: `cv2` (OpenCV), `numpy`, `open3d`
- `nepi_app_onvif_mgr`: `requests`, `wsdiscovery`, `xml.etree.ElementTree`

The `rui/` subdirectory in each app contains React components that are imported by `nepi_rui` at build time. Updating an app's UI requires rebuilding the RUI frontend (`npm run build` in `nepi_rui/src/rui_webserver/rui-app/`).

Each app's `deploy_nepi_app.sh` script handles installation to `/opt/nepi/` for production deployment.

## Naming Conventions

Follows the NEPI convention:
- **Public API methods:** `snake_case` with docstrings
- **Private/internal methods:** `_camelCase`, no docstrings
- **`Cb` suffix:** ROS callback; rename requires auditing all external call sites

## Known Constraints and Fragile Areas

**ONVIF manager cannot use topic-based subscribers for device discovery.** WS-Discovery uses multicast UDP and conflicts with the ROS network stack if mixed carelessly. The `onvif_mgr` uses only services and timer-based polling for its control interface. Using topic subscribers for ONVIF device control will break WS-Discovery. This is not obvious from the code structure and must be preserved.

**ONVIF config saving is manual.** `nepi_app_onvif_mgr` does not use the standard `SaveCfgIF` interface. It writes YAML files directly to `/mnt/nepi_storage/user_cfg/`. Changes to how `config_mgr` handles user config files do not automatically apply to the ONVIF manager.

**The `image_viewer` dynamic topic discovery is polling-based.** `nepi_sdk.find_topics_by_msg('Image')` is called on each status cycle. In systems with many image topics, this may introduce latency or inconsistency in topic lists shown to the user.

**App state is lost on node restart unless persisted.** Apps do not write config back to YAML automatically (except `onvif_mgr`). If `apps_mgr` restarts an app node, all runtime state resets to param server defaults. Use `config_mgr`'s save mechanism explicitly if persistence is needed.

**React UI components are tightly coupled to ROS topic names.** The RUI components for each app are hardcoded to subscribe to the exact topic names defined in the app's node. Renaming any published topic or status message field requires updating both the Python node and the corresponding React component.

## Decision Log

- 2026-03 — CLAUDE.md created — Initial developer reference, Claude Code authoring pass.
