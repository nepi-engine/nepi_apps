#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import os
import time
import sys
import numpy as np
import cv2
import random
import copy
import threading



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls


from nepi_app_file_pub_depthmap.msg import FilePubDepthmapStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_interfaces.msg import NavPose

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ColorImageIF
from nepi_api.data_if import DepthMapIF
from nepi_api.data_if import ImageIF
from nepi_api.data_if import NavPoseIF
from nepi_api.system_if import ControlsIF


#########################################
# Controls
#########################################
#
# THREE control sets, because the page groups its rows under three headings and
# a control set renders as one flat list.
#
# A ControlsIF is ALWAYS a direct child of the NODE namespace: its __init__
# builds create_namespace(node_namespace, controls_name), there is no namespace
# argument, and nepi_utils.get_clean_name() rewrites '/' to '_' so the name
# cannot carry a path. The set NAME is therefore the only thing distinguishing
# one set from another, and the RUI derives the same three names.
#
# The third set is named navpose_SOURCE, not navpose, and that is load bearing.
# This node builds NavPoseIF(namespace = node_namespace), and NavPoseIF appends
# its own data_product when the basename is not already it (data_if.py), so that
# IF is rooted at <node>/navpose. A control set named 'navpose' would land on
# exactly that namespace and advertise a second <node>/navpose/status of a
# different message type. Getting this wrong is silent.

CONTROLS_NAME_PLAYBACK        = 'controls'
CONTROLS_NAME_FOLDER_SETTINGS = 'folder_settings'
CONTROLS_NAME_NAVPOSE         = 'navpose_source'

# Button controls, keyed here so the updated callback can tell a command press
# from a value edit without restating the names inline.
BUTTON_CONTROLS = ['start_pub', 'stop_pub', 'step_forward', 'step_backward']

# Row widths for the grouped lines.
_ROW_VALUE_WIDTH = 70
_ROW_FOV_WIDTH = 80

# Bounds for the operator-authored static pose. The node declares none of these:
# they are the WGS84 domain for the geopoint, the full turn for the three
# attitude angles, an operator sanity range for a local ENU offset, and a span
# from deep subsea to high-altitude flight for altitude and depth.
MIN_LATITUDE_DEG  = -90.0
MAX_LATITUDE_DEG  = 90.0
MIN_LONGITUDE_DEG = -180.0
MAX_LONGITUDE_DEG = 180.0
MIN_ANGLE_DEG     = -360.0
MAX_ANGLE_DEG     = 360.0
MIN_ALTITUDE_M    = -500.0
MAX_ALTITUDE_M    = 20000.0
MIN_DEPTH_M       = 0.0
MAX_DEPTH_M       = 11000.0
MAX_POSITION_M    = 100000.0

# nepi_controls clamps 'round' to 6 decimal places, so a latitude control holds
# about 0.11 m of resolution.
GEO_ROUND_PLACES = 6

# Static pose value controls: (control name, param/dict suffix, label, bounds).
# The control name IS the param name the app has always used, and the navpose
# dict key is that name without the navpose_static_ prefix -- so one table drives
# the control set, the route into setNavPoseStaticValueCb, and the dict write.
NAVPOSE_STATIC_VALUE_CONTROLS = (
    ('navpose_static_latitude',    'Latitude (deg)',    [MIN_LATITUDE_DEG, MAX_LATITUDE_DEG],   GEO_ROUND_PLACES),
    ('navpose_static_longitude',   'Longitude (deg)',   [MIN_LONGITUDE_DEG, MAX_LONGITUDE_DEG], GEO_ROUND_PLACES),
    ('navpose_static_altitude_m',  'Altitude (m)',      [MIN_ALTITUDE_M, MAX_ALTITUDE_M],       2),
    ('navpose_static_depth_m',     'Depth (m)',         [MIN_DEPTH_M, MAX_DEPTH_M],             2),
    ('navpose_static_heading_deg', 'Heading (deg)',     [MIN_ANGLE_DEG, MAX_ANGLE_DEG],         2),
    ('navpose_static_roll_deg',    'Roll (deg)',        [MIN_ANGLE_DEG, MAX_ANGLE_DEG],         2),
    ('navpose_static_pitch_deg',   'Pitch (deg)',       [MIN_ANGLE_DEG, MAX_ANGLE_DEG],         2),
    ('navpose_static_yaw_deg',     'Yaw (deg)',         [MIN_ANGLE_DEG, MAX_ANGLE_DEG],         2),
    ('navpose_static_x_m',         'Position X (m)',    [-MAX_POSITION_M, MAX_POSITION_M],      2),
    ('navpose_static_y_m',         'Position Y (m)',    [-MAX_POSITION_M, MAX_POSITION_M],      2),
    ('navpose_static_z_m',         'Position Z (m)',    [-MAX_POSITION_M, MAX_POSITION_M],      2),
)

# Static pose frame controls: (control name, navpose dict key, label).
NAVPOSE_STATIC_FRAME_CONTROLS = (
    ('navpose_static_frame_nav',      'frame_nav',      'Nav Frame'),
    ('navpose_static_frame_altitude', 'frame_altitude', 'Altitude Frame'),
    ('navpose_static_frame_depth',    'frame_depth',    'Depth Frame'),
)


class ControlValue:
  """Stand-in for the std_msgs value the app's setter callbacks expect.

  The setters kept their original signatures through this migration, so the
  control routes hand them an object with a .data attribute exactly as the ROS
  subscribers did. Nothing about their per-field clamping or frame validation
  moved.
  """

  def __init__(self, data):
    self.data = data


def build_playback_controls_init_dict(min_rate, max_rate, factory_rate):
  """Build the playback control set.

  Key order is display order: create_controls_dict iterates the init dict and
  the RUI renders controls_msg_list in that order. Controls sharing a non-empty
  display_group render on ONE horizontal line, in that same order.

  Args:
      min_rate: lowest publish rate in Hz.
      max_rate: highest publish rate in Hz.
      factory_rate: factory publish rate in Hz.

  Returns:
      dict: control-name -> init dict, in display order.
  """
  return {

    'start_pub': {
        'type': 'Button',
        'display_name': 'Start Publishing', 'display_group': 'publish',
        'description': 'Start publishing collections from the current folder'},

    'stop_pub': {
        'type': 'Button',
        'display_name': 'Stop Publishing', 'display_group': 'publish',
        'description': 'Stop publishing and release the data publishers'},

    # One row: the pause toggle, the rate box and the random toggle. Rate and
    # Random hide while paused and the two step Buttons appear -- see
    # syncControlVisibility, which mirrors what the hand-written panel did.
    'paused': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Paused', 'display_group': 'playback',
        'description': 'Hold on the current collection instead of advancing'},

    'rate_hz': {
        'type': 'Float', 'default': float(factory_rate),
        'bounds': [min_rate, max_rate],
        'round': 2, 'display_round': 2,
        'display_name': 'Rate (Hz)', 'display_group': 'playback',
        'display_width': _ROW_VALUE_WIDTH,
        'description': 'Rate collections are published at while not paused'},

    'random': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Random', 'display_group': 'playback',
        'description': 'Pick the next collection at random rather than in order'},

    'step_forward': {
        'type': 'Button',
        'display_name': 'Forward', 'display_group': 'step',
        'display_hidden': True,
        'description': 'While paused, advance one collection'},

    'step_backward': {
        'type': 'Button',
        'display_name': 'Back', 'display_group': 'step',
        'display_hidden': True,
        'description': 'While paused, go back one collection'},

    'overlay': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Overlay Filename',
        'description': 'Draw the source filename on each published color image'},
  }


def build_folder_settings_controls_init_dict(min_fov, max_fov,
                                             factory_width, factory_height):
  """Build the field of view and folder settings control set.

  Args:
      min_fov: lowest field of view in degrees.
      max_fov: highest field of view in degrees.
      factory_width: factory horizontal field of view in degrees.
      factory_height: factory vertical field of view in degrees.

  Returns:
      dict: control-name -> init dict, in display order.
  """
  return {

    # One row. Both axes share the same degree bound, but they stay two Float
    # controls rather than one Floats control because the folder settings
    # sidecar sets them independently and each carries its own label.
    'width_deg': {
        'type': 'Float', 'default': float(factory_width),
        'bounds': [min_fov, max_fov],
        'round': 2, 'display_round': 2,
        'display_name': 'Width (deg)', 'display_group': 'fov',
        'display_width': _ROW_FOV_WIDTH,
        'description': 'Angular width the published products declare'},

    'height_deg': {
        'type': 'Float', 'default': float(factory_height),
        'bounds': [min_fov, max_fov],
        'round': 2, 'display_round': 2,
        'display_name': 'Height (deg)', 'display_group': 'fov',
        'display_width': _ROW_FOV_WIDTH,
        'description': 'Angular height the published products declare'},

    'apply_folder_settings': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Apply Folder Settings',
        'description': 'Read a collection folder settings sidecar and apply the field of view it declares'},
  }


def build_navpose_controls_init_dict(source_mode_options, factory_source_mode,
                                     min_timeout, max_timeout, factory_timeout,
                                     frame_nav_options, frame_altitude_options,
                                     frame_depth_options):
  """Build the NavPose source control set.

  Args:
      source_mode_options: the three navpose source modes.
      factory_source_mode: factory source mode.
      min_timeout: lowest system pose staleness window in seconds.
      max_timeout: highest system pose staleness window in seconds.
      factory_timeout: factory staleness window in seconds.
      frame_nav_options: nav frames NavPoseIF offers.
      frame_altitude_options: altitude frames NavPoseIF offers.
      frame_depth_options: depth frames NavPoseIF offers.

  Returns:
      dict: control-name -> init dict, in display order.
  """
  controls = {

    # Selection, not Menu: the stored value is the mode STRING the node compares
    # against NAVPOSE_SOURCE_MODE_OPTIONS, and a Menu value is an index that
    # would re-point if that list were ever reordered.
    'navpose_source_mode': {
        'type': 'Selection', 'default': factory_source_mode,
        'options': list(source_mode_options),
        'display_name': 'Source Mode',
        'description': 'Which pose is published: the system pose, a static one, or auto'},

    'navpose_system_timeout_sec': {
        'type': 'Float', 'default': float(factory_timeout),
        'bounds': [min_timeout, max_timeout],
        'round': 2, 'display_round': 1,
        'display_name': 'System Pose Timeout (s)',
        'description': 'A system pose older than this counts as unavailable, which is what makes auto fall back to static'},
  }

  for name, label, bounds, round_places in NAVPOSE_STATIC_VALUE_CONTROLS:
    controls[name] = {
        'type': 'Float', 'default': 0.0,
        'bounds': bounds,
        'round': round_places, 'display_round': round_places,
        'display_name': label,
        'description': 'Static pose ' + label}

  frame_options = {
      'navpose_static_frame_nav': frame_nav_options,
      'navpose_static_frame_altitude': frame_altitude_options,
      'navpose_static_frame_depth': frame_depth_options,
  }
  for name, dict_key, label in NAVPOSE_STATIC_FRAME_CONTROLS:
    options = list(frame_options[name])
    controls[name] = {
        'type': 'Selection', 'default': options[0] if len(options) > 0 else '',
        'options': options,
        'display_name': label,
        'description': 'Frame the static pose ' + label.lower() + ' is declared in'}

  return controls


#########################################
# Node Class
#########################################

class NepiFilePubDepthmapApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  # A depthmap collection is made of these three products, identified by
  # filename suffix.  The color_image carries its own timestamp; the
  # depth_map and depth_map_image share a timestamp.
  COLLECTION_TYPES = ['color_image', 'depth_map', 'depth_map_image']
  TYPE_SUFFIXES = {
      'depth_map_image': '-depth_map_image.png',
      'color_image': '-color_image.png',
      'depth_map': '-depth_map.npy'
  }

  SUPPORTED_FILE_TYPES = ['png', 'npy']

  #Set Initial Values
  MIN_RATE = 0.1
  MAX_RATE = 20
  FACTORY_PUB_RATE = 1.0

  UPDATER_DELAY_SEC = 1.0

  #############################
  ## Field of view
  #
  # The angular width and height the three published products declare.  Every
  # consumer that reasons about direction rather than distance -- nepi_app_obstacles
  # is the current one -- derives its per-pixel bearings from these two numbers, so
  # a value that does not match the camera that produced the data skews that
  # reasoning even when the depth values themselves are correct.
  #
  # The factory pair is the app's historical hardcoded pair, kept so behavior does
  # not move until an operator asks for it.  A collection folder can carry the
  # values its own sensor needs in a settings sidecar -- see FOLDER_SETTINGS_FILE.
  FACTORY_WIDTH_DEG = 100.0
  FACTORY_HEIGHT_DEG = 70.0
  MIN_FOV_DEG = 1.0
  MAX_FOV_DEG = 180.0

  #############################
  ## Collection folder settings sidecar
  #
  # A collection folder may carry one of these next to its data files.  It is how
  # a converted or captured set declares the settings it needs, so the operator
  # does not have to know them and this app does not have to know the set:
  #
  #     width_deg: 63.1
  #     height_deg: 49.5
  #     description: TUM RGB-D rgbd_dataset_freiburg2_pioneer_360
  #
  # Every key is optional.  A missing or unreadable file, or a key that is not a
  # usable number, leaves the current setting where it is and produces a warning.
  # Nothing here is ever written back to the folder.
  #
  # The name deliberately ends in neither -color_image.png, -depth_map.npy nor
  # -depth_map_image.png, which is the only reason it is safe to leave sitting in
  # a collection folder -- buildCollections() groups by walking the sorted file
  # list, so a stray file matching one of those three suffixes would resync the
  # grouping and corrupt every collection after it.  listFolderFiles() filters on
  # getFileType(), so this file is never even offered to the grouper.
  FOLDER_SETTINGS_FILE = 'nepi_collection_settings.yaml'
  FOLDER_SETTINGS_FOV_KEYS = ['width_deg','height_deg']
  FOLDER_SETTINGS_DESCRIPTION_KEY = 'description'

  #############################
  ## NavPose source
  #
  # This app publishes a navpose on <node>/navpose from one of two SOURCES.
  #
  #   'system' -- the pose published by navpose_mgr for its base frame, forwarded
  #               unchanged.  The frames arrive WITH that pose and are its
  #               author's declaration about it, so they are passed through as
  #               received.  The operator frame selection is IGNORED here.
  #   'static' -- a fixed pose this app authors from operator input.  The app is
  #               the author, so the operator declares which frames it is
  #               expressed in, and only here do the selected frames apply.
  #
  # NAVPOSE_SOURCE_MODE_AUTO prefers 'system' and falls back to 'static' when no
  # system pose has arrived inside the staleness window; the other two force one
  # source regardless.
  NAVPOSE_SOURCE_MODE_AUTO = 'auto'
  NAVPOSE_SOURCE_MODE_SYSTEM = 'system'
  NAVPOSE_SOURCE_MODE_STATIC = 'static'
  NAVPOSE_SOURCE_MODE_OPTIONS = [NAVPOSE_SOURCE_MODE_AUTO,
                                 NAVPOSE_SOURCE_MODE_SYSTEM,
                                 NAVPOSE_SOURCE_MODE_STATIC]

  # navpose_mgr publishes each of its frames at
  # <base_namespace>/navposes/<frame>/navpose as a nepi_interfaces/NavPose, and
  # its own base frame is named 'base_frame' (navpose_mgr.NAVPOSE_BASE_FRAME).
  # Same join device_if_idx makes for its reference-frame subscription.
  NAVPOSE_SYSTEM_FRAME = 'base_frame'
  NAVPOSE_SYSTEM_SUBFOLDER = 'navposes'
  NAVPOSE_SYSTEM_SUBTOPIC = 'navpose'

  # A system pose older than this counts as unavailable, which is what makes
  # 'auto' fall back to 'static'.  Three seconds is three missed ticks at
  # navpose_mgr's MIN_PUB_RATE of 1 Hz, so a briefly slow publisher does not
  # bounce the active mode, while a stopped one is noticed promptly.
  NAVPOSE_SYSTEM_TIMEOUT_SEC = 3.0
  MIN_NAVPOSE_SYSTEM_TIMEOUT_SEC = 1.0
  MAX_NAVPOSE_SYSTEM_TIMEOUT_SEC = 60.0

  # Steady republish rate for <node>/navpose.  Runs on its own timer so the topic
  # carries data whether or not a collection is being published.
  NAVPOSE_PUB_RATE_HZ = 5.0

  node_if = None
  controls_if = None
  folder_controls_if = None
  navpose_controls_if = None

  if os.path.exists(HOME_FOLDER + '/sample_data'):
    current_folder = HOME_FOLDER + '/sample_data'
  else:
    current_folder = HOME_FOLDER

  last_folder = ""
  current_folders = []
  current_collection = 'None'
  current_ind = 0
  collection_count = 0

  collections = []
  num_collections = 0

  navpose_if = None
  color_if = None
  depth_map_if = None
  depth_map_image_if = None

  paused = False
  oneshot_offset = 1

  width_deg = FACTORY_WIDTH_DEG
  height_deg = FACTORY_HEIGHT_DEG

  # Folder settings state.  folder_settings_found and folder_settings_status are
  # report-only: they say what the last apply attempt did, so an operator can see
  # whether the toggle changed anything.
  apply_folder_settings = False
  folder_settings_found = False
  folder_settings_status = 'No folder settings applied'

  random = False
  overlay = False
  rate = FACTORY_PUB_RATE
  running = False

  restart = False
  update_pub = False

  # Loaded data for the current collection
  cur_color_img = None
  cur_depth_map = None
  cur_depth_img = None
  cur_min_range_m = 0.0
  cur_max_range_m = 1.0
  data_lock = threading.Lock()

  # NavPose source state.  system_navpose_dict is the most recent pose received
  # from navpose_mgr, exactly as received; system_navpose_time is when it arrived
  # (not the pose's own timestamp), which is what the staleness window measures.
  navpose_source_mode = NAVPOSE_SOURCE_MODE_AUTO
  navpose_active_mode = NAVPOSE_SOURCE_MODE_STATIC
  navpose_system_timeout_sec = NAVPOSE_SYSTEM_TIMEOUT_SEC
  system_navpose_dict = None
  system_navpose_time = None
  navpose_lock = threading.Lock()

  # The operator-authored static pose, seeded from NavPoseIF.get_blank_navpose_dict()
  # in setupNavPoseSource() so it is well formed before any operator input.
  static_navpose_dict = None


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_depthmap" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting IF Initialization Processes")

    ##############################
    # Initialize Class Variables


    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
    }

    # Params Config Dict ####################
    # Every operator-adjustable value moved to one of the three ControlsIF sets,
    # each of which registers and persists its own param under its own
    # namespace. The two that stay are node-WRITTEN state rather than
    # operator-typed values: current_folder is set by the folder navigation
    # commands, and running is a side effect of start/stop that seeds the
    # restart on the next launch.
    self.PARAMS_DICT = {
        'current_folder': {
            'namespace': self.node_namespace,
            'factory_val': self.HOME_FOLDER
        },
        'running': {
            'namespace': self.node_namespace,
            'factory_val': False
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': FilePubDepthmapStatus,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    # What stays here are the COMMANDS and one inbound telemetry subscription.
    # Folder navigation carries a relative name plus a traversal verb, which a
    # control set -- where each value is written independently -- cannot express
    # atomically; start/stop are the app's programmatic publishing API, and the
    # Button controls call the same private methods these callbacks do.
    #
    # Removed here and now driven over the three control sets' update_control
    # topics: set_rate, set_random, set_overlay, pause_pub, step_forward,
    # step_backward, set_width_deg, set_height_deg, set_apply_folder_settings,
    # set_navpose_source_mode, set_navpose_system_timeout, the eleven
    # set_navpose_static_* topics and the three set_navpose_static_frame_*
    # topics.
    self.SUBS_DICT = {
        'select_folder': {
            'namespace': self.node_namespace,
            'topic': 'select_folder',
            'msg': String,
            'qsize': None,
            'callback': self.selectFolderCb,
            'callback_args': ()
        },
        'home_folder': {
            'namespace': self.node_namespace,
            'topic': 'home_folder',
            'msg': Empty,
            'qsize': None,
            'callback': self.homeFolderCb,
            'callback_args': ()
        },
        'back_folder': {
            'namespace': self.node_namespace,
            'topic': 'back_folder',
            'msg': Empty,
            'qsize': None,
            'callback': self.backFolderCb,
            'callback_args': ()
        },
        'start_pub': {
            'namespace': self.node_namespace,
            'topic': 'start_pub',
            'msg': Empty,
            'qsize': None,
            'callback': self.startPubCb,
            'callback_args': ()
        },
        'stop_pub': {
            'namespace': self.node_namespace,
            'topic': 'stop_pub',
            'msg': Empty,
            'qsize': None,
            'callback': self.stopPubCb,
            'callback_args': ()
        },

        # The system navpose, published by navpose_mgr for its base frame. This
        # is INBOUND telemetry, not operator state, so it stays a subscriber.
        # Same topic join device_if_idx makes for its reference-frame
        # subscription. The navpose_ key prefix keeps it domain-unique -- see the
        # 2026-07 DECISION LOG entry on registry keys.
        'navpose_system_sub': {
            'namespace': nepi_sdk.create_namespace(
                            nepi_sdk.create_namespace(self.base_namespace, self.NAVPOSE_SYSTEM_SUBFOLDER),
                            self.NAVPOSE_SYSTEM_FRAME),
            'topic': self.NAVPOSE_SYSTEM_SUBTOPIC,
            'msg': NavPose,
            'qsize': 1,
            'callback': self.systemNavPoseCb,
            'callback_args': ()
        },
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT
    )


    ##############################
    # Create the three synchronized data interfaces
    data_ns = self.node_namespace

    # ONE NavPose IF for all three data interfaces, built here and handed to each
    # of them, the way device_if_idx builds idx_navpose_if and hands it to every
    # data product IF it creates.  Component sub-topics are disabled for the same
    # reason device_if_idx disables them: navpose_mgr scans the ROS graph for
    # those component message types and would otherwise offer this app as a
    # selectable navpose source.
    #
    # This is not decoration -- it is what makes the data interfaces publish
    # their status messages at all.  When no navpose_if is supplied, DepthMapIF
    # and BaseImageIF each build a ConnectNavPoseIF on their OWN node_if
    # (data_if.py navpose blocks), and ConnectNodeIF registers the generic key
    # 'status_pub' on whatever node_if it is given (connect_node_if.py).
    # register_pubs() is a keyed dict.update(), so that registration overwrites
    # the data IF's own 'status_pub' entry: the DepthMapStatus / ImageStatus
    # publisher is orphaned (its topic stays advertised, so the product still
    # appears in a consumer's selector) and every publish_status() call after
    # that hands a DepthMapStatus to a ConnectIFStatus publisher, which raises
    # and is swallowed by node_if's throttled try/except.  No status reaches the
    # wire, so no consumer can connect.  See the 2026-07 DECISION LOG entry on
    # domain-unique registry keys for the general case.  Supplying the navpose IF
    # from outside skips that internal construction entirely -- which is exactly
    # why the ZED path has never hit this.
    self.navpose_if = NavPoseIF(namespace = data_ns,
                data_source_description = 'file',
                data_ref_description = 'source',
                pub_navpose = True,
                pub_location = False,
                pub_heading = False,
                pub_orientation = False,
                pub_position = False,
                pub_altitude = False,
                pub_depth = False,
                pub_pan_tilt = False,
                save_data_if = None,
                save_data_enabled = False,
                log_name = 'navpose',
                msg_if = self.msg_if
                )

    # Seed the static pose before anything can read it.  The navpose publish
    # timer started at the end of this method is the only reader.
    self.setupNavPoseSource()

    self.color_if = ColorImageIF(namespace = data_ns,
                data_product = 'color_image',
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
                navpose_if = self.navpose_if,
                log_name = 'color_image',
                msg_if = self.msg_if
                )
    self.color_if.wait_for_ready()
    self.color_if.unregister_pubs()
    self.color_if.set_image_callback('needs_update_callback', self.publish_collection)

    # pub_image = False so the depth map IF does NOT auto-generate a colorized
    # depth image; the pre-rendered depth_map_image from file is published instead.
    self.depth_map_if = DepthMapIF(namespace = data_ns,
                data_product = 'depth_map',
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
                pub_image = False,
                navpose_if = self.navpose_if,
                log_name = 'depth_map',
                msg_if = self.msg_if
                )
    self.depth_map_if.wait_for_ready()
    self.depth_map_if.unregister_pubs()
    self.depth_map_if.set_image_callback('needs_update_callback', self.publish_collection)

    # An image IS published for this depth map, just not by the depth map IF, so
    # the status flag consumers read has to say so. pub_image = False above only
    # suppresses the colorizing DepthMapImageIF; it must not be read as "this
    # depth map has no image", which is what a False img_pub_enabled tells every
    # consumer (nepi_app_obstacles getDepthMapImageTopic gates on exactly this
    # flag and reports 'None' without it, leaving the viewer unmounted).
    self.depth_map_if.set_image_pub_enabled(True)

    # ImageIF publishes the pre-rendered depth_map_image png directly (no
    # colorization / no processing), unlike DepthMapImageIF which would
    # colorize a raw depth array.
    #
    # Its namespace is the DEPTH MAP namespace, not the node namespace, so the
    # image lands at <node>/depth_map/depth_map_image -- one level under the
    # depth map, exactly where DepthMapIF's own DepthMapImageIF would put it and
    # exactly where every depth map consumer looks for it (nepi_app_obstacles
    # DEPTH_MAP_IMAGE_SUBTOPIC, nepi_app_stereo_cam DEPTH_IMAGE_SUBTOPIC).
    # Published as a sibling of the depth map, it is invisible to all of them.
    depth_map_ns = nepi_sdk.create_namespace(data_ns, 'depth_map')
    self.depth_map_image_if = ImageIF(namespace = depth_map_ns,
                data_product = 'depth_map_image',
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
                navpose_if = self.navpose_if,
                log_name = 'depth_map_image',
                msg_if = self.msg_if
                )
    self.depth_map_image_if.wait_for_ready()
    self.depth_map_image_if.unregister_pubs()
    self.depth_map_image_if.set_image_callback('needs_update_callback', self.publish_collection)

    ##############################
    # Controls. Mounted AFTER navpose_if, because the navpose set's three frame
    # Selections take their option lists from it, and before initCb below reads
    # app state from the sets. Each set is given no node_if and builds its own:
    # sharing one would merge the registries and a generic key would silently
    # orphan a sibling's publisher (2026-07 DECISION LOG).
    self.setupControls()

    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Start updater process


    nepi_sdk.start_timer_process(self.UPDATER_DELAY_SEC, self.updaterCb)
    nepi_sdk.start_timer_process(1, self.collectionPublishCb, oneshot = True)
    nepi_sdk.start_timer_process(1.0, self.statusPublishCb)
    # NavPose publishing is deliberately NOT chained off collectionPublishCb: the
    # navpose topic must carry data whether or not a collection is being published.
    nepi_sdk.start_timer_process(float(1) / self.NAVPOSE_PUB_RATE_HZ,
                                 self.publishNavPoseCb, oneshot = True)



    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")
    self.publish_status()
    # Spin forever (until object is detected)
    nepi_sdk.spin()
    ##############################

#######################
  ### App Config Functions



  def initCb(self,do_updates = False):
    # Runs twice at startup: once from NodeClassIF's init_configs, before any
    # ControlsIF exists, and once explicitly after setupControls. The first pass
    # falls back to the factory values, the second picks up whatever the config
    # manager restored.
    if self.node_if is not None:
      current_folder = self.node_if.get_param('current_folder')
      if os.path.exists(current_folder) == False:
        current_folder = self.HOME_FOLDER
      self.current_folder = current_folder
      self.restart = self.node_if.get_param('running')
    self.applyControls()
    self.syncControlVisibility()
    if do_updates == True:
      pass
    self.publish_status()

  def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Reseting")
      # Each ControlsIF owns its own config tier under its own namespace, so the
      # app level reset has to hand the reset down to all three or the controls
      # keep their current values while the rest of the app resets.
      for controls_if in self.allControlsIfs():
        try:
          controls_if.reset()
        except Exception as e:
          self.msg_if.pub_warn("File Pub Depthmap: controls reset failed: " + str(e))
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      for controls_if in self.allControlsIfs():
        try:
          controls_if.factory_reset()
        except Exception as e:
          self.msg_if.pub_warn("File Pub Depthmap: controls factory reset failed: " + str(e))
      self.initCb(do_updates = do_updates)


  #############################
  ## Controls

  def setupControls(self):
    # Three sets, one per page heading. All three share one updated callback and
    # one route table: a control name is unique across the three sets, so the
    # name alone says which value changed.
    self.controls_init_dicts = {
        CONTROLS_NAME_PLAYBACK: build_playback_controls_init_dict(
            self.MIN_RATE, self.MAX_RATE, self.FACTORY_PUB_RATE),
        CONTROLS_NAME_FOLDER_SETTINGS: build_folder_settings_controls_init_dict(
            self.MIN_FOV_DEG, self.MAX_FOV_DEG,
            self.FACTORY_WIDTH_DEG, self.FACTORY_HEIGHT_DEG),
        CONTROLS_NAME_NAVPOSE: build_navpose_controls_init_dict(
            self.NAVPOSE_SOURCE_MODE_OPTIONS, self.NAVPOSE_SOURCE_MODE_AUTO,
            self.MIN_NAVPOSE_SYSTEM_TIMEOUT_SEC, self.MAX_NAVPOSE_SYSTEM_TIMEOUT_SEC,
            self.NAVPOSE_SYSTEM_TIMEOUT_SEC,
            self.navpose_if.get_frame_nav_options(),
            self.navpose_if.get_frame_altitude_options(),
            self.navpose_if.get_frame_depth_options()),
    }
    self.controls_routes = self.controlRoutes()

    self.controls_if = self.buildControlsIf(
        CONTROLS_NAME_PLAYBACK, 'Playback Controls',
        'Publishing, playback and overlay settings')
    self.folder_controls_if = self.buildControlsIf(
        CONTROLS_NAME_FOLDER_SETTINGS, 'Field of View and Folder Settings',
        'Field of view the published products declare, and the folder settings sidecar')
    self.navpose_controls_if = self.buildControlsIf(
        CONTROLS_NAME_NAVPOSE, 'NavPose Source',
        'Which pose is published and the static pose this app authors')

  def buildControlsIf(self, controls_name, display_name, description):
    init_dict = self.controls_init_dicts[controls_name]
    self.checkControlsInitDict(controls_name, init_dict)
    try:
      controls_if = ControlsIF(
          controls_name = controls_name,
          controls_display_name = display_name,
          controls_description = description,
          controls_init_dict = init_dict,
          controls_updated_callback = self.controlsUpdatedCb,
          pub_status = True,
          save_params = True,
          msg_if = self.msg_if,
      )
      controls_if.wait_for_controls_ready(timeout = 10)
      return controls_if
    except Exception as e:
      # Same degrade-to-None contract the data interfaces already have in this
      # node: the app still publishes, it just loses that panel, because every
      # read goes through getControlValue and falls back to the factory value.
      self.msg_if.pub_warn("File Pub Depthmap: controls unavailable for " +
                           str(controls_name) + ": " + str(e))
      return None

  def checkControlsInitDict(self, controls_name, init_dict):
    # create_controls_dict drops a malformed control with a log warning rather
    # than raising, so a typo in an init dict costs one widget and nothing else
    # says so. Run it here first and name what went missing.
    try:
      controls_dict = nepi_controls.create_controls_dict(init_dict)
    except Exception as e:
      self.msg_if.pub_warn("File Pub Depthmap: could not validate controls init dict " +
                           str(controls_name) + ": " + str(e))
      return
    missing = [name for name in init_dict.keys() if name not in controls_dict.keys()]
    if len(missing) > 0:
      self.msg_if.pub_warn("File Pub Depthmap: controls dropped at registration in " +
                           str(controls_name) + ": " + str(missing))

  def allControlsIfs(self):
    return [c for c in [self.controls_if, self.folder_controls_if,
                        self.navpose_controls_if] if c is not None]

  def findControlsIf(self, control_name):
    # Which of the three sets owns this control. Looked up rather than captured
    # in the callback because ControlsIF takes its updated callback at
    # construction, before the IF it would have to close over exists.
    for controls_if in self.allControlsIfs():
      try:
        if control_name in controls_if.get_controls_dict():
          return controls_if
      except Exception:
        continue
    return None

  def controlRoutes(self):
    # control name -> the app callback that already owns that value. Routing
    # rather than reimplementing is what keeps setRateCb's clamp,
    # clampFovDeg's not-a-number fallback, setNavPoseSourceModeCb's option
    # check and setNavPoseStaticFrame's frame validation exactly where they were.
    routes = {
        'paused':   self.pausePubCb,
        'rate_hz':  self.setRateCb,
        'random':   self.setRandomCb,
        'overlay':  self.setOverlayCb,
        'width_deg':  self.setWidthDegCb,
        'height_deg': self.setHeightDegCb,
        'apply_folder_settings': self.setApplyFolderSettingsCb,
        'navpose_source_mode': self.setNavPoseSourceModeCb,
        'navpose_system_timeout_sec': self.setNavPoseSystemTimeoutCb,
        'navpose_static_frame_nav': self.setNavPoseStaticFrameNavCb,
        'navpose_static_frame_altitude': self.setNavPoseStaticFrameAltitudeCb,
        'navpose_static_frame_depth': self.setNavPoseStaticFrameDepthCb,
    }
    # The eleven static pose values share one callback, which takes the param
    # name as its second argument -- exactly the callback_args form the removed
    # subscribers used. The control name IS that param name.
    for name, label, bounds, round_places in NAVPOSE_STATIC_VALUE_CONTROLS:
      routes[name] = (lambda m, n = name: self.setNavPoseStaticValueCb(m, n))
    return routes

  def getControlValue(self, control_name, fallback = None):
    controls_if = self.findControlsIf(control_name)
    if controls_if is None:
      return fallback
    value = None
    try:
      value = controls_if.get_control_value(control_name)
    except Exception as e:
      self.msg_if.pub_warn("File Pub Depthmap: failed to read control " +
                           str(control_name) + ": " + str(e))
    if value is None:
      return fallback
    return value

  def setControlValue(self, control_name, value):
    controls_if = self.findControlsIf(control_name)
    if controls_if is None:
      return
    try:
      controls_if.set_control_value(control_name, value)
    except Exception as e:
      self.msg_if.pub_warn("File Pub Depthmap: failed to write control " +
                           str(control_name) + ": " + str(e))

  def setControlHidden(self, control_name, hidden):
    controls_if = self.findControlsIf(control_name)
    if controls_if is None:
      return
    try:
      controls_if.set_control_hidden(control_name, hidden)
    except Exception:
      pass

  def applyControls(self):
    # The single point where a control value becomes running app state. The
    # static pose values are NOT read here: they live in static_navpose_dict,
    # which initCb seeds through the same route table the updates use.
    self.paused = bool(self.getControlValue('paused', False))
    self.rate = float(self.getControlValue('rate_hz', self.FACTORY_PUB_RATE))
    self.random = bool(self.getControlValue('random', False))
    self.overlay = bool(self.getControlValue('overlay', False))
    self.width_deg = self.clampFovDeg(
        self.getControlValue('width_deg', self.FACTORY_WIDTH_DEG), self.FACTORY_WIDTH_DEG)
    self.height_deg = self.clampFovDeg(
        self.getControlValue('height_deg', self.FACTORY_HEIGHT_DEG), self.FACTORY_HEIGHT_DEG)
    self.apply_folder_settings = bool(self.getControlValue('apply_folder_settings', False))
    self.navpose_source_mode = str(self.getControlValue(
        'navpose_source_mode', self.NAVPOSE_SOURCE_MODE_AUTO))
    self.navpose_system_timeout_sec = float(self.getControlValue(
        'navpose_system_timeout_sec', self.NAVPOSE_SYSTEM_TIMEOUT_SEC))
    self.applyStaticNavPoseControls()

  def applyStaticNavPoseControls(self):
    # Copy the static pose controls into static_navpose_dict. The control name
    # is the param name, and the navpose dict key is that name without the
    # navpose_static_ prefix -- the same mapping the removed per-topic callbacks
    # used, so there is still no translation table.
    if self.static_navpose_dict is None:
      return
    self.navpose_lock.acquire()
    for name, label, bounds, round_places in NAVPOSE_STATIC_VALUE_CONTROLS:
      dict_key = name.replace('navpose_static_','')
      if dict_key in self.static_navpose_dict:
        value = self.getControlValue(name, None)
        if value is not None:
          self.static_navpose_dict[dict_key] = float(value)
    for name, dict_key, label in NAVPOSE_STATIC_FRAME_CONTROLS:
      value = self.getControlValue(name, None)
      if value is not None and value != '':
        self.static_navpose_dict[dict_key] = str(value)
    self.navpose_lock.release()

  def syncControlVisibility(self):
    # Mirrors what the hand-written panel did. Rate and Random are shown while
    # running forward, the two step Buttons while paused; the static pose fields
    # and their three frames are hidden while a SYSTEM pose is being forwarded,
    # because in that mode the node never reads them -- the RUI disabled them for
    # exactly this reason. Gated on the RESOLVED active mode, not the setting:
    # 'auto' says nothing on its own about which source is publishing.
    paused = (self.paused == True)
    self.setControlHidden('rate_hz', paused)
    self.setControlHidden('random', paused)
    self.setControlHidden('step_forward', paused == False)
    self.setControlHidden('step_backward', paused == False)

    forwarding = (self.getNavPoseActiveMode() == self.NAVPOSE_SOURCE_MODE_SYSTEM)
    for name, label, bounds, round_places in NAVPOSE_STATIC_VALUE_CONTROLS:
      self.setControlHidden(name, forwarding)
    for name, dict_key, label in NAVPOSE_STATIC_FRAME_CONTROLS:
      self.setControlHidden(name, forwarding)

  def controlsUpdatedCb(self, control_name):
    # Called by whichever ControlsIF owns the control, with the control name
    # AFTER its dict is updated and its status published. One callback for all
    # three sets: a control name is unique across them.
    if control_name == 'start_pub':
      self.startPub()
    elif control_name == 'stop_pub':
      self.stopPub()
    elif control_name == 'step_forward':
      self.stepForwardPubCb(None)
    elif control_name == 'step_backward':
      self.stepBackwardPubCb(None)
    else:
      route = self.controls_routes.get(control_name, None)
      if route is not None:
        value = self.getControlValue(control_name)
        if value is not None:
          route(ControlValue(value))

    # applyControls runs after the route so an in-callback clamp (setRateCb,
    # clampFovDeg) or rejection (an unknown source mode, an unoffered frame) is
    # what lands in app state, then visibility is re-derived from it.
    self.applyControls()
    self.syncControlVisibility()

    # Matches what the removed set_param calls did: persist on change. The
    # config IF debounces this onto its own timer, so a slider drag does not
    # write a file per frame.
    if control_name not in BUTTON_CONTROLS and self.node_if is not None:
      self.node_if.save_config()

    self.publish_status()






  #############################
  ## APP callbacks

  def selectFolderCb(self,msg):
    current_folder = self.current_folder
    new_folder_name = msg.data
    new_folder = os.path.join(current_folder,new_folder_name)
    self.updateFolderInfo(new_folder)
    if os.path.exists(new_folder):
      if self.node_if is not None:
        self.node_if.set_param('current_folder',new_folder)
    self.publish_status()


  def homeFolderCb(self,msg):
    self.current_folder = self.HOME_FOLDER
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param('current_folder',self.HOME_FOLDER)


  def backFolderCb(self,msg):
    current_folder = self.node_if.get_param('current_folder')
    if current_folder != self.HOME_FOLDER:
      new_folder = os.path.dirname(current_folder )
      self.updateFolderInfo(new_folder)
      if os.path.exists(new_folder):
        if self.node_if is not None:
          self.node_if.set_param('current_folder',new_folder)
      self.publish_status()


  def pausePubCb(self,msg):
    ##self.msg_if.pub_info(msg)
    self.paused = msg.data
    self.oneshot_offset = 0
    self.update_pub = True
    self.publish_status()

  def stepForwardPubCb(self,msg):
    if self.paused:
      self.oneshot_offset = 1

  def stepBackwardPubCb(self,msg):
    if self.paused:
      self.oneshot_offset = -1



  #############################
  ## Publish control callbacks

  def setRandomCb(self,msg):
    ##self.msg_if.pub_info(msg)
    self.random = msg.data
    self.publish_status()


  def setOverlayCb(self,msg):
      ##self.msg_if.pub_info(msg)
      overlay = msg.data
      self.overlay = overlay
      self.publish_status()

  def setRateCb(self,msg):
    ##self.msg_if.pub_info(msg)
    rate = msg.data
    if rate < self.MIN_RATE:
      rate = self.MIN_RATE
    if rate > self.MAX_RATE:
      rate = self.MAX_RATE
    self.rate = rate
    self.publish_status()


  #############################
  ## Field of view

  def clampFovDeg(self, value, fallback):
    # Same clamp form setRateCb uses, with one addition: a value that is not a
    # usable number at all falls back rather than clamping, because that is what
    # a malformed sidecar or a hand-edited param file produces and there is no
    # sensible edge of the range to pin it to.
    try:
      fov_deg = float(value)
    except Exception:
      self.msg_if.pub_warn("Rejected field of view value: " + str(value) +
                           " ; not a number, keeping " + str(fallback))
      return fallback
    if fov_deg != fov_deg or fov_deg in [float('inf'), float('-inf')]:
      self.msg_if.pub_warn("Rejected field of view value: " + str(value) +
                           " ; not finite, keeping " + str(fallback))
      return fallback
    if fov_deg < self.MIN_FOV_DEG:
      self.msg_if.pub_warn("Clamped field of view " + str(fov_deg) +
                           " up to " + str(self.MIN_FOV_DEG))
      fov_deg = self.MIN_FOV_DEG
    if fov_deg > self.MAX_FOV_DEG:
      self.msg_if.pub_warn("Clamped field of view " + str(fov_deg) +
                           " down to " + str(self.MAX_FOV_DEG))
      fov_deg = self.MAX_FOV_DEG
    return fov_deg

  def setWidthDeg(self, width_deg):
    # The control persists the value; this only clamps it into app state. Note
    # it does NOT write the control back -- applyFolderSettings does that, once,
    # after both axes are set, which is what keeps the update recursion at
    # depth two.
    self.width_deg = self.clampFovDeg(width_deg, self.width_deg)

  def setHeightDeg(self, height_deg):
    self.height_deg = self.clampFovDeg(height_deg, self.height_deg)

  def setWidthDegCb(self,msg):
    self.setWidthDeg(msg.data)
    self.publish_status()

  def setHeightDegCb(self,msg):
    self.setHeightDeg(msg.data)
    self.publish_status()


  #############################
  ## Collection folder settings

  def readFolderSettings(self, folder):
    # Returns the sidecar dict for a folder, or None when there is no usable one.
    # nepi_utils.read_yaml_2_dict() logs and returns an empty dict for both a
    # missing and an unreadable file, so neither case can raise from here; an
    # empty YAML document loads as None, which is why that is checked too.
    settings_file = os.path.join(folder, self.FOLDER_SETTINGS_FILE)
    if os.path.exists(settings_file) == False:
      return None
    settings_dict = nepi_utils.read_yaml_2_dict(settings_file)
    if isinstance(settings_dict, dict) == False or len(settings_dict) == 0:
      self.msg_if.pub_warn("Unusable folder settings file " + str(settings_file) +
                           " ; keeping current settings")
      return None
    return settings_dict

  def applyFolderSettings(self, folder):
    # Reads the current folder's sidecar and applies what it carries.  Called on
    # every folder change and when the toggle is switched on.  Only ever called
    # with apply_folder_settings True -- the check lives in the callers so that a
    # disabled toggle never touches the filesystem at all.
    settings_dict = self.readFolderSettings(folder)
    if settings_dict is None:
      self.folder_settings_found = False
      self.folder_settings_status = ('No ' + self.FOLDER_SETTINGS_FILE +
                                     ' in this folder; settings unchanged')
      self.msg_if.pub_info(self.folder_settings_status)
      return

    self.folder_settings_found = True
    applied = []
    for key in self.FOLDER_SETTINGS_FOV_KEYS:
      if key not in settings_dict:
        continue
      # Each value goes through the same clamp the set topics use, so a sidecar
      # cannot put a value on the wire that an operator could not.
      if key == 'width_deg':
        self.setWidthDeg(settings_dict[key])
        applied.append('width_deg ' + str(self.width_deg))
      else:
        self.setHeightDeg(settings_dict[key])
        applied.append('height_deg ' + str(self.height_deg))

    # Push what the sidecar set back into the controls, or the RUI keeps showing
    # the previous field of view next to products already published with the new
    # one. set_control_value only fires the updated callback when the value
    # actually changed, and the route it then takes -- setWidthDegCb ->
    # setWidthDeg -- writes no control of its own, so this terminates at depth
    # two.
    self.setControlValue('width_deg', self.width_deg)
    self.setControlValue('height_deg', self.height_deg)

    description = settings_dict.get(self.FOLDER_SETTINGS_DESCRIPTION_KEY, '')
    if len(applied) == 0:
      self.folder_settings_status = ('Found ' + self.FOLDER_SETTINGS_FILE +
                                     ' but it set no known settings')
    else:
      self.folder_settings_status = 'Applied ' + ', '.join(applied)
      if description != '':
        self.folder_settings_status += ' from ' + str(description)
    self.msg_if.pub_info(self.folder_settings_status)

  def clearFolderSettingsStatus(self):
    # What the report fields say while the toggle is off.  found is False because
    # nothing was looked for, not because nothing is there.
    self.folder_settings_found = False
    self.folder_settings_status = ('Folder settings off; ' +
                                   self.FOLDER_SETTINGS_FILE + ' not read')

  def setApplyFolderSettingsCb(self,msg):
    apply_settings = msg.data
    self.apply_folder_settings = apply_settings
    if apply_settings == True:
      # Applied immediately on the current folder, not deferred to the next
      # folder change, so switching the toggle on has a visible effect.
      self.applyFolderSettings(self.current_folder)
    else:
      self.clearFolderSettingsStatus()
    self.publish_status()


  #############################
  ## NavPose source

  def setupNavPoseSource(self):
    # Seed the static pose from the IF's own blank navpose dict so it is well
    # formed before the operator has set anything.  The blank dict's frame_depth
    # is 'MSL', which is not one of NavPoseIF.get_frame_depth_options(), so the
    # three static frames are seeded from the option lists instead -- a value the
    # operator can actually re-select from the RUI dropdowns.
    static_dict = self.navpose_if.get_blank_navpose_dict()
    static_dict['navpose_frame'] = self.node_name
    static_dict['navpose_description'] = 'static operator authored pose'
    static_dict['frame_nav'] = self.navpose_if.get_frame_nav_options()[0]
    static_dict['frame_altitude'] = self.navpose_if.get_frame_altitude_options()[0]
    static_dict['frame_depth'] = self.navpose_if.get_frame_depth_options()[0]
    # Every component is reported as present so a consumer sees a complete pose.
    # A static pose is a declaration that these values hold, not a measurement
    # that may be missing.
    for has_key in ['has_location','has_heading','has_orientation',
                    'has_position','has_altitude','has_depth']:
      static_dict[has_key] = True
    self.static_navpose_dict = static_dict

  def systemNavPoseCb(self,msg):
    navpose_dict = nepi_nav.convert_navpose_msg2dict(msg)
    if navpose_dict is None:
      return
    self.navpose_lock.acquire()
    self.system_navpose_dict = navpose_dict
    self.system_navpose_time = nepi_utils.get_time()
    self.navpose_lock.release()

  def getSystemNavPose(self):
    # Returns the most recent system pose, or None when none has arrived inside
    # the staleness window.  The returned dict carries the frames its AUTHOR set;
    # nothing here rewrites them.
    self.navpose_lock.acquire()
    navpose_dict = copy.deepcopy(self.system_navpose_dict)
    navpose_time = self.system_navpose_time
    self.navpose_lock.release()
    if navpose_dict is None or navpose_time is None:
      return None
    if (nepi_utils.get_time() - navpose_time) > self.navpose_system_timeout_sec:
      return None
    return navpose_dict

  def getSystemNavPoseAvailable(self):
    return self.getSystemNavPose() is not None

  def setNavPoseSystemTimeoutCb(self,msg):
    timeout_sec = msg.data
    if timeout_sec < self.MIN_NAVPOSE_SYSTEM_TIMEOUT_SEC:
      timeout_sec = self.MIN_NAVPOSE_SYSTEM_TIMEOUT_SEC
    if timeout_sec > self.MAX_NAVPOSE_SYSTEM_TIMEOUT_SEC:
      timeout_sec = self.MAX_NAVPOSE_SYSTEM_TIMEOUT_SEC
    self.navpose_system_timeout_sec = timeout_sec
    self.publish_status()

  def setNavPoseSourceModeCb(self,msg):
    mode = msg.data
    if mode not in self.NAVPOSE_SOURCE_MODE_OPTIONS:
      self.msg_if.pub_warn("Rejected navpose source mode: " + str(mode) +
                           " ; not one of " + str(self.NAVPOSE_SOURCE_MODE_OPTIONS))
      return
    self.navpose_source_mode = mode
    self.publish_status()

  def setNavPoseStaticValueCb(self,msg,args):
    # args is the CONTROL name; the navpose dict key is the same name without
    # the navpose_static_ prefix, which is why the controls are named that way.
    # The control itself persists the value, so the set_param this used to do is
    # gone.
    param_name = args
    dict_key = param_name.replace('navpose_static_','')
    value = float(msg.data)
    self.navpose_lock.acquire()
    if self.static_navpose_dict is not None and dict_key in self.static_navpose_dict:
      self.static_navpose_dict[dict_key] = value
    self.navpose_lock.release()
    self.publish_status()

  def setNavPoseStaticFrame(self, frame, dict_key, param_name, frame_options):
    # Reject anything the IF does not offer and leave the previous value in
    # place, so a bad frame never reaches a published pose. The control itself
    # persists the value, so the set_param this used to do is gone; param_name
    # is kept in the signature because it names the control and reads in the
    # three callers.
    if frame not in frame_options:
      self.msg_if.pub_warn("Rejected static navpose " + str(dict_key) + ": " + str(frame) +
                           " ; not one of " + str(frame_options))
      return
    self.navpose_lock.acquire()
    if self.static_navpose_dict is not None:
      self.static_navpose_dict[dict_key] = frame
    self.navpose_lock.release()
    self.publish_status()

  def setNavPoseStaticFrameNavCb(self,msg):
    self.setNavPoseStaticFrame(msg.data, 'frame_nav', 'navpose_static_frame_nav',
                               self.navpose_if.get_frame_nav_options())

  def setNavPoseStaticFrameAltitudeCb(self,msg):
    self.setNavPoseStaticFrame(msg.data, 'frame_altitude', 'navpose_static_frame_altitude',
                               self.navpose_if.get_frame_altitude_options())

  def setNavPoseStaticFrameDepthCb(self,msg):
    self.setNavPoseStaticFrame(msg.data, 'frame_depth', 'navpose_static_frame_depth',
                               self.navpose_if.get_frame_depth_options())

  def getNavPoseActiveMode(self):
    # 'auto' prefers the system source and falls back to static; the other two
    # force one source.  A forced 'system' with no fresh pose publishes nothing
    # rather than silently authoring one, because the operator asked for the
    # system pose specifically.
    mode = self.navpose_source_mode
    if mode == self.NAVPOSE_SOURCE_MODE_SYSTEM:
      return self.NAVPOSE_SOURCE_MODE_SYSTEM
    if mode == self.NAVPOSE_SOURCE_MODE_STATIC:
      return self.NAVPOSE_SOURCE_MODE_STATIC
    if self.getSystemNavPoseAvailable() == True:
      return self.NAVPOSE_SOURCE_MODE_SYSTEM
    return self.NAVPOSE_SOURCE_MODE_STATIC

  def getStaticNavPose(self):
    self.navpose_lock.acquire()
    navpose_dict = copy.deepcopy(self.static_navpose_dict)
    self.navpose_lock.release()
    return navpose_dict

  def publishNavPoseCb(self,timer):
    # Runs on its own steady timer, independent of the collection publish path,
    # so <node>/navpose carries data whether or not a folder is being published
    # and a consumer can connect at any time.
    active_mode = self.getNavPoseActiveMode()
    navpose_dict = None
    if active_mode == self.NAVPOSE_SOURCE_MODE_SYSTEM:
      # FORWARDING.  The pose and its three frames were authored by navpose_mgr.
      # The operator's static frame selection is NOT read here and must never be:
      # a frame is a declaration about a pose, so only the pose's author may set
      # it.  getStaticNavPose() is not called on this branch.
      navpose_dict = self.getSystemNavPose()
    else:
      # AUTHORING.  This app is the author of this pose, so the operator's
      # frame_nav / frame_altitude / frame_depth selections apply -- and they
      # apply ONLY here, on the static branch.
      navpose_dict = self.getStaticNavPose()
      if navpose_dict is not None:
        pub_time = nepi_utils.get_time()
        for time_key in ['time_location','time_heading','time_orientation',
                         'time_position','time_altitude','time_depth']:
          navpose_dict[time_key] = pub_time

    if self.navpose_active_mode != active_mode:
      self.navpose_active_mode = active_mode
      self.publish_status()

    if navpose_dict is not None and self.navpose_if is not None:
      try:
        self.navpose_if.publish_navpose(navpose_dict)
      except Exception as e:
        self.msg_if.pub_warn("Failed to publish navpose: " + str(e))

    nepi_sdk.start_timer_process(float(1) / self.NAVPOSE_PUB_RATE_HZ,
                                 self.publishNavPoseCb, oneshot = True)


  #############################
  ## Collection grouping and folder navigation

  def getFileType(self, filepath):
    name = os.path.basename(filepath)
    for f_type in self.COLLECTION_TYPES:
      if name.endswith(self.TYPE_SUFFIXES[f_type]):
        return f_type
    return None

  def listFolderFiles(self, folder):
    files = []
    try:
      for name in sorted(os.listdir(folder)):
        full = os.path.join(folder, name)
        if os.path.isfile(full) and self.getFileType(full) is not None:
          files.append(full)
    except Exception as e:
      self.msg_if.pub_warn("Unable to list folder " + str(folder) + " : " + str(e))
    return sorted(files)

  def buildCollections(self, files):
    # Type-order resync grouping.  Walk the sorted file list accumulating one
    # file of each type into the current collection.  When all three types are
    # present the collection is complete.  If a type that is already filled
    # reappears before completion, the current collection is incomplete: drop
    # its files (with a warning) and start a fresh collection from that file.
    collections = []
    dropped = 0
    current = dict()
    for f in files:
      f_type = self.getFileType(f)
      if f_type is None:
        continue
      if f_type in current:
        dropped_files = [os.path.basename(current[t]) for t in current]
        self.msg_if.pub_warn("Dropping incomplete collection files: " + str(dropped_files))
        dropped += len(current)
        current = dict()
      current[f_type] = f
      if len(current) == len(self.COLLECTION_TYPES):
        collections.append(current)
        current = dict()
    if len(current) > 0:
      dropped_files = [os.path.basename(current[t]) for t in current]
      self.msg_if.pub_warn("Dropping trailing incomplete collection files: " + str(dropped_files))
      dropped += len(current)
    self.msg_if.pub_info("Found " + str(len(collections)) + " complete collections; dropped " + str(dropped) + " incomplete files")
    return collections, dropped

  def updateFolderInfo(self, folder):
    if folder != self.last_folder:
      self.stopPub()

      if os.path.exists(folder):
        self.current_folder = folder
        # Applied before startPub() below, so the first collection of a new
        # folder already goes out with that folder's field of view.
        if self.apply_folder_settings == True:
          self.applyFolderSettings(folder)
        else:
          self.clearFolderSettingsStatus()
        current_paths = nepi_utils.get_folder_list(folder)
        current_folders = []
        for path in current_paths:
          current_folders.append(os.path.basename(path))
        self.current_folders = sorted(current_folders)
        self.msg_if.pub_warn("Folders: " + str(self.current_folders))
        files = self.listFolderFiles(folder)
        collections, dropped = self.buildCollections(files)
        self.collection_count = len(collections)
        self.msg_if.pub_warn("Collection Count: " + str(self.collection_count))
        if self.collection_count > 0:
          self.startPub()
    self.last_folder = copy.deepcopy(self.current_folder)

  def updaterCb(self,timer):
    update_status = False
    # Get settings from param server
    current_folder = copy.deepcopy(self.current_folder)
    # Update folder info
    self.updateFolderInfo(current_folder)
    # Start publishing if needed
    restart = self.restart
    if restart == True:
      self.startPub()
      update_status = True
    self.restart = False
    # Publish status if needed
    if update_status == True:
      self.publish_status()


  #############################
  ## Publish lifecycle

  def registerPubs(self):
    for data_if in [self.color_if, self.depth_map_if, self.depth_map_image_if]:
      if data_if is not None:
        data_if.register_pubs()

  def unregisterPubs(self):
    for data_if in [self.color_if, self.depth_map_if, self.depth_map_image_if]:
      if data_if is not None:
        data_if.unregister_pubs()

  def startPubCb(self,msg):
    self.msg_if.pub_info('Got start publishing msg: ' + str(msg))
    self.startPub()

  def startPub(self):
    self.msg_if.pub_warn("Start Pub Called")

    current_folder = self.current_folder
    self.collections = []
    self.num_collections = 0
    if os.path.exists(current_folder):
      files = self.listFolderFiles(current_folder)
      collections, dropped = self.buildCollections(files)
      self.collections = collections
      self.num_collections = len(collections)
      self.collection_count = self.num_collections
      if self.num_collections > 0:
        self.msg_if.pub_warn("Registering data IF pubs")
        self.registerPubs()
        self.current_ind = 0
        self.running = True
        self.msg_if.pub_warn("Set Running to True")
        self.publish_status()
        if self.node_if is not None:
          self.node_if.set_param('running',True)
      else:
        self.msg_if.pub_info("No collections found in folder " + current_folder)
    else:
      self.msg_if.pub_info("Folder " + current_folder + " not found")
    self.publish_status()

  def stopPubCb(self,msg):
    self.stopPub()

  def stopPub(self):
    self.running = False
    self.data_lock.acquire()
    self.cur_color_img = None
    self.cur_depth_map = None
    self.cur_depth_img = None
    self.data_lock.release()
    self.current_collection = "None"
    self.publish_status()
    self.msg_if.pub_warn("Unregistering data IF pubs")
    self.unregisterPubs()
    if self.node_if is not None:
      self.node_if.set_param('running',False)


  #############################
  ## Collection load and synchronized publish

  def loadCollection(self, collection):
    color_file = collection.get('color_image')
    depth_map_file = collection.get('depth_map')
    depth_img_file = collection.get('depth_map_image')

    color_img = None
    depth_img = None
    depth_map = None
    min_range_m = 0.0
    max_range_m = 1.0

    if color_file is not None:
      color_img = cv2.imread(color_file)
    if depth_img_file is not None:
      depth_img = cv2.imread(depth_img_file)
    if depth_map_file is not None:
      try:
        depth_map = np.load(depth_map_file)
        finite = depth_map[np.isfinite(depth_map)]
        if finite.size > 0:
          min_range_m = float(np.min(finite))
          max_range_m = float(np.max(finite))
      except Exception as e:
        self.msg_if.pub_warn("Failed to load depth map " + str(depth_map_file) + " : " + str(e))

    # Overlay the collection name on the color image if enabled
    if self.overlay == True and color_img is not None:
      height, width = color_img.shape[0:2]
      font = cv2.FONT_HERSHEY_DUPLEX
      fontScale, thickness = nepi_img.get_optimal_font_dims(color_img, font_scale = 1.5e-3, thickness_scale = 1.5e-3)
      fontColor = (0, 255, 0)
      lineType = 1
      text2overlay = os.path.basename(color_file)
      bottomLeftCornerOfText = (int(width*.05), int(height*.1))
      cv2.putText(color_img, text2overlay,
          bottomLeftCornerOfText,
          font,
          fontScale,
          fontColor,
          thickness,
          lineType)

    self.data_lock.acquire()
    self.cur_color_img = color_img
    self.cur_depth_map = depth_map
    self.cur_depth_img = depth_img
    self.cur_min_range_m = min_range_m
    self.cur_max_range_m = max_range_m
    self.data_lock.release()

    if color_file is not None:
      self.current_collection = os.path.basename(color_file)
    elif depth_map_file is not None:
      self.current_collection = os.path.basename(depth_map_file)

  def collectionPublishCb(self,timer):
    running = self.running
    set_random = self.random
    oneshot_offset = copy.deepcopy(self.oneshot_offset)
    self.oneshot_offset = 0
    current_ind = copy.deepcopy(self.current_ind)

    if self.paused:
      step = oneshot_offset
    else:
      step = 1
    if running and (step != 0 or self.update_pub == True):
      self.update_pub = False
      if len(self.collections) > 0:
        # Set current index
        if set_random == True and self.paused == True and step != 0:
          current_ind = int(random.random() * self.num_collections)
        else:
          current_ind += step
        # Check ind bounds
        if current_ind > (len(self.collections)-1):
          current_ind = 0 # Start over
        elif current_ind < 0:
          current_ind = self.num_collections-1

        collection = self.collections[current_ind]
        self.current_ind = current_ind
        self.loadCollection(collection)
        self.publish_collection()

    delay = 0.1
    running = self.running
    if running == True and self.paused == False:
        delay = 1.0 / self.rate

    nepi_sdk.start_timer_process(delay, self.collectionPublishCb, oneshot = True)


  def publish_collection(self):
    """Publish the current collection's three products on a shared timestamp.

    A single timestamp value is generated for the tick and passed to all three
    publish calls (color image, raw depth map, and pre-rendered depth map image)
    so downstream consumers can associate the products of one collection.
    """
    self.data_lock.acquire()
    color_img = self.cur_color_img
    depth_map = self.cur_depth_map
    depth_img = self.cur_depth_img
    min_range_m = self.cur_min_range_m
    max_range_m = self.cur_max_range_m
    self.data_lock.release()

    # Single shared timestamp for all three products of this collection
    timestamp = nepi_utils.get_time()

    if color_img is not None and self.color_if is not None:
      self.color_if.publish_cv2_img(color_img,
                                    encoding = 'bgr8',
                                    timestamp = timestamp,
                                    width_deg = self.width_deg,
                                    height_deg = self.height_deg,
                                    pub_twice = self.paused)

    if depth_map is not None and self.depth_map_if is not None:
      self.depth_map_if.publish_np_depth_map(depth_map,
                                    timestamp = timestamp,
                                    width_deg = self.width_deg,
                                    height_deg = self.height_deg,
                                    min_range_m = min_range_m,
                                    max_range_m = max_range_m,
                                    pub_twice = self.paused)

    if depth_img is not None and self.depth_map_image_if is not None:
      self.depth_map_image_if.publish_cv2_img(depth_img,
                                    encoding = 'bgr8',
                                    timestamp = timestamp,
                                    width_deg = self.width_deg,
                                    height_deg = self.height_deg,
                                    pub_twice = self.paused)


  def statusPublishCb(self,timer):
    self.publish_status()

             ###################
  ## Status Publisher
  def publish_status(self):
    """Populate and publish the latched app status message."""
    status_msg = FilePubDepthmapStatus()

    status_msg.home_folder = self.HOME_FOLDER
    current_folder = self.current_folder
    status_msg.current_folder = current_folder
    if current_folder == self.HOME_FOLDER:
      selected_folder = 'Home'
    else:
      selected_folder = os.path.basename(current_folder)
    status_msg.selected_folder = selected_folder
    status_msg.current_folders = self.current_folders
    status_msg.supported_file_types = self.SUPPORTED_FILE_TYPES

    status_msg.collection_count = self.collection_count
    status_msg.current_collection = self.current_collection

    status_msg.paused = self.paused

    status_msg.set_random = self.random
    status_msg.set_overlay = self.overlay
    status_msg.min_max_rate = [self.MIN_RATE, self.MAX_RATE]
    status_msg.set_rate = self.rate
    status_msg.running = self.running

    status_msg.min_max_fov_deg = [self.MIN_FOV_DEG, self.MAX_FOV_DEG]
    status_msg.set_width_deg = self.width_deg
    status_msg.set_height_deg = self.height_deg

    status_msg.apply_folder_settings = self.apply_folder_settings
    status_msg.folder_settings_found = self.folder_settings_found
    status_msg.folder_settings_status = self.folder_settings_status

    # NavPose source. navpose_active_mode is the RESOLVED source, which is what
    # the RUI gates its static pose fields and frame dropdowns on -- an 'auto'
    # setting says nothing on its own about which source is publishing.
    status_msg.navpose_source_mode = self.navpose_source_mode
    status_msg.navpose_source_mode_options = self.NAVPOSE_SOURCE_MODE_OPTIONS
    status_msg.navpose_active_mode = self.navpose_active_mode
    status_msg.navpose_system_available = self.getSystemNavPoseAvailable()
    status_msg.navpose_system_timeout_sec = self.navpose_system_timeout_sec
    status_msg.navpose_pub_rate = self.NAVPOSE_PUB_RATE_HZ

    static_dict = self.getStaticNavPose()
    if static_dict is None:
      static_dict = dict()
    status_msg.navpose_static_latitude = static_dict.get('latitude', 0.0)
    status_msg.navpose_static_longitude = static_dict.get('longitude', 0.0)
    status_msg.navpose_static_heading_deg = static_dict.get('heading_deg', 0.0)
    status_msg.navpose_static_roll_deg = static_dict.get('roll_deg', 0.0)
    status_msg.navpose_static_pitch_deg = static_dict.get('pitch_deg', 0.0)
    status_msg.navpose_static_yaw_deg = static_dict.get('yaw_deg', 0.0)
    status_msg.navpose_static_x_m = static_dict.get('x_m', 0.0)
    status_msg.navpose_static_y_m = static_dict.get('y_m', 0.0)
    status_msg.navpose_static_z_m = static_dict.get('z_m', 0.0)
    status_msg.navpose_static_altitude_m = static_dict.get('altitude_m', 0.0)
    status_msg.navpose_static_depth_m = static_dict.get('depth_m', 0.0)

    status_msg.navpose_static_frame_nav = static_dict.get('frame_nav', '')
    status_msg.navpose_static_frame_altitude = static_dict.get('frame_altitude', '')
    status_msg.navpose_static_frame_depth = static_dict.get('frame_depth', '')
    if self.navpose_if is not None:
      status_msg.navpose_frame_nav_options = self.navpose_if.get_frame_nav_options()
      status_msg.navpose_frame_altitude_options = self.navpose_if.get_frame_altitude_options()
      status_msg.navpose_frame_depth_options = self.navpose_if.get_frame_depth_options()

    if self.node_if is not None:
      self.node_if.publish_pub('status_pub', status_msg)



  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info(" Shutting down: Executing script cleanup actions")
    for controls_if in self.allControlsIfs():
      try:
        controls_if.unregister()
      except Exception:
        pass
    self.controls_if = None
    self.folder_controls_if = None
    self.navpose_controls_if = None


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePubDepthmapApp()
