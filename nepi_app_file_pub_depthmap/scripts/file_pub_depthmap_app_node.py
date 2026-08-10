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

  width_deg = 100
  height_deg = 70

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
    self.PARAMS_DICT = {
        'current_folder': {
            'namespace': self.node_namespace,
            'factory_val': self.HOME_FOLDER
        },
        'random': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'overlay': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'rate': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_PUB_RATE
        },
        'running': {
            'namespace': self.node_namespace,
            'factory_val': False
        },

        # NavPose source params.  Unlike pub/sub/service registry keys, a param's
        # ROS wire name IS namespace + key, so the navpose_ prefix here is both
        # the domain-unique registry key and the operator-visible param name.
        'navpose_source_mode': {
            'namespace': self.node_namespace,
            'factory_val': self.NAVPOSE_SOURCE_MODE_AUTO
        },
        'navpose_system_timeout_sec': {
            'namespace': self.node_namespace,
            'factory_val': self.NAVPOSE_SYSTEM_TIMEOUT_SEC
        },

        # The static pose the operator authors.  One param per navpose dict key
        # this app lets an operator set; the key names are the navpose dict key
        # names with the navpose_static_ prefix, so there is no translation table
        # between a param and the dict field it fills.
        'navpose_static_latitude': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_longitude': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_heading_deg': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_roll_deg': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_pitch_deg': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_yaw_deg': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_x_m': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_y_m': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_z_m': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_altitude_m': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },
        'navpose_static_depth_m': {
            'namespace': self.node_namespace,
            'factory_val': 0.0
        },

        # Frames the STATIC pose is declared in.  These are the operator's
        # declaration about a pose the operator authored, and they are applied
        # only when the resolved active mode is 'static' -- see publishNavPoseCb().
        'navpose_static_frame_nav': {
            'namespace': self.node_namespace,
            'factory_val': NavPoseIF.NAVPOSE_NAV_FRAME_OPTIONS[0]
        },
        'navpose_static_frame_altitude': {
            'namespace': self.node_namespace,
            'factory_val': NavPoseIF.NAVPOSE_ALT_FRAME_OPTIONS[0]
        },
        'navpose_static_frame_depth': {
            'namespace': self.node_namespace,
            'factory_val': NavPoseIF.NAVPOSE_DEPTH_FRAME_OPTIONS[0]
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
        'set_rate': {
            'namespace': self.node_namespace,
            'topic': 'set_rate',
            'msg': Float32,
            'qsize': None,
            'callback': self.setRateCb,
            'callback_args': ()
        },
        'set_random': {
            'namespace': self.node_namespace,
            'topic': 'set_random',
            'msg': Bool,
            'qsize': None,
            'callback': self.setRandomCb,
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
        'pause_pub': {
            'namespace': self.node_namespace,
            'topic': 'pause_pub',
            'msg': Bool,
            'qsize': None,
            'callback': self.pausePubCb,
            'callback_args': ()
        },
        'step_forward': {
            'namespace': self.node_namespace,
            'topic': 'step_forward',
            'msg': Empty,
            'qsize': None,
            'callback': self.stepForwardPubCb,
            'callback_args': ()
        },
        'step_backward': {
            'namespace': self.node_namespace,
            'topic': 'step_backward',
            'msg': Empty,
            'qsize': None,
            'callback': self.stepBackwardPubCb,
            'callback_args': ()
        },
        'set_overlay': {
            'namespace': self.node_namespace,
            'topic': 'set_overlay',
            'msg': Bool,
            'qsize': None,
            'callback': self.setOverlayCb,
            'callback_args': ()
        },

        #############################
        ## NavPose source subscribers
        #
        # Every key below is prefixed navpose_ so it cannot collide with an entry
        # this node or a sub-IF sharing its node_if already registered -- see the
        # 2026-07 DECISION LOG entry on domain-unique registry keys.  The ROS wire
        # name comes from namespace + topic, not from the key, so the topics read
        # set_navpose_* while the keys read navpose_set_*.

        # The system navpose, published by navpose_mgr for its base frame.  Same
        # topic join device_if_idx makes for its reference-frame subscription.
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
        'navpose_set_system_timeout': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_system_timeout',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseSystemTimeoutCb,
            'callback_args': ()
        },
        'navpose_set_source_mode': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_source_mode',
            'msg': String,
            'qsize': None,
            'callback': self.setNavPoseSourceModeCb,
            'callback_args': ()
        },

        # One topic per static pose value.  All eleven share setNavPoseStaticValueCb
        # and carry the param key in callback_args, the same callback_args form
        # node_if_ai_detector uses for its per-source data subscribers.
        'navpose_set_static_latitude': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_latitude',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_latitude')
        },
        'navpose_set_static_longitude': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_longitude',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_longitude')
        },
        'navpose_set_static_heading': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_heading',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_heading_deg')
        },
        'navpose_set_static_roll': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_roll',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_roll_deg')
        },
        'navpose_set_static_pitch': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_pitch',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_pitch_deg')
        },
        'navpose_set_static_yaw': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_yaw',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_yaw_deg')
        },
        'navpose_set_static_x': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_x',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_x_m')
        },
        'navpose_set_static_y': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_y',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_y_m')
        },
        'navpose_set_static_z': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_z',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_z_m')
        },
        'navpose_set_static_altitude': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_altitude',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_altitude_m')
        },
        'navpose_set_static_depth': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_depth',
            'msg': Float32,
            'qsize': None,
            'callback': self.setNavPoseStaticValueCb,
            'callback_args': ('navpose_static_depth_m')
        },

        # The three frames of the STATIC pose.  Each is validated against the
        # matching NavPoseIF option list before it is accepted.
        'navpose_set_static_frame_nav': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_frame_nav',
            'msg': String,
            'qsize': None,
            'callback': self.setNavPoseStaticFrameNavCb,
            'callback_args': ()
        },
        'navpose_set_static_frame_altitude': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_frame_altitude',
            'msg': String,
            'qsize': None,
            'callback': self.setNavPoseStaticFrameAltitudeCb,
            'callback_args': ()
        },
        'navpose_set_static_frame_depth': {
            'namespace': self.node_namespace,
            'topic': 'set_navpose_static_frame_depth',
            'msg': String,
            'qsize': None,
            'callback': self.setNavPoseStaticFrameDepthCb,
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
    if self.node_if is not None:
      current_folder = self.node_if.get_param('current_folder')
      if os.path.exists(current_folder) == False:
        current_folder = self.HOME_FOLDER
      self.current_folder = current_folder
      self.random = self.node_if.get_param('random')
      self.overlay = self.node_if.get_param('overlay')
      self.rate = self.node_if.get_param('rate')
      self.restart = self.node_if.get_param('running')

      self.navpose_source_mode = self.node_if.get_param('navpose_source_mode')
      self.navpose_system_timeout_sec = self.node_if.get_param('navpose_system_timeout_sec')
      # The static pose is only loaded once it has been seeded.  NodeClassIF can
      # call this back during its own construction, which is before
      # setupNavPoseSource() has run.
      if self.static_navpose_dict is not None:
        self.navpose_lock.acquire()
        for param_name in ['navpose_static_latitude','navpose_static_longitude',
                           'navpose_static_heading_deg',
                           'navpose_static_roll_deg','navpose_static_pitch_deg',
                           'navpose_static_yaw_deg',
                           'navpose_static_x_m','navpose_static_y_m','navpose_static_z_m',
                           'navpose_static_altitude_m','navpose_static_depth_m',
                           'navpose_static_frame_nav','navpose_static_frame_altitude',
                           'navpose_static_frame_depth']:
          dict_key = param_name.replace('navpose_static_','')
          if dict_key in self.static_navpose_dict:
            self.static_navpose_dict[dict_key] = self.node_if.get_param(param_name)
        self.navpose_lock.release()
    if do_updates == True:
      pass
    self.publish_status

  def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)




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
    if self.node_if is not None:
      self.node_if.set_param('random',msg.data)


  def setOverlayCb(self,msg):
      ##self.msg_if.pub_info(msg)
      overlay = msg.data
      self.overlay = overlay
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('overlay',overlay)

  def setRateCb(self,msg):
    ##self.msg_if.pub_info(msg)
    rate = msg.data
    if rate < self.MIN_RATE:
      rate = self.MIN_RATE
    if rate > self.MAX_RATE:
      rate = self.MAX_RATE
    self.rate = rate
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param('rate',rate)


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
    if self.node_if is not None:
      self.node_if.set_param('navpose_system_timeout_sec',timeout_sec)

  def setNavPoseSourceModeCb(self,msg):
    mode = msg.data
    if mode not in self.NAVPOSE_SOURCE_MODE_OPTIONS:
      self.msg_if.pub_warn("Rejected navpose source mode: " + str(mode) +
                           " ; not one of " + str(self.NAVPOSE_SOURCE_MODE_OPTIONS))
      return
    self.navpose_source_mode = mode
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param('navpose_source_mode',mode)

  def setNavPoseStaticValueCb(self,msg,args):
    # args is the param key; the navpose dict key is the same name without the
    # navpose_static_ prefix, which is why the params are named that way.
    param_name = args
    dict_key = param_name.replace('navpose_static_','')
    value = float(msg.data)
    self.navpose_lock.acquire()
    if self.static_navpose_dict is not None and dict_key in self.static_navpose_dict:
      self.static_navpose_dict[dict_key] = value
    self.navpose_lock.release()
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param(param_name,value)

  def setNavPoseStaticFrame(self, frame, dict_key, param_name, frame_options):
    # Reject anything the IF does not offer and leave the previous value in
    # place, so a bad frame never reaches a published pose.
    if frame not in frame_options:
      self.msg_if.pub_warn("Rejected static navpose " + str(dict_key) + ": " + str(frame) +
                           " ; not one of " + str(frame_options))
      return
    self.navpose_lock.acquire()
    if self.static_navpose_dict is not None:
      self.static_navpose_dict[dict_key] = frame
    self.navpose_lock.release()
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param(param_name,frame)

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
      fontScale, thickness = nepi_img.optimal_font_dims(color_img, font_scale = 1.5e-3, thickness_scale = 1.5e-3)
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


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePubDepthmapApp()
