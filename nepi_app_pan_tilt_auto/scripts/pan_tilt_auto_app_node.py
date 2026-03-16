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
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
import os
import time
import sys
import numpy as np
import copy
import math
import threading


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_targets


from nepi_app_pan_tilt_auto.msg import PanTiltAutoAppStatus
from nepi_interfaces.msg import DevicePTXStatus, RangeWindow, ImageMouseEvent
from nepi_interfaces.msg import RangeWindow, Target, Targets, TargetingStatus


from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF


UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

TARGET_TOPIC_TIMEOUT_SEC = 2
TARGET_TRACK_TIMEOUT_SEC = 2

#########################################
# Node Class
#########################################

class NepiPanTiltAutoApp(object):

  DEFAULT_MIN_MAX_DEG = 50

  SCAN_SWITCH_DEG = 5 # If angle withing this bound, switch dir
  SCAN_UPDATE_INTERVAL = .5

  TRACK_MAX_UPDATE_RATE = 0.5
  TRACK_MIN_ERROR_DEG = 10
  TRACK_DEFAULT_SOURCE = 'targets'
  TRACK_SENSITIVITY = 0.6

  TRACK_RESET_TIME_SEC = 2
  TRACK_DEFAULT_TARGETS = ['person']

  TARGET_BEST_FILTER_OPTIONS = nepi_targets.TARGET_BEST_FILTER_OPTIONS
  TARGET_BEST_FILTER_DEFAULT = 'LARGEST'

  LOCK_MAX_UPDATE_RATE = 0.5
  LOCK_MIN_ERROR_DEG = 5
  LOCK_DEFAULT_SOURCE = 'base_frame'

  IMAGE_PRIORITY_OPTIONS = ['IMAGES','DETECTIONS','TARGETS']
  IMAGE_PRIORITY_NAMES = ['color_image','detection_image','target_image']

  #####################
  
  node_if = None
  process_needs_update = False
  status_msg = PanTiltAutoAppStatus() 
  status_update_rate = 1

  #####################

  available_pan_tilts = []
  pt_connect_if = None
  selected_pan_tilt = "None"
  connected_topic = None
  connected = False
  pt_status_msg = DevicePTXStatus()

  min_pan_softstop_deg = -DEFAULT_MIN_MAX_DEG
  max_pan_softstop_deg = DEFAULT_MIN_MAX_DEG
  min_tilt_softstop_deg = -DEFAULT_MIN_MAX_DEG
  max_tilt_softstop_deg = DEFAULT_MIN_MAX_DEG

  goto_position = [0,0]

  navpose_update_rate = 1
  status_update_rate = 1

  current_position = None

  #####################
  has_scan_pan = True
  has_scan_tilt = True
  has_sin_pan = False
  has_sin_tilt = False
  has_homing = False
  has_set_home = False

  pan_scanning = False
  tilt_scanning = False

  pan_tracking = False
  tilt_tracking = False

  pan_locked = False
  tilt_locked = False

  #####################

  scan_pan_enabled = False
  scan_pan_track_hold = False

  scan_pan_sec = 5

  scan_pan_last_time = None
  scan_pan_times = [0,0,0,0,0]
  scan_pan_time = 1
  sin_pan_enabled = False
  scan_pan_sin_ind = 0
  
  rpi = 1
  rti = 1

  scan_tilt_enabled = False
  scan_tilt_track_hold = False
  scan_tilt_sec = 5

  scan_tilt_last_time = None
  scan_tilt_times = [0,0,0,0,0]
  scan_tilt_time = 1
  sin_tilt_enabled = False
  scan_tilt_sin_ind = 0


  scan_pan_min_deg = -DEFAULT_MIN_MAX_DEG
  scan_pan_max_deg = DEFAULT_MIN_MAX_DEG
  scan_tilt_min_deg = -DEFAULT_MIN_MAX_DEG
  scan_tilt_max_deg = DEFAULT_MIN_MAX_DEG

  scan_pan_deg = 0
  scan_pan_speed = 0
  scan_pan_deg = 0
  scan_pan_speed = 0

  scan_tilt_sins = 0
  scan_tilt_sin_ind = 0
  scan_tilt_sins = 0
  scan_tilt_sin_ind = 0

  #####################
  targets_msg = None
  last_targets_time = 0
  
  track_ordered_list = TRACK_DEFAULT_TARGETS
  track_max_update_rate = TRACK_MAX_UPDATE_RATE
  track_best_filter = TARGET_BEST_FILTER_DEFAULT
  track_min_error_deg = TRACK_MIN_ERROR_DEG
  track_source_topic = TRACK_DEFAULT_SOURCE
  track_source_connected_namespace = "None"
  track_source_connected = False
  track_source_connecting = False
  last_track_pan_time = 0
  last_track_tilt_time = 0


  track_pan_enabled = False
  track_tilt_enabled = False

  track_pan_min_deg = -DEFAULT_MIN_MAX_DEG
  track_pan_max_deg = DEFAULT_MIN_MAX_DEG
  track_tilt_min_deg = -DEFAULT_MIN_MAX_DEG
  track_tilt_max_deg = DEFAULT_MIN_MAX_DEG

  track_reset_time_sec = TRACK_RESET_TIME_SEC

  track_source_namespace = 'None'
  track_last_namespace = 'None'
  track_if = None
  track_pan_dict = None
  track_tilt_dict = None

  track_num_avg = 1
  track_pan_error = 0
  track_pan_sensitivity = TRACK_SENSITIVITY
  track_tilt_error = 0
  track_tilt_sensitivity = TRACK_SENSITIVITY


  #####################
  navpose_msg = None
  last_navpose_time = 0
  
  lock_max_update_rate = LOCK_MAX_UPDATE_RATE
  lock_min_error_deg = LOCK_MIN_ERROR_DEG
  lock_source_topic = LOCK_DEFAULT_SOURCE
  lock_source_connected_namespace = "None"
  lock_source_connected = False
  lock_source_connecting = False
  last_lock_time = 0


  lock_pan_enabled = False
  lock_tilt_enabled = False

  lock_pan_min_deg = -DEFAULT_MIN_MAX_DEG
  lock_pan_max_deg = DEFAULT_MIN_MAX_DEG
  lock_tilt_min_deg = -DEFAULT_MIN_MAX_DEG
  lock_tilt_max_deg = DEFAULT_MIN_MAX_DEG

  lock_source_namespace = 'None'
  lock_last_namespace = 'None'
  lock_if = None
  lock_pan_dict = None
  lock_tilt_dict = None

  lock_num_avg = 1
  lock_pan_error = []
  lock_pan_sensitivity = 0.8
  lock_tilt_error = []
  lock_tilt_sensitivity = 0.8


  has_published = False

  ############

  click_pan_enabled = True
  click_tilt_enabled = True
  set_mouse_click = [0,0]



  #################
  ### Image Viewer
  #################
  FACTORY_SELECTED_PAN_TILTS = ["None","None","None","None"]

  update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ
 
  single_image_topic = "None"
  selected_image_topics = ["None","None","None","None"]
  num_windows = 1
  last_num_windows = 4

  image_priority_list = []
  image_priority_dict = dict()
  for i, option in enumerate(IMAGE_PRIORITY_OPTIONS):
     image_priority_dict[option] = IMAGE_PRIORITY_NAMES[i]

  available_image_topics = []  
  available_image_dict = dict()
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_pan_tilt_scan" # Can be overwitten by luanch command
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
    self.track_source_namespace = os.path.join(self.base_namespace,self.track_source_topic)

    self.scan_pan_times = [0,0,0,0,0]
    self.scan_tilt_times = [0,0,0,0,0]
    self.scan_pan_sins = []
    self.scan_pan_sin_ind = 0
    self.scan_tilt_sins = []
    self.scan_tilt_sin_ind = 0


    
    # SCAN SCANNING ##############
    # timed scan scanning is not supported yet


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
        'selected_pan_tilt': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_pan_tilt
        },
        'scan_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },          
        'scan_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },      
        'scan_pan_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_pan_min_deg
        },           
        'scan_pan_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_pan_max_deg
        },   
        'scan_tilt_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_tilt_min_deg
        },           
        'scan_tilt_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_tilt_max_deg
        },    

        'sin_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'sin_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },

        'track_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },  
        'track_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },   
        'track_pan_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.track_pan_min_deg
        },           
        'track_pan_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.track_pan_max_deg
        },   
        'track_tilt_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.track_tilt_min_deg
        },           
        'track_tilt_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.track_tilt_max_deg
        },          
        'track_reset_time_sec': {
            'namespace': self.node_namespace,
            'factory_val': self.track_reset_time_sec
        },      

        
        'lock_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },  
        'lock_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },          
        'lock_pan_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.lock_pan_min_deg
        },           
        'lock_pan_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.lock_pan_max_deg
        },   
        'lock_tilt_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.lock_tilt_min_deg
        },           
        'lock_tilt_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.lock_tilt_max_deg
        },  

        'click_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        }
        ,
        'click_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        #####################
        ###Image Viewer
        #####################
        'single_image_topic': {
            'namespace': self.node_namespace,
            'factory_val': self.single_image_topic
        },
        'selected_image_topics': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_image_topics
        },
        'num_windows': {
            'namespace': self.node_namespace,
            'factory_val': self.num_windows
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': PanTiltAutoAppStatus,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'select_pan_and_tilt': {
            'namespace': self.node_namespace,
            'topic': 'select_pt_device',
            'msg': String,
            'qsize': None,
            'callback': self.selectTopicCb, 
            'callback_args': ()
        },
        'stop_pan_tilt': {
            'namespace': self.node_namespace,
            'topic': 'stop_pan_tilt',
            'msg': Empty,
            'qsize': 1,
            'callback': self.stopPanTiltCb, 
            'callback_args': ()
        },

        'set_scan_pan': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setScanPanCb, 
            'callback_args': ()
        },
        'set_scan_tilt_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setScanTiltCb, 
            'callback_args': ()
        },
        'set_scan_pan_window': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_pan_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setScanPanWindowCb, 
            'callback_args': ()
        },
        'set_scan_tilt_window': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_tilt_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setScanTiltWindowCb, 
            'callback_args': ()
        },




        'set_track_pan_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_track_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackPanCb, 
            'callback_args': ()
        },
        'set_track_tilt_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_track_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackTiltCb, 
            'callback_args': ()
        },
        'set_track_pan_window': {
            'namespace': self.node_namespace,
            'topic': 'set_track_pan_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setTrackPanWindowCb, 
            'callback_args': ()
        },
        'set_track_tilt_window': {
            'namespace': self.node_namespace,
            'topic': 'set_track_tilt_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setTrackTiltWindowCb, 
            'callback_args': ()
        },
        'set_track_reset_time_sec': {
            'namespace': self.node_namespace,
            'topic': 'set_track_reset_time_sec',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTrackResetTimeSecCb, 
            'callback_args': ()
        },


        'set_lock_pan_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_lock_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setLockPanCb, 
            'callback_args': ()
        },
        'set_lock_tilt_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_lock_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setLockTiltCb, 
            'callback_args': ()
        },
        'set_lock_pan_window': {
            'namespace': self.node_namespace,
            'topic': 'set_lock_pan_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setLockPanWindowCb, 
            'callback_args': ()
        },
        'set_lock_tilt_window': {
            'namespace': self.node_namespace,
            'topic': 'set_lock_tilt_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setLockTiltWindowCb, 
            'callback_args': ()
        },


        'set_pan_click_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_pan_click_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setPanClickCb, 
            'callback_args': ()
        },
        'set_tilt_click_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_tilt_click_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setTiltClickCb, 
            'callback_args': ()
        },


        ######################
        ###Image Viewer
        ######################
        'set_mouse_click': {
            'namespace': self.node_namespace,
            'topic': 'set_mouse_click',
            'msg': ImageMouseEvent,
            'qsize': None,
            'callback': self.clickCb, 
            'callback_args': ()
        },
        'set_topic_1': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_1',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic1Cb, 
            'callback_args': ()
        },
        'set_topic_2': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_2',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic2Cb, 
            'callback_args': ()
        },
          'set_topic_3': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_3',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic3Cb, 
            'callback_args': ()
        },
          'set_topic_4': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_4',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic4Cb, 
            'callback_args': ()
        },
          'set_num_windows': {
            'namespace': self.node_namespace,
            'topic': 'set_num_windows',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setNumWindowsCb, 
            'callback_args': ()
        },
          'set_image_priority': {
            'namespace': self.node_namespace,
            'topic': 'set_image_priority',
            'msg': String,
            'qsize': 10,
            'callback': self.setImagePriorityCb, 
            'callback_args': ()
        }
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT
    )

    nepi_sdk.wait()
    


    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Start updater process
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

    self.msg_if.pub_warn("Starting status pub")
    nepi_sdk.start_timer_process(1.0, self.publishStatusCb)
    nepi_sdk.start_timer_process(0.1, self.connectTrackSourceCb, oneshot = True)
    #nepi_sdk.start_timer_process(self.update_image_subs_interval_sec, self.updateImageSubsThread)

    self.msg_if.pub_warn("has_scan_pan: " + str(self.has_scan_pan))
    if self.has_scan_pan:
        # Start Scan Pan Process
        self.msg_if.pub_info("Starting scan pan scanning process")
        nepi_sdk.start_timer_process(self.SCAN_UPDATE_INTERVAL, self.scanPanProcess)
        nepi_sdk.start_timer_process(self.track_max_update_rate, self.trackPanProcess)


    if self.has_scan_tilt:
        # Start Scan Pan Process
        self.msg_if.pub_info("Starting scan tilt scanning process")
        nepi_sdk.start_timer_process(self.SCAN_UPDATE_INTERVAL, self.scanTiltProcess)
        nepi_sdk.start_timer_process(self.track_max_update_rate, self.trackTiltProcess)



    #self.msg_if.pub_warn("supports_sin_scan: " + str(self.supports_sin_scan))
    #if self.supports_sin_scan:
        #self.msg_if.pub_warn("Starting sin scanning process")
        #nepi_sdk.start_timer_process(.5, self.scanPanSinProcess, oneshot = True)
        #nepi_sdk.start_timer_process(.5, self.scanTiltSinProcess, oneshot = True)

    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")

    # Spin forever (until object is detected)
    nepi_sdk.spin()
    ##############################

#######################
  ### App Config Functions



  def initCb(self,do_updates = False):
    if self.node_if is not None:

        
      self.selected_pan_tilt = self.node_if.get_param('selected_pan_tilt')

      scan_pan_enabled = self.node_if.get_param('scan_pan_enabled')
      scan_tilt_enabled = self.node_if.get_param('scan_tilt_enabled')
      self.scan_pan_min_deg = self.node_if.get_param('scan_pan_min_deg')
      self.scan_pan_max_deg = self.node_if.get_param('scan_pan_max_deg')
      self.scan_tilt_min_deg = self.node_if.get_param('scan_tilt_min_deg')
      self.scan_tilt_max_deg = self.node_if.get_param('scan_tilt_max_deg')
      self.setScanPan(scan_pan_enabled)
      self.setScanTilt(scan_tilt_enabled)

      track_pan_enabled = self.node_if.get_param('track_pan_enabled')
      track_tilt_enabled = self.node_if.get_param('track_tilt_enabled')
      self.track_pan_min_deg = self.node_if.get_param('track_pan_min_deg')
      self.track_pan_max_deg = self.node_if.get_param('track_pan_max_deg')
      self.track_tilt_min_deg = self.node_if.get_param('track_tilt_min_deg')
      self.track_tilt_max_deg = self.node_if.get_param('track_tilt_max_deg')
      self.track_reset_time_sec = self.node_if.get_param('track_reset_time_sec')
      self.setTrackPan(track_pan_enabled)
      self.setTrackTilt(track_tilt_enabled)

      lock_pan_enabled = self.node_if.get_param('lock_pan_enabled')
      lock_tilt_enabled = self.node_if.get_param('lock_tilt_enabled')
      self.lock_pan_min_deg = self.node_if.get_param('lock_pan_min_deg')
      self.lock_pan_max_deg = self.node_if.get_param('lock_pan_max_deg')
      self.lock_tilt_min_deg = self.node_if.get_param('lock_tilt_min_deg')
      self.lock_tilt_max_deg = self.node_if.get_param('lock_tilt_max_deg')
      self.setLockPan(lock_pan_enabled)
      self.setLockTilt(lock_tilt_enabled)

      self.num_windows = self.node_if.get_param('num_windows')
      self.selected_image_topics = self.node_if.get_param('selected_image_topics')
      self.single_image_topic = self.node_if.get_param('single_image_topic')

    if do_updates == True:
      pass
    self.publish_status()

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


  def imageUpdateCb(self):
    self.img_needs_update = True


  def updaterCb(self,timer):
    #self.msg_if.pub_warn("Updater Called")
    if self.pt_connect_if is not None:
      current_position = self.pt_connect_if.get_pan_tilt_position()
      #self.msg_if.pub_warn("current_position: " + str(current_position))

    selected_pan_tilt = copy.deepcopy(self.selected_pan_tilt)
    last_available = copy.deepcopy(self.available_pan_tilts)
    needs_publish = False

    
    ##############
    topics = nepi_sdk.find_topics_by_msg('DevicePTXStatus')
    available_pan_tilts = []
    for topic in topics:
      available_pan_tilts.append(topic.replace('/status',''))
    if available_pan_tilts != last_available:
      self.available_pan_tilts = available_pan_tilts
      needs_publish = True

    ####################
    if self.connected_topic is not None:
      if self.connected_topic not in self.available_pan_tilts:
        success = self.unsubscribe_pt_topic()
    if selected_pan_tilt == 'None' and len(self.available_pan_tilts) > 0:
        self.selected_pan_tilt = self.available_pan_tilts[0]
    needs_publish = True

    if self.selected_pan_tilt in self.available_pan_tilts and self.connected_topic != selected_pan_tilt:
      success = self.subscribe_pt_topic(self.selected_pan_tilt)

    elif self.pt_connect_if is not None:
       self.connected = self.pt_connect_if.check_connection()
       needs_publish = True
    else:
       self.connected = False

    ######################
    topics = nepi_sdk.find_topics_by_msg('Image')
    available_image_topics = []
    image_priority_list = []
    for topic in topics:
      available_image_topics.append(topic)   
      image_name = os.path.basename(topic)
      for priority_option in self.image_priority_dict.keys(): 
         priority_name = self.image_priority_dict[priority_option]
         if priority_name == image_name and priority_option not in image_priority_list:
            image_priority_list.append(priority_option)
    self.available_image_topics = available_image_topics
    self.image_priority_list = image_priority_list
      
    ##################
    # Get settings from param server
    if needs_publish == True:
      self.publish_status()
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


  def get_image_priority_topic(self,image_topic,image_priority):
    priority_topic = image_topic
    if image_priority in self.image_priority_dict.keys():
      base_topic = os.path.dirname(image_topic)
      priority_name = self.image_priority_dict[image_priority]
      check_topic = os.path.join(base_topic,priority_name)
      topics = nepi_sdk.find_topics_by_msg('Image')
      if check_topic in  topics:
         priority_topic = check_topic
    return priority_topic
     




  ##############################
  ## Node PT Commands

  def stopPanTiltCb(self, _):
      self.stopPanTilt()

  def stopPanTilt(self):
      self.stopMoving() 
      self.publish_status()


  def stopMoving(self,axis = 'All'):       
        if axis == 'pan' or axis == 'All':
            self.stopPanControls()
            self.pt_connect_if.goto_to_pan_position(self.current_position[0])
        if axis == 'tilt' or axis == 'All':
            self.stopTiltControls()
            self.pt_connect_if.goto_to_tilt_position(self.current_position[1])
        self.publish_status() 




  def stopPanControls(self):
      self.scan_pan_enabled = False
      self.scan_pan_track_hold = False
      self.track_pan_enabled = False
      self.lock_pan_enabled = False
      self.click_pan_enabled = True


  def stopTiltControls(self):
      self.scan_tilt_enabled = False
      self.scan_tilt_track_hold = False
      self.track_tilt_enabled = False
      self.lock_tilt_enabled = False
      self.click_tilt_enabled = True

  def getPanClickEnabled(self):
     return (self.click_pan_enabled == True) and (self.scan_pan_enabled == False and self.track_pan_enabled == False and self.lock_pan_enabled == False)

  def getTiltClickEnabled(self):
     return (self.click_tilt_enabled == True) and (self.scan_tilt_enabled == False and self.track_tilt_enabled == False and self.lock_tilt_enabled == False)


  ##########################################
  # SCAN

  def setScanPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan pan: " + str(enabled))
        self.setScanPan(enabled)


  def setScanPan(self,enabled):
        was_scanning = copy.deepcopy(self.scan_pan_enabled)
        self.scan_pan_enabled = enabled
        self.publish_status()
        if enabled == True:
            self.scan_pan_track_hold = False
            #self.track_pan_enabled = False
            self.lock_pan_enabled = False
        if (was_scanning == True and enabled == False and self.track_pan_enabled == False):
            self.pt_connect_if.goto_to_pan_position(0.0)
        self.node_if.set_param('scan_pan_enabled', enabled)
        


  def setScanTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan tilt: " + str(enabled))
        self.setScanTilt(enabled)


  def setScanTilt(self,enabled):
        was_scanning = copy.deepcopy(self.scan_tilt_enabled)
        self.scan_tilt_enabled = enabled
        self.publish_status()
        if enabled == True:
            #self.track_tilt_enabled = False
            self.scan_tilt_track_hold = False
            self.lock_tilt_enabled = False
        if (was_scanning == True and enabled == False):
               self.pt_connect_if.goto_to_tilt_position(0.0)        
        self.node_if.set_param('scan_tilt_enabled', self.scan_tilt_enabled)




  def setScanPanWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
      else:
        self.msg_if.pub_info("Setting scan pan limits to: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
        self.setScanPanWindow(adj_min_deg,adj_max_deg)

  def setScanPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.scan_pan_min_deg = min_deg
            self.scan_pan_max_deg = max_deg
            self.msg_if.pub_info("Scan Pan limits set to: " + "%.2f" % min_deg * self.rpi + " " + "%.2f" % max_deg * self.rpi)
            self.publish_status()
            self.node_if.set_param('scan_pan_min_deg', min_deg)
            self.node_if.set_param('scan_pan_max_deg', max_deg)
            


  def setScanTiltWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting scan tilt limits to: " + "%.2f" % adj_min_deg * self.rti + " " + "%.2f" % adj_max_deg * self.rti)
      self.setScanTiltWindow(adj_min_deg,adj_max_deg)


  def setScanTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.scan_tilt_min_deg = min_deg
          self.scan_tilt_max_deg = max_deg
          self.msg_if.pub_info("Scan Tilt limits set to: " + "%.2f" % min_deg * self.rti + " " + "%.2f" % max_deg * self.rti)
          self.publish_status()
          self.node_if.set_param('scan_tilt_min_deg', min_deg)
          self.node_if.set_param('scan_tilt_max_deg', max_deg)



  ##########################################
  # TRACK

  def setTrackPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track pan: " + str(enabled))
        self.setTrackPan(enabled)


  def setTrackPan(self,enabled):
        if enabled == True:
            #self.scan_pan_enabled = False
            self.lock_pan_enabled = False           
        self.track_pan_enabled = enabled
        self.scan_pan_track_hold = enabled
        self.publish_status()
        self.node_if.set_param('track_pan_enabled', self.track_pan_enabled)

  def setTrackTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track tilt: " + str(enabled))
        self.setTrackTilt(enabled)

  def setTrackTilt(self,enabled):
        if enabled == True:
            #self.scan_tilt_enabled = False
            self.lock_tilt_enabled = False
        self.track_tilt_enabled = enabled
        self.scan_tilt_track_hold = enabled
        self.publish_status()
        self.node_if.set_param('track_tilt_enabled', self.track_tilt_enabled)


  def setTrackPanWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
      else:
        self.msg_if.pub_info("Setting track pan limits to: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
        self.setTrackPanWindow(adj_min_deg,adj_max_deg)

  def setTrackPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.track_pan_min_deg = min_deg
            self.track_pan_max_deg = max_deg
            self.msg_if.pub_info("Track Pan limits set to: " + "%.2f" % min_deg * self.rpi + " " + "%.2f" % max_deg * self.rpi)
            self.publish_status()
            self.node_if.set_param('track_pan_min_deg', min_deg)
            self.node_if.set_param('track_pan_max_deg', max_deg)
            


  def setTrackTiltWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting track tilt limits to: " + "%.2f" % adj_min_deg * self.rti + " " + "%.2f" % adj_max_deg * self.rti)
      self.setTrackTiltWindow(adj_min_deg,adj_max_deg)


  def setTrackTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.track_tilt_min_deg = min_deg
          self.track_tilt_max_deg = max_deg
          self.msg_if.pub_info("Track Tilt limits set to: " + "%.2f" % min_deg * self.rti + " " + "%.2f" % max_deg * self.rti)
          self.publish_status()
          self.node_if.set_param('track_tilt_min_deg', min_deg)
          self.node_if.set_param('track_tilt_max_deg', max_deg)

  def setTrackResetTimeSecCb(self, msg):
      reset_time = msg.data
      
      self.setTrackResetTimeSec(reset_time)


  def setTrackResetTimeSec(self,reset_time):
        if reset_time < 0:
            reset_time = -1
        self.msg_if.pub_info("Setting track reset time to: " + str(reset_time))
        self.track_reset_time_sec = reset_time
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_reset_time_sec', reset_time)
       

  ##########################################
  # Lock

  def setLockPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting lock pan: " + str(enabled))
        self.setLockPan(enabled)


  def setLockPan(self,enabled):
        was_locked = copy.deepcopy(self.lock_pan_enabled)
        if enabled == True:
            self.track_pan_enabled = False
        if (was_locked == True and enabled == False):
            if self.scan_pan_enabled == True:
                last_pan_error = self.lock_pan_error
                if last_pan_error > 0:
                    self.goto_position[0] = self.scan_pan_max_deg
                else:
                    self.goto_position[0] = self.scan_pan_min_deg
                self.scan_pan_track_hold = False
            else:
                self.goto_position[0] = 0.0
            self.pt_connect_if.goto_to_pan_position(self.goto_position[0])
            
        self.lock_pan_enabled = enabled
        self.publish_status()
        self.node_if.set_param('lock_pan_enabled', self.lock_pan_enabled)

  def setLockTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting lock tilt: " + str(enabled))
        self.setLockTilt(enabled)

  def setLockTilt(self,enabled):
        was_locked = copy.deepcopy(self.lock_tilt_enabled)
        if enabled == True:
            self.track_tilt_enabled = False
        if (was_locked == True and enabled == False):
            if self.scan_tilt_enabled == True:
                last_tilt_error = self.lock_tilt_error
                if last_tilt_error > 0:
                    self.goto_position[0] = self.scan_tilt_max_deg
                else:
                    self.goto_position[0] = self.scan_tilt_min_deg
                self.scan_tilt_track_hold = False
            else:
                self.goto_position[0] = 0.0
            self.pt_connect_if.goto_to_tilt_position(self.goto_position[0])
            
        self.lock_tilt_enabled = enabled
        self.publish_status()
        self.node_if.set_param('lock_tilt_enabled', self.lock_tilt_enabled)



  def setLockPanWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
      else:
        self.msg_if.pub_info("Setting lock pan limits to: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
        self.setLockPanWindow(adj_min_deg,adj_max_deg)

  def setLockPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.lock_pan_min_deg = min_deg
            self.lock_pan_max_deg = max_deg
            self.msg_if.pub_info("Lock Pan limits set to: " + "%.2f" % min_deg * self.rpi + " " + "%.2f" % max_deg * self.rpi)
            self.publish_status()
            self.node_if.set_param('lock_pan_min_deg', min_deg)
            self.node_if.set_param('lock_pan_max_deg', max_deg)
            


  def setLockTiltWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting lock tilt limits to: " + "%.2f" % adj_min_deg * self.rti + " " + "%.2f" % adj_max_deg * self.rti)
      self.setLockTiltWindow(adj_min_deg,adj_max_deg)


  def setLockTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.lock_tilt_min_deg = min_deg
          self.lock_tilt_max_deg = max_deg
          self.msg_if.pub_info("Lock Tilt limits set to: " + "%.2f" % min_deg * self.rti + " " + "%.2f" % max_deg * self.rti)
          self.publish_status()
          self.node_if.set_param('lock_tilt_min_deg', min_deg)
          self.node_if.set_param('lock_tilt_max_deg', max_deg)




  '''
  def setSinPanCb(self, msg):
      enabled = msg.data
      self.scan_pan_last_time = nepi_utils.get_time()
      self.msg_if.pub_info("Setting Sin pan: " + str(enabled))
      self.setSinPan(enabled)


  def setSinPan(self,enabled):
      self.sin_pan_enabled = enabled
      self.publish_status()
      if enabled == False and self.sin_tilt_enabled == False and self.sin_pan_enabled == False and self.setSpeedRatioCb is not None:
          self.msg_if.pub_info("2")
          self.setSpeedRatioCb(self.speed_ratio)
      self.node_if.set_param('sin_pan_enabled', self.sin_pan_enabled)
      
  '''





  def scanPanProcess(self,timer):
      #self.msg_if.pub_warn("Starting Pan Scan Process") 
      #self.msg_if.pub_warn("current_position: " + str(self.current_position)) 
      cur_time = nepi_utils.get_time()
      scan_time = None

      if self.current_position == None:
        self.pan_scanning = False
      else:
        if self.scan_pan_enabled == False or self.scan_pan_track_hold == True:
            self.pan_scanning = False
        elif self.pt_connect_if is not None:
            self.pan_scanning = True


            pan_cur = self.current_position[0]
            if self.goto_position[0] != self.scan_pan_min_deg and self.goto_position[0] != self.scan_pan_max_deg:
                self.msg_if.pub_warn("goto pan scan pos: " + str(self.scan_pan_min_deg)) 
                self.goto_position[0] = self.scan_pan_min_deg
                self.pt_connect_if.goto_to_pan_position(self.scan_pan_min_deg)  
              
            elif (pan_cur < (self.scan_pan_min_deg + self.SCAN_SWITCH_DEG)):
                last_time = self.scan_pan_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_pan_last_time
                self.scan_pan_last_time = nepi_utils.get_time()

                #self.msg_if.pub_warn("goto pan pos: " + str(self.scan_pan_max_deg)) 
                self.goto_position[0] = self.scan_pan_max_deg
                self.pt_connect_if.goto_to_pan_position(self.scan_pan_max_deg)

                
            elif (pan_cur > (self.scan_pan_max_deg - self.SCAN_SWITCH_DEG)):
                last_time = self.scan_pan_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_pan_last_time
                self.scan_pan_last_time = nepi_utils.get_time()
                self.goto_position[0] = self.scan_pan_min_deg
                self.pt_connect_if.goto_to_pan_position(self.scan_pan_min_deg)


            if scan_time is not None:
                self.scan_pan_times.pop(0)
                self.scan_pan_times.append(scan_time)
                
                # Calc scan pan times and sin
                scan_pan_times = copy.deepcopy(self.scan_pan_times)
                times = [x for x in scan_pan_times if x != 0]
                scan_pan_time = 0
                if len(times) > 0:
                    scan_pan_time = sum(times) / len(times)
                self.scan_pan_time = scan_time # scan_pan_time
                self.scan_pan_deg = abs(self.scan_pan_max_deg - self.scan_pan_min_deg)
                self.scan_pan_speed = self.scan_pan_deg/self.scan_pan_time

                # sin_len = math.ceil(scan_pan_time) *2
                # self.scan_pan_sins = list( (np.sin(  (np.linspace(0,1,sin_len)*4*math.pi) - (math.pi)/2) + 1  ) /2 )
                # self.scan_pan_sin_ind = 0
                # #self.msg_if.pub_warn("updated pan sin " + str(self.scan_pan_sins))



  def scanTiltProcess(self,timer):
      #self.msg_if.pub_warn("Starting Tilt Scan Process") 

      cur_time = nepi_utils.get_time()
      scan_time = None
      if self.current_position == None:
        self.tilt_scanning = False
      else:
        if self.scan_tilt_enabled == False or self.scan_tilt_track_hold == True:
            self.tilt_scanning = False
        elif self.pt_connect_if is not None:
            self.tilt_scanning = True


            tilt_cur = self.current_position[1]
            if self.goto_position[1] != self.scan_tilt_min_deg and self.goto_position[0] != self.scan_tilt_max_deg:
                self.goto_position[1] = self.scan_tilt_min_deg
                self.pt_connect_if.goto_to_tilt_position(self.scan_tilt_min_deg)  
            elif (tilt_cur < (self.scan_tilt_min_deg + self.SCAN_SWITCH_DEG)):
                last_time = self.scan_tilt_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_tilt_last_time
                self.scan_tilt_last_time = nepi_utils.get_time()
                self.goto_position[1] = self.scan_tilt_max_deg
                self.pt_connect_if.goto_to_tilt_position(self.scan_tilt_max_deg)

            elif (tilt_cur > (self.scan_tilt_max_deg - self.SCAN_SWITCH_DEG)):
                last_time = self.scan_tilt_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_tilt_last_time
                self.scan_tilt_last_time = nepi_utils.get_time()
                self.goto_position[1] = self.scan_tilt_min_deg
                self.pt_connect_if.goto_to_tilt_position(self.scan_tilt_min_deg)
            if scan_time is not None:
                self.scan_tilt_times.pop(0)
                self.scan_tilt_times.append(scan_time)
                
                # Calc scan tilt times and sin
                scan_tilt_times = copy.deepcopy(self.scan_tilt_times)
                times = [x for x in scan_tilt_times if x != 0]
                scan_tilt_time = 0
                if len(times) > 0:
                    scan_tilt_time = sum(times) / len(times)
                self.scan_tilt_time = scan_time # scan_tilt_time
                self.scan_tilt_deg = abs(self.scan_tilt_max_deg - self.scan_tilt_min_deg)
                self.scan_tilt_speed = self.scan_tilt_deg/self.scan_tilt_time

                # sin_len = math.ceil(scan_tilt_time) *2
                # self.scan_tilt_sins = list( (np.sin(  (np.linspace(0,1,sin_len)*4*math.pi) - (math.pi)/2) + 1  ) /2 )
                # self.scan_tilt_sin_ind = 0
                # self.msg_if.pub_warn("updated tilt sin " + str(self.scan_tilt_sins))



  def trackPanProcess(self,timer):
      if self.track_pan_enabled == False or self.track_source_connected == False:
           self.pan_tracking = False
      else:
          if self.current_position == None or self.pan_locked == True:
            self.pan_tracking = False
          else:
            pan_cur = self.current_position[0]
            track_dict = copy.deepcopy(self.track_pan_dict)
            last_error = copy.deepcopy(self.track_pan_error)
            if track_dict is not None:
                self.scan_pan_track_hold = True
                self.pan_tracking = True
                self.last_track_pan_time = nepi_utils.get_time()
                self.track_pan_error = track_dict['azimuth_deg']
                if abs(self.track_pan_error) > self.track_min_error_deg:
                  #self.msg_if.pub_warn("Got track pan error " + str(pan_error))    
                  pan_to_goal = pan_cur + self.track_pan_error * self.rpi * self.track_pan_sensitivity
                  if pan_to_goal != self.goto_position[0]:
                      self.goto_position[0] = pan_to_goal
                      self.pt_connect_if.goto_to_pan_position(pan_to_goal)
            else:
                last_track_pan_time = nepi_utils.get_time() - self.last_track_pan_time
                if last_track_pan_time > self.track_reset_time_sec:
                  self.pan_tracking = False

                  if self.scan_pan_enabled == True:
                      if self.scan_pan_track_hold == True: 
                          if last_error > 0:
                              self.goto_position[0] = self.scan_pan_max_deg
                          else:
                              self.goto_position[0] = self.scan_pan_min_deg
                      #self.msg_if.pub_warn("Resetting Pan Scan Process to: " + str(self.goto_position[0]))
                  elif self.goto_position[0] != 0.0:
                      self.goto_position[0] = 0.0
                  self.pt_connect_if.goto_to_pan_position(self.goto_position[0])

                  self.scan_pan_track_hold = False
      
                
      

  def trackTiltProcess(self,timer):
      if self.track_tilt_enabled == False or self.track_source_connected == False:
           self.tilt_tracking = False
      else:
          if self.current_position == None or self.tilt_locked == True:
            self.tilt_tracking = False
          else:
            tilt_cur = self.current_position[1]
            track_dict = copy.deepcopy(self.track_tilt_dict)
            last_error = copy.deepcopy(self.track_tilt_error)
            if track_dict is not None:
                self.scan_tilt_track_hold = True
                self.tilt_tracking = True
                self.track_tilt_error = track_dict['elevation_deg']
                self.last_track_tilt_time = nepi_utils.get_time()
                if abs(self.track_tilt_error) > self.track_min_error_deg:
                  #self.msg_if.pub_warn("Got track tilt error " + str(tilt_error))    
                  tilt_to_goal = tilt_cur + self.track_tilt_error * self.rpi * self.track_tilt_sensitivity
                  if tilt_to_goal != self.goto_position[1]:
                      self.goto_position[1] = tilt_to_goal
                      self.pt_connect_if.goto_to_tilt_position(tilt_to_goal)
            else:
                last_track_tilt_time = nepi_utils.get_time() - self.last_track_tilt_time
                if last_track_tilt_time > self.track_reset_time_sec:
                  self.tilt_tracking = False
                  if self.scan_tilt_enabled == True:
                      if self.scan_tilt_track_hold == True:
                          if last_error > 0:
                              self.goto_position[1] = self.scan_tilt_max_deg
                          else:
                              self.goto_position[1] = self.scan_tilt_min_deg
                          #self.msg_if.pub_warn("Resetting Tilt Scan Process to: " + str(self.goto_position[1]))
                  elif self.goto_position[1] != 0.0:
                      self.goto_position[1] = 0.0
                  self.pt_connect_if.goto_to_tilt_position(self.goto_position[1])

                  self.scan_tilt_track_hold = False
                





  def setPanClickCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Pan Enabled: " + str(enabled))
      self.setPanClick(enabled)

  def setPanClick(self,enabled):
      if self.click_pan_enabled == False:
          self.click_position = [0,0]
      if self.click_pan_enabled == True:
          self.stopPanControls()
          self.track_pan_error = 0

      self.click_pan_enabled = enabled
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('click_pan_enabled', self.scan_tilt_enabled) 
        #self.node_if.save_config()

  def setTiltClickCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Tilt: " + str(enabled))
      #self.setTiltClick(enabled)

  def setTiltClick(self,enabled):
      if self.click_tilt_enabled == False:
          self.click_position = [0,0]
      if self.click_tilt_enabled == True:
          self.scan_tilt_enabled = False
          self.track_tilt_enabled = False
          self.track_tilt_error = 0
      self.click_tilt_enabled = enabled
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('click_tilt_enabled', self.scan_tilt_enabled)
        self.node_if.save_config()

  def clickCb(self,msg):
      
      click_count = msg.click_count

      if click_count > 1:
        if self.num_windows == 1:
           self.setNumWindows(self.last_num_windows)
        else:
           image_index = msg.image_index
           image_topic = msg.image_topic
           if image_topic != 'None':
            self.single_image_topic = image_topic
            self.publish_status()
            self.setNumWindows(1)
         
      else:
        self.click_position = [0,0]
        click_pan_enabled = self.getPanClickEnabled()
        click_tilt_enabled = self.getTiltClickEnabled()
        image_index = msg.image_index
        if msg.click_event == True:
            if self.num_windows > 1:
                self.single_image_topic = msg.image_topic
                self.publish_status()
                if self.node_if is not None:
                    self.node_if.set_param('single_image_topic', msg.image_topic)
                    self.node_if.save_config()            
            else:
                pixel = [msg.click_pixel.x, msg.click_pixel.y ]
                status_msg = msg.image_status_msg
                image_width = status_msg.width_px
                image_height = status_msg.height_px
                image_fov_horz = status_msg.width_deg
                image_fov_vert = status_msg.height_deg
                image_zoom_ratio = status_msg.zoom_ratio
                if image_width > 10 and image_height > 10 and image_fov_horz > 10 and image_fov_vert > 10 and image_zoom_ratio < 0.1:
                    object_loc_x_ratio_from_center = float(pixel[0] - image_width/2) / float(image_width/2)
                    object_loc_y_ratio_from_center = float(pixel[1] - image_height/2) / float(image_height/2)
                    vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
                    horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))
                    self.click_position = [horz_angle_deg,vert_angle_deg]

                if click_pan_enabled == True:
                    if self.current_position == None:
                        pass
                    else:
                        pan_cur = self.current_position[0]
                        pan_to_goal = self.click_position[0] + pan_cur
                        self.msg_if.pub_warn("Pixel Selected, Going to Pan Pos " + str(pan_to_goal))#)
                        self.goto_position[0] = pan_to_goal
                        self.pt_connect_if.goto_to_pan_position(pan_to_goal)
                else: 
                    self.msg_if.pub_warn("Pan Click Enabled is False")#)



                if click_tilt_enabled == True:
                    if self.current_position == None:
                        pass
                    else:
                        tilt_cur = self.current_position[1]
                        tilt_to_goal = self.click_position[1] + tilt_cur
                        self.msg_if.pub_warn("Pixel Selected, Going to Tilt Pos " + str(tilt_to_goal))#)
                        self.goto_position[1] = tilt_to_goal
                        self.pt_connect_if.goto_to_tilt_position(tilt_to_goal)
                else: 
                    self.msg_if.pub_warn("Tilt Click Enabled is False")#)


  def selectTopicCb(self,msg):
    selected_pan_tilt = msg.data
    if selected_pan_tilt in self.available_pan_tilts:
      self.selected_pan_tilt = selected_pan_tilt
      self.publish_status()
      if self.node_if is not None:
        self.msg_if.pub_warn("selected_pan_tilt: " + str(selected_pan_tilt))
        self.node_if.set_param('selected_pan_tilt', selected_pan_tilt)
    



  def subscribe_pt_topic(self, topic):
    self.msg_if.pub_warn("subscribe_pt_topic Called")

    success = False
    if self.pt_connect_if is not None:
      success = self.unsubscribe_pt_topic()

    pt_connect_if = ConnectPTXDeviceIF(namespace = topic,
                                       panTiltCb = self.panTiltCb,
                                       stopPanCb = self.stopPanCb,
                                       stopTiltCb = self.stopTiltCb,
                                       msg_if = self.msg_if
                                        )
    ready = pt_connect_if.wait_for_ready()
    if ready == True:
      self.pt_connect_if = pt_connect_if
      self.connected_topic = topic
      self.msg_if.pub_warn("connected_topic: " + str(self.connected_topic))
      if self.pt_connect_if is not None:
        limits = self.pt_connect_if.get_pan_tilt_soft_limits()
        self.msg_if.pub_warn("setting scan limits: " + str(limits))
        if limits is not None:
            self.min_pan_softstop_deg = limits[0]
            self.max_pan_softstop_deg = limits[1]
            self.min_tilt_softstop_deg = limits[2]
            self.max_tilt_softstop_deg = limits[3]

    return success
  
  def unsubscribe_pt_topic(self):
    self.msg_if.pub_warn("unsubscribe_pt_topic Called")

    success = True
    if self.pt_connect_if is not None:
      success = self.pt_connect_if.unregister()
      self.connected = False
      self.connected_topic = None
      self.current_position = None
      nepi_sdk.sleep(1)
      self.pt_connect_if = None
    return success

  def panTiltCb(self, pan_deg, tilt_deg):
     self.current_position = [pan_deg, tilt_deg]
     #self.msg_if.pub_warn("PT position: " + str(self.current_position))

  def stopPanCb(self):
     self.msg_if.pub_warn("Got Stop Pan Cb")
     self.stopPanControls()

  def stopTiltCb(self):
     self.msg_if.pub_warn("Got Stop Tilt Cb")
     self.stopTiltControls()  

  def connectTrackSourceCb(self,timer):
    #self.msg_if.pub_info("connectTrackSourceCb called")
    track_namespace = copy.deepcopy(self.track_source_namespace)
    if track_namespace != self.track_source_connected_namespace  and self.track_source_connecting == False:
        if self.track_if is not None:
            self.track_if = None
            nepi_sdk.sleep(1)
        self.track_source_connected = False
        #self.msg_if.pub_warn("set_track_source connected False")
        #self.msg_if.pub_warn("track_namespace: " + str(nepi_sdk.find_topic(track_namespace)))

        if nepi_sdk.find_topic(track_namespace) != "": 

            self.track_source_connecting = True                 
            self.track_if = nepi_sdk.create_subscriber(track_namespace, Targets, self.targetsCb, queue_size = 1, callback_args= (track_namespace), log_name_list = [])
           
    
    nepi_sdk.start_timer_process(1, self.connectTrackSourceCb, oneshot = True)



  def targetsCb(self,msg, args):
    #self.msg_if.pub_info("Targets callback got new targets mgs")
    target_namespace = args
    self.track_source_connected = True
    #self.msg_if.pub_warn("set_track_source connected True")
    self.track_source_connected_namespace = self.track_source_namespace
    self.track_source_connecting == False
    self.targets_msg = msg.targets
    self.last_targets_time = nepi_utils.get_time()


    #self.msg_if.pub_warn("Got targets msg list " + str(targets_msg))
    targets_dict_list = []
    for target_msg in self.targets_msg:
        target_dict = nepi_targets.convert_target_msg2dict(target_msg)
        targets_dict_list.append(target_dict)
        #self.msg_if.pub_warn("Added target list for name " + str(target_dict['target_name']))
    targets_dict_list = nepi_targets.filter_by_names(targets_dict_list,self.track_ordered_list)

    track_dict = None
    if len(targets_dict_list) > 0:
        #self.msg_if.pub_warn("Processing target list length " + str(len(targets_dict_list)))
        track_dict = nepi_targets.find_best(targets_dict_list, best_filter = self.track_best_filter)

    self.track_pan_dict = copy.deepcopy(track_dict)
    self.track_tilt_dict = copy.deepcopy(track_dict)


##########################
###Image Veiwer
##########################

  def setImageTopic1Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 0
    img_topic = msg.data
    if self.num_windows == 1:
        self.single_image_topic = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('single_image_topic', msg.data)
            self.node_if.save_config()       
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        self.node_if.save_config()

  def setImageTopic2Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 1
    img_topic = msg.data
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        self.node_if.save_config()

  def setImageTopic3Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 2
    img_topic = msg.data
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        self.node_if.save_config()

  def setImageTopic4Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 3
    img_topic = msg.data
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        self.node_if.save_config()
      
  def setNumWindowsCb(self,msg):
    self.msg_if.pub_info(str(msg))
    num_windows = msg.data
    self.setNumWindows(num_windows)

  def setNumWindows(self,num_windows):
    if num_windows > 0 and num_windows < 5:
      if num_windows == 1 and self.num_windows != 1:
         self.last_num_windows = copy.deepcopy(self.num_windows)
      self.num_windows = num_windows
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('num_windows', self.num_windows)
        self.node_if.save_config()

  def setImagePriorityCb(self,msg):
    image_priority = msg.data
    self.single_image_topic = self.get_image_priority_topic(self.single_image_topic,image_priority)
    selected_image_topics = []
    for image_topic in self.selected_image_topics:
      selected_image_topics.append(self.get_image_priority_topic(image_topic,image_priority))
    self.selected_image_topics = selected_image_topics
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param('single_image_topic', self.single_image_topic)
      self.node_if.set_param('selected_image_topics', self.selected_image_topics)
      self.node_if.save_config()

        
        
     

  ####################

 
  

  def publishStatusCb(self,timer):
    self.publish_status()

  def publish_status(self):
    self.status_msg.available_pan_tilts = self.available_pan_tilts
    selected_pan_tilt = 'None'
    if self.selected_pan_tilt in self.available_pan_tilts:
       selected_pan_tilt = self.selected_pan_tilt
    self.status_msg.selected_pan_tilt = selected_pan_tilt

    connected_topic = self.connected_topic
    if connected_topic is None:
       connected_topic = 'None'
    self.status_msg.connected_topic = connected_topic
    self.status_msg.connected = self.connected

    ###
    current_position = [-999,-999]
    #self.msg_if.pub_warn("self.current_position: " + str(self.current_position))
    if self.current_position is not None:
      current_position = self.current_position

    self.status_msg.current_position = current_position


    self.status_msg.pan_scanning = self.pan_scanning
    self.status_msg.tilt_scanning = self.tilt_scanning

    self.status_msg.pan_tracking = self.pan_tracking
    self.status_msg.tilt_tracking = self.tilt_tracking

    self.status_msg.pan_locked = self.pan_locked
    self.status_msg.tilt_locked = self.tilt_locked


    self.status_msg.scan_pan_enabled = self.scan_pan_enabled
    self.status_msg.scan_tilt_enabled = self.scan_tilt_enabled
    self.status_msg.scan_pan_min_deg = self.scan_pan_min_deg
    self.status_msg.scan_pan_max_deg = self.scan_pan_max_deg
    self.status_msg.scan_tilt_min_deg = self.scan_tilt_min_deg
    self.status_msg.scan_tilt_max_deg = self.scan_tilt_max_deg




    self.status_msg.track_source_namespace = self.track_source_namespace
    self.status_msg.track_source_connected = self.track_source_connected
    self.status_msg.track_pan_enabled = self.track_pan_enabled
    self.status_msg.track_tilt_enabled = self.track_tilt_enabled
    self.status_msg.track_pan_min_deg = self.track_pan_min_deg
    self.status_msg.track_pan_max_deg = self.track_pan_max_deg
    self.status_msg.track_tilt_min_deg = self.track_tilt_min_deg
    self.status_msg.track_tilt_max_deg = self.track_tilt_max_deg
    self.status_msg.track_reset_time_sec = self.track_reset_time_sec

    self.status_msg.lock_pan_enabled = self.lock_pan_enabled
    self.status_msg.lock_tilt_enabled = self.lock_tilt_enabled
    self.status_msg.lock_pan_min_deg = self.lock_pan_min_deg
    self.status_msg.lock_pan_max_deg = self.lock_pan_max_deg
    self.status_msg.lock_tilt_min_deg = self.lock_tilt_min_deg
    self.status_msg.lock_tilt_max_deg = self.lock_tilt_max_deg
    
    #self.status_msg.scan_speed_ratio =

    ###############
    ###Image Viewer
    ###############
    image_topics = copy.deepcopy(self.selected_image_topics)
    self.status_msg.num_windows = self.num_windows
    if self.num_windows == 1:
        image_topics[0] = self.single_image_topic
    for i, topic in enumerate(image_topics):
       if topic != 'None':
          if topic not in self.available_image_topics:
             image_topics[i] = 'None'
    self.status_msg.image_topics = image_topics
    self.status_msg.image_priority_options = self.image_priority_list

    ############


    if self.node_if is not None:
      if self.has_published == False:
        #self.msg_if.pub_warn("Publishing Status: " + str(self.status_msg))
        self.has_published = True
      self.node_if.publish_pub('status_pub', self.status_msg) 
      self.node_if.save_config()
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info(" Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiPanTiltAutoApp()

