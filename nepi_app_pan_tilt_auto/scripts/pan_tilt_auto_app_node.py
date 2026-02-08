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
from nepi_interfaces.msg import RangeWindow, Target, Targets, TargetFilter, TargetFilters, TargetingStatus


from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF


UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

#########################################
# Node Class
#########################################

class NepiPanTiltAutoApp(object):

  AUTO_SCAN_SWITCH_DEG = 5 # If angle withing this bound, switch dir
  AUTO_SCAN_UPDATE_INTERVAL = .5

  TRACK_MAX_UPDATE_RATE = 1
  TRACK_EXIT_FUNCTION = 'HOME'
  TRACK_DEFAULT_TARGETS = ['person']

  TRACK_FILTER_OPTIONS = nepi_targets.TARGET_FILTER_OPTIONS
  TRACK_DEFAULT_FILTER = 'LARGEST'
  TRACK_MIN_ERROR_DEG = 5
  TRACK_DEFAULT_SOURCE = 'targets'

  has_auto_pan = True
  has_auto_tilt = True
  has_sin_pan = False
  has_sin_tilt = False
  has_homing = False
  has_set_home = False


  current_position = None

  scan_pan_min = -50
  scan_pan_max = 50
  scan_tilt_min = -50
  scan_tilt_max = 50



  is_auto_pan = False
  start_auto_pan = False
  auto_pan_enabled = False
  track_pan_enabled = False
  nav_lock_pan_enabled = False
  auto_pan_sec = 5

  auto_pan_last_time = None
  auto_pan_times = [0,0,0,0,0]
  auto_pan_time = 1
  sin_pan_enabled = False
  auto_pan_sin_ind = 0
  
  rpi = 1
  rti = 1

  is_auto_tilt = False
  start_auto_tilt = False
  auto_tilt_enabled = False
  track_tilt_enabled = False
  nav_lock_tilt_enabled = False
  auto_tilt_sec = 5

  auto_tilt_last_time = None
  auto_tilt_times = [0,0,0,0,0]
  auto_tilt_time = 1
  sin_tilt_enabled = False
  auto_tilt_sin_ind = 0

  track_ordered_list = TRACK_DEFAULT_TARGETS
  track_max_update_rate = TRACK_MAX_UPDATE_RATE
  track_filter = TRACK_DEFAULT_FILTER
  track_min_error_deg = TRACK_MIN_ERROR_DEG
  track_source_topic = TRACK_DEFAULT_SOURCE
  track_source_connected_namespace = "None"
  track_source_connected = False
  track_source_connecting = False

  track_exit_process = TRACK_EXIT_FUNCTION
  track_source_namespace = 'None'
  track_last_namespace = 'None'
  track_if = None
  track_dict = None

  num_errors = 1
  pan_errors = []
  tilt_errors = []

  last_track_msg = None
  track_dict = None


  pt_connect_if = None
  node_if = None
  process_needs_update = False
  status_msg = PanTiltAutoAppStatus() 

  available_pan_tilts = []
  selected_pan_tilt = "None"
  connected_topic = None
  connected = False
  pt_status_msg = DevicePTXStatus()

  min_pan_softstop_deg = -50
  max_pan_softstop_deg = 50
  min_tilt_softstop_deg = -50
  max_tilt_softstop_deg = 50
  


  scan_pan_deg = 0
  scan_pan_speed = 0
  scan_pan_deg = 0
  scan_pan_speed = 0

  auto_tilt_sins = 0
  auto_tilt_sin_ind = 0
  auto_tilt_sins = 0
  auto_tilt_sin_ind = 0

  pan_errors = []
  tilt_errors = []
  num_errors = 1

  pan_deg = 0
  Tilt_deg = 0

  pan_goal_deg = 0.0
  tilt_goal_deg = 0.0

  navpose_update_rate = 1
  status_update_rate = 1

  ############

  click_pan_enabled = True
  click_tilt_enabled = True
  set_click_position = [0,0]

  #################
  ### Image Viewer
  #################
  FACTORY_SELECTED_PAN_TILTS = ["None","None","None","None"]

  update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ

  single_image_topic = "None"
  selected_image_topics = ["None","None","None","None"]
  num_windows = 1
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_pan_tilt_auto" # Can be overwitten by luanch command
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

    self.auto_pan_times = [0,0,0,0,0]
    self.auto_tilt_times = [0,0,0,0,0]
    self.auto_pan_sins = []
    self.auto_pan_sin_ind = 0
    self.auto_tilt_sins = []
    self.auto_tilt_sin_ind = 0



    # AUTO SCANNING ##############
    # timed auto scanning is not supported yet


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
        'auto_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },   
        'track_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },           
        'min_auto_pan_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_pan_min
        },           
        'max_auto_pan_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_pan_max
        },       
        'sin_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'sin_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'track_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },  
        'auto_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },           
        'min_auto_tilt_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_tilt_min
        },           
        'max_auto_tilt_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_tilt_max
        },
        'min_auto_tilt_deg': {
            'namespace': self.node_namespace,
            'factory_val':0 #self.factoryLimits['max_tilt_softstop_deg']
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
        'set_track_pan': {
            'namespace': self.node_namespace,
            'topic': 'set_track_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackPanHandler, 
            'callback_args': ()
        },
        'set_auto_pan': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setAutoPanHandler, 
            'callback_args': ()
        },
        'set_auto_pan_window': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_pan_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setAutoPanWindowHandler, 
            'callback_args': ()
        },
        '''
        'set_auto_pan_sin': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_pan_sin_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setSinPanHandler, 
            'callback_args': ()
        },
        '''
        'set_track_tilt': {
            'namespace': self.node_namespace,
            'topic': 'set_track_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackTiltHandler, 
            'callback_args': ()
        },
        'set_auto_tilt': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setAutoTiltHandler, 
            'callback_args': ()
        },
        'set_auto_tilt_window': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_tilt_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setAutoTiltWindowHandler, 
            'callback_args': ()
        },
        'set_pan_click_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_pan_click_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setPanClickHandler, 
            'callback_args': ()
        },
        'set_tilt_click_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_tilt_click_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setTiltClickHandler, 
            'callback_args': ()
        },


        ######################
        ###Image Viewer
        ######################
        'set_click_position': {
            'namespace': self.node_namespace,
            'topic': 'set_click_position',
            'msg': ImageMouseEvent,
            'qsize': None,
            'callback': self.clickPositionCB, 
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

    self.msg_if.pub_warn("has_auto_pan: " + str(self.has_auto_pan))
    if self.has_auto_pan:
        # Start Auto Pan Process
        self.msg_if.pub_info("Starting auto pan scanning process")
        nepi_sdk.start_timer_process(self.AUTO_SCAN_UPDATE_INTERVAL, self.autoPanProcess)
        nepi_sdk.start_timer_process(self.track_max_update_rate, self.trackPanProcess)


    if self.has_auto_tilt:
        # Start Auto Pan Process
        self.msg_if.pub_info("Starting auto tilt scanning process")
        nepi_sdk.start_timer_process(self.AUTO_SCAN_UPDATE_INTERVAL, self.autoTiltProcess)
        nepi_sdk.start_timer_process(self.track_max_update_rate, self.trackTiltProcess)



    #self.msg_if.pub_warn("supports_sin_scan: " + str(self.supports_sin_scan))
    #if self.supports_sin_scan:
        #self.msg_if.pub_warn("Starting sin scanning process")
        #nepi_sdk.start_timer_process(.5, self.autoPanSinProcess, oneshot = True)
        #nepi_sdk.start_timer_process(.5, self.autoTiltSinProcess, oneshot = True)

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
      self.scan_pan_min = self.node_if.get_param('min_auto_pan_deg')
      self.scan_pan_max = self.node_if.get_param('max_auto_pan_deg')
      self.scan_tilt_min = self.node_if.get_param('min_auto_tilt_deg')
      self.scan_tilt_max = self.node_if.get_param('max_auto_tilt_deg')
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




  ##############################
  ## Node PT Commands

  def stopMovingHandler(self, _):
      self.stopMoving()

  def stopMoving(self):
      self.stopPanControls()
      self.stopTiltControls()
      pan_cur_deg = self.status_msg.pan_now_deg * self.rpi
      tilt_cur_deg = self.status_msg.tilt_now_deg * self.rti
      self.msg_if.pub_info("Stopping motion by request", log_name_list = self.log_name_list)
      if self.stopMovingCb is not None:
          self.stopMovingCb()
      elif self.gotoPositionCb is not None:
          self.gotoPositionCb(pan_cur_deg,tilt_cur_deg)
      self.publish_status()



  def stopPanControls(self):
      self.setAutoPan(False)
      self.setTrackPan(False)
      self.setNavLockPan(False)
      self.click_pan_enabled = True


  def stopTiltControls(self):
      self.setTrackTilt(False)
      self.setAutoTilt(False)
      self.setNavLockTilt(False)
      self.click_tilt_enabled = True

  def getPanClickEnabled(self):
     return (self.click_pan_enabled == True) and (self.auto_pan_enabled == False and self.track_pan_enabled == False and self.nav_lock_pan_enabled == False)

  def getTiltClickEnabled(self):
     return (self.click_tilt_enabled == True) and (self.auto_tilt_enabled == False and self.track_tilt_enabled == False and self.nav_lock_tilt_enabled == False)

  def setAutoPanWindowHandler(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
      else:
        self.msg_if.pub_info("Setting auto pan limits to: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
        self.setAutoPanWindow(adj_min_deg,adj_max_deg)

  def setAutoPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.scan_pan_min = min_deg
            self.scan_pan_max = max_deg
            self.msg_if.pub_info("Auto Pan limits set to: " + "%.2f" % min_deg * self.rpi + " " + "%.2f" % max_deg * self.rpi)
            self.publish_status()
            self.node_if.set_param('min_pan_softstop_deg', min_deg)
            self.node_if.set_param('max_pan_softstop_deg', max_deg)
            




  def setAutoTiltWindowHandler(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting auto tilt limits to: " + "%.2f" % adj_min_deg * self.rti + " " + "%.2f" % adj_max_deg * self.rti)
      self.setAutoTiltWindow(adj_min_deg,adj_max_deg)


  def setAutoTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.scan_tilt_min = min_deg
          self.scan_tilt_max = max_deg
          self.msg_if.pub_info("Auto Tilt limits set to: " + "%.2f" % min_deg * self.rti + " " + "%.2f" % max_deg * self.rti)
          self.publish_status()
          self.node_if.set_param('min_tilt_softstop_deg', min_deg)
          self.node_if.set_param('max_tilt_softstop_deg', max_deg)



  def setAutoPanHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting auto pan: " + str(enabled))
      self.setAutoPan(enabled)

  def setAutoPan(self,enabled):
      self.track_pan_enabled = False
      self.nav_lock_pan_enabled = False
      self.auto_pan_enabled = enabled
      self.msg_if.pub_info("self.auto_pan_enabled: " + str(self.auto_pan_enabled))
      self.publish_status()
      self.node_if.set_param('auto_pan_enabled', self.auto_pan_enabled)
        


  def setAutoTiltHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting auto tilt: " + str(enabled))
      self.setAutoTilt(enabled)

  def setAutoTilt(self,enabled):
      self.track_tilt_enabled = False
      self.nav_lock_tilt_enabled = False
      self.auto_tilt_enabled = enabled
      self.publish_status()
      self.node_if.set_param('auto_tilt_enabled', self.auto_tilt_enabled) 



  def setTrackPanHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting track pan: " + str(enabled))
      self.setTrackPan(enabled)

  def setTrackPan(self,enabled):
      self.auto_pan_enabled = False
      self.nav_lock_pan_enabled = False
      self.track_pan_enabled = enabled
      self.publish_status()
      self.node_if.set_param('track_pan_enabled', self.track_pan_enabled)



  def setTrackTiltHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting track tilt: " + str(enabled))
      self.setTrackTilt(enabled)

  def setTrackTilt(self,enabled):
      self.auto_tilt_enabled = False
      self.nav_lock_tilt_enabled = False
      self.track_tilt_enabled = enabled
      self.publish_status()
      self.node_if.set_param('track_tilt_enabled', self.track_tilt_enabled) 



  def setNavLockPanHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting nav lock pan: " + str(enabled))
      self.setTrackPan(enabled)

  def setNavLockPan(self,enabled):
      self.auto_pan_enabled = False
      self.track_pan_enabled = False
      self.nav_lock_pan_enabled = enabled
      self.publish_status()
      self.node_if.set_param('nav_lock_pan_enabled', self.nav_lock_pan_enabled)




  def setNavLockTiltHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting nav lock tilt: " + str(enabled))
      self.setTrackTilt(enabled)

  def setNavLockTilt(self,enabled):
      self.auto_tilt_enabled = False
      self.track_tilt_enabled = False
      self.nav_lock_tilt_enabled = enabled
      self.publish_status()
      self.node_if.set_param('nav_lock_tilt_enabled', self.nav_lock_tilt_enabled)



  '''
  def setSinPanHandler(self, msg):
      enabled = msg.data
      self.auto_pan_last_time = nepi_utils.get_time()
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



  def autoPanProcess(self,timer):
      #self.msg_if.pub_warn("Starting Pan Scan Process") 
      #self.msg_if.pub_warn("current_position: " + str(self.current_position)) 
      cur_time = nepi_utils.get_time()
      scan_time = None

      if self.current_position == None:
        pass
      else:
        if self.auto_pan_enabled == False:
            self.is_auto_pan = False
        elif self.pt_connect_if is not None:
            start_auto_pan = False
            if self.is_auto_pan == False:
                start_auto_pan = True     
            self.is_auto_pan = True    

            pan_cur = self.current_position[0]
            if start_auto_pan == True:

                self.msg_if.pub_warn("goto pan pos: " + str(self.scan_pan_min)) 
                self.pt_connect_if.goto_to_pan_position(self.scan_pan_min)  
              
            elif (pan_cur < (self.scan_pan_min + self.AUTO_SCAN_SWITCH_DEG)):
                last_time = self.auto_pan_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.auto_pan_last_time
                self.auto_pan_last_time = nepi_utils.get_time()

                #self.msg_if.pub_warn("goto pan pos: " + str(self.scan_pan_max)) 
                self.pt_connect_if.goto_to_pan_position(self.scan_pan_max)

                
            elif (pan_cur > (self.scan_pan_max - self.AUTO_SCAN_SWITCH_DEG)):
                last_time = self.auto_pan_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.auto_pan_last_time
                self.auto_pan_last_time = nepi_utils.get_time()

                self.pt_connect_if.goto_to_pan_position(self.scan_pan_min)
        if scan_time is not None:
            self.auto_pan_times.pop(0)
            self.auto_pan_times.append(scan_time)
            
            # Calc auto pan times and sin
            auto_pan_times = copy.deepcopy(self.auto_pan_times)
            times = [x for x in auto_pan_times if x != 0]
            auto_pan_time = 0
            if len(times) > 0:
                auto_pan_time = sum(times) / len(times)
            self.auto_pan_time = scan_time # auto_pan_time
            self.scan_pan_deg = abs(self.scan_pan_max - self.scan_pan_min)
            self.scan_pan_speed = self.scan_pan_deg/self.auto_pan_time

            sin_len = math.ceil(auto_pan_time) *2
            self.auto_pan_sins = list( (np.sin(  (np.linspace(0,1,sin_len)*4*math.pi) - (math.pi)/2) + 1  ) /2 )
            self.auto_pan_sin_ind = 0
            #self.msg_if.pub_warn("updated pan sin " + str(self.auto_pan_sins), log_name_list = self.log_name_list)



  def autoTiltProcess(self,timer):
      #self.msg_if.pub_warn("Starting Tilt Scan Process") 

      cur_time = nepi_utils.get_time()
      scan_time = None
      if self.current_position == None:
        pass
      else:
        if self.auto_tilt_enabled == False:
            self.is_auto_tilt = False

        else:
            if self.track_tilt_enabled == True:
                self.track_tilt_enabled = False
            start_auto_tilt = False
            if self.is_auto_tilt == False:
                start_auto_tilt = True     
            self.is_auto_tilt = True    
            tilt_cur = self.current_position[1]
            if start_auto_tilt == True:
                self.pt_connect_if.goto_to_tilt_position(self.scan_tilt_min)  
            elif (tilt_cur < (self.scan_tilt_min + self.AUTO_SCAN_SWITCH_DEG)):
                last_time = self.auto_tilt_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.auto_tilt_last_time
                self.auto_pan_last_time = nepi_utils.get_time()

                self.pt_connect_if.goto_to_tilt_position(self.scan_tilt_max)

            elif (tilt_cur > (self.scan_tilt_max - self.AUTO_SCAN_SWITCH_DEG)):
                last_time = self.auto_tilt_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.auto_tilt_last_time
                self.auto_pan_last_time = nepi_utils.get_time()

                self.pt_connect_if.goto_to_tilt_position(self.scan_tilt_min)
        if scan_time is not None:
            self.auto_tilt_times.pop(0)
            self.auto_tilt_times.append(scan_time)
            
            # Calc auto pan times and sin
            auto_tilt_times = copy.deepcopy(self.auto_tilt_times)
            times = [x for x in auto_tilt_times if x != 0]
            auto_tilt_time = 0
            if len(times) > 0:
                auto_tilt_time = sum(times) / len(times)
            self.auto_tilt_time = scan_time # auto_tilt_time
            self.scan_tilt_deg = abs(self.scan_tilt_max - self.scan_tilt_min)
            self.scan_tilt_speed = self.scan_tilt_deg/self.auto_tilt_time

            sin_len = math.ceil(auto_tilt_time) *2
            self.auto_tilt_sins = list( (np.sin(  (np.linspace(0,1,sin_len)*4*math.pi) - (math.pi)/2) + 1  ) /2 )
            self.auto_tilt_sin_ind = 0
            self.msg_if.pub_warn("updated tilt sin " + str(self.auto_tilt_sins), log_name_list = self.log_name_list)



  def trackPanProcess(self,timer):
      if self.track_pan_enabled == True and self.track_source_connected == True:
          if self.current_position == None:
            pass
          else:
            pan_cur = self.current_position[0]
            track_dict = copy.deepcopy(self.track_dict)
            if track_dict is not None:
                pan_error = track_dict['azimuth_deg']
                #self.msg_if.pub_warn("Got track pan error " + str(pan_error), log_name_list = self.log_name_list)
                
                self.pan_errors.append(pan_error)
                if len(self.pan_errors) > self.num_errors:
                    self.pan_errors.pop(0)
                avg_error =  sum(self.pan_errors) / len(self.pan_errors)
                #self.msg_if.pub_warn("Got avg pan error " + str(avg_error), log_name_list = self.log_name_list)
                if abs(avg_error) > self.track_min_error_deg:
                    pan_to_goal = pan_cur + avg_error /2 * self.rpi
                    self.pt_connect_if.goto_to_pan_position(pan_to_goal)
            else:
                self.pt_connect_if.goto_to_pan_position(0.0)
      

  def trackTiltProcess(self,timer):
      if self.track_tilt_enabled == True and self.track_source_connected == True:
          if self.current_position == None:
            pass
          else:
            tilt_cur = self.current_position[1]
            track_dict = copy.deepcopy(self.track_dict)
            if track_dict is not None:
                tilt_error = track_dict['elevation_deg']
                #self.msg_if.pub_warn("Got track tilt error " + str(tilt_error), log_name_list = self.log_name_list)
                
                self.tilt_errors.append(tilt_error)
                if len(self.tilt_errors) > self.num_errors:
                    self.tilt_errors.pop(0)
                avg_error =  sum(self.tilt_errors) / len(self.tilt_errors)
                #self.msg_if.pub_warn("Got avg tilt error " + str(avg_error), log_name_list = self.log_name_list)
                if abs(avg_error) > self.track_min_error_deg:
                    tilt_to_goal = tilt_cur + avg_error /2 * self.rti
                    self.pt_connect_if.goto_to_tilt_position(tilt_to_goal)
            else:
                self.pt_connect_if.goto_to_tilt_position(0.0)





  def setPanClickHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Pan Enabled: " + str(enabled))
      self.setPanClick(enabled)

  def setPanClick(self,enabled):
      if self.click_pan_enabled == False:
          self.click_position = [0,0]
      if self.click_pan_enabled == True:
          self.stopPanControls()
          self.pan_errors = []

      self.click_pan_enabled = enabled
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('click_pan_enabled', self.auto_tilt_enabled) 
        #self.node_if.save_config()

  def setTiltClickHandler(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Tilt: " + str(enabled))
      #self.setTiltClick(enabled)

  def setTiltClick(self,enabled):
      if self.click_tilt_enabled == False:
          self.click_position = [0,0]
      if self.click_tilt_enabled == True:
          self.auto_tilt_enabled = False
          self.track_tilt_enabled = False
          self.tilt_errors = []
      self.click_tilt_enabled = enabled
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('click_tilt_enabled', self.auto_tilt_enabled)
        self.node_if.save_config()

  def clickPositionCB(self,msg):
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
                    self.msg_if.pub_warn("Pixel Selected, Going to Pan Pos " + str(pan_to_goal))#, log_name_list = self.log_name_list)
                    self.pt_connect_if.goto_to_pan_position(pan_to_goal)
            else: 
                self.msg_if.pub_warn("Pan Click Enabled is False")#, log_name_list = self.log_name_list)



            if click_tilt_enabled == True:
                if self.current_position == None:
                    pass
                else:
                    tilt_cur = self.current_position[1]
                    tilt_to_goal = self.click_position[1] + tilt_cur
                    self.msg_if.pub_warn("Pixel Selected, Going to Tilt Pos " + str(tilt_to_goal))#, log_name_list = self.log_name_list)

                    self.pt_connect_if.goto_to_tilt_position(tilt_to_goal)
            else: 
                self.msg_if.pub_warn("Tilt Click Enabled is False")#, log_name_list = self.log_name_list)


  def selectTopicCb(self,msg):
    selected_pan_tilt = msg.data
    if selected_pan_tilt in self.available_pan_tilts:
      self.selected_pan_tilt = selected_pan_tilt
      self.publish_status()
      if self.node_if is not None:
        self.msg_if.pub_warn("selected_pan_tilt: " + str(selected_pan_tilt))
        self.node_if.set_param('selected_pan_tilt', selected_pan_tilt)
    

  def updaterCb(self,timer):
    #self.msg_if.pub_warn("Updater Called")
    if self.pt_connect_if is not None:
      current_position = self.pt_connect_if.get_pan_tilt_position()
      #self.msg_if.pub_warn("current_position: " + str(current_position))

    """ 
    self.pan_deg = self.current_position[0]
    self.error_pan_deg = self.pan_goal_deg - self.pan_deg
    self.tilt_deg = self.current_position[1]
    self.error_tilt_deg = self.tilt_goal_deg - self.tilt_deg
    """
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
        success = self.unsubscribe_topic()
    if selected_pan_tilt == 'None' and len(self.available_pan_tilts) > 0:
        self.selected_pan_tilt = self.available_pan_tilts[0]
    needs_publish = True

    if self.selected_pan_tilt in self.available_pan_tilts and self.connected_topic != selected_pan_tilt:
      success = self.subscribe_topic(self.selected_pan_tilt)

    elif self.pt_connect_if is not None:
       self.connected = self.pt_connect_if.check_connection()
       needs_publish = True
    else:
       self.connected = False

    # Get settings from param server
    if needs_publish == True:
      self.publish_status()
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)






  def subscribe_topic(self, topic):
    self.msg_if.pub_warn("Subscribe_topic Called")

    success = False
    if self.pt_connect_if is not None:
      success = self.unsubscribe_topic()

    pt_connect_if = ConnectPTXDeviceIF(namespace = topic,
                                       panTiltCb = self.panTiltCb,
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
  
  def unsubscribe_topic(self):
    self.msg_if.pub_warn("Unsubscribe_topic Called")

    success = True
    if self.pt_connect_if is not None:
      success = self.pt_connect_if.unsubscribe()
      self.connected = False
      self.connected_topic = None
      self.current_position = None
      nepi_sdk.sleep(1)
      self.pt_connect_if = None
    return success

  def panTiltCb(self, pan_deg, tilt_deg):
     self.current_position = [pan_deg, tilt_deg]
     #self.msg_if.pub_warn("PT position: " + str(self.current_position))
     

  def connectTrackSourceCb(self,timer):
    #self.msg_if.pub_info("connectTrackSourceCb called")
    track_namespace = copy.deepcopy(self.track_source_namespace)
    if track_namespace != self.track_source_connected_namespace  and self.track_source_connecting == False:
        if self.track_if is not None:
            self.track_if = None
            nepi_sdk.sleep(1)
        self.track_source_connected = False
        #self.msg_if.pub_warn("set_track_source connected False")
        self.msg_if.pub_warn("track_namespace: " + str(nepi_sdk.find_topic(track_namespace)))

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
    self.last_track_msg = nepi_utils.get_time()
    targets_dict_list = []
    targets_msg = msg.targets
    #self.msg_if.pub_warn("Got targets msg list " + str(targets_msg))
    
    for target_msg in targets_msg:
        target_dict = nepi_targets.convert_target_msg2dict(target_msg)
        targets_dict_list.append(target_dict)
        #self.msg_if.pub_warn("Added target list for name " + str(target_dict['target_name']))

    track_dict = None
    if len(targets_dict_list) > 0:
        #self.msg_if.pub_warn("Processing target list length " + str(len(targets_dict_list)))
        track_dict = nepi_targets.filter_targets_list(targets_dict_list,self.track_ordered_list,self.track_filter)
    #self.msg_if.pub_warn("Got track dict " + str(track_dict))
    self.track_dict = copy.deepcopy(track_dict)


##########################
###Image Veiwer
##########################

  def setImageTopic1Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 0
    img_topic = msg.data
    if self.num_windows == 1:
        self.single_image_topic = msg.image_topic
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('single_image_topic', msg.image_topic)
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
    if num_windows > 0 and num_windows < 5:
      self.num_windows = num_windows
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('num_windows', self.num_windows)
        self.node_if.save_config()


  ####################

 



  def processCb(self,timer):
    process_needs_update = copy.deepcopy(self.process_needs_update)
    self.process_needs_update = False


    if needs_publish == True:
      self.publish_status()
    # Set next process delay                         
    step_delay = 1
   
    nepi_sdk.start_timer_process(1.0, self.processCb, oneshot = True)

  

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


    self.status_msg.has_auto_pan = self.has_auto_pan
    self.status_msg.has_auto_tilt = self.has_auto_tilt
    self.status_msg.auto_pan_enabled = self.auto_pan_enabled
    self.status_msg.auto_tilt_enabled = self.auto_tilt_enabled
    self.status_msg.track_pan_enabled = self.track_pan_enabled
    self.status_msg.track_tilt_enabled = self.track_tilt_enabled
    self.status_msg.click_pan_enabled = self.click_pan_enabled
    self.status_msg.click_tilt_enabled = self.click_tilt_enabled

    ###
    current_position = [-999,-999]
    #self.msg_if.pub_warn("self.current_position: " + str(self.current_position))
    if self.current_position is not None:
      current_position = self.current_position

    """
        pt_status_msg = None
        if self.pt_connect_if is not None:
          pt_status_msg = self.pt_connect_if.get_status_dict()

        goto_position = [-999,-999]
        if pt_status_msg is not None:
          goto_position = [pt_status_msg.goal_pan_deg, pt_status_msg.goal_tilt_deg]

        goto_error =  [-999,-999]
        if self.current_position is not None and pt_status_msg is not None:
          goto_error = [goto_position[0]-current_position[0], goto_position[0]-current_position[0]]
    """  

    self.status_msg.current_position = current_position


    self.status_msg.scan_pan_min_deg = self.scan_pan_min
    self.status_msg.scan_pan_max_deg = self.scan_pan_max
    self.status_msg.scan_tilt_min_deg = self.scan_tilt_min
    self.status_msg.scan_tilt_max_deg = self.scan_tilt_max
    self.status_msg.track_source_namespace = self.track_source_namespace
    self.status_msg.track_source_connected = self.track_source_connected
    
    #self.status_msg.scan_speed_ratio =

    ###############
    ###Image Viewer
    ###############
    image_topics = copy.deepcopy(self.selected_image_topics)
    # for i, topic in enumerate(topics):
    #    if nepi_sdk.check_for_topic(topic) == False:
    #       topics[i] = 'None'
    self.status_msg.num_windows = self.num_windows
    if self.num_windows == 1:
        image_topics[0] = self.single_image_topic
    self.status_msg.image_topics = image_topics

    ############


    if self.node_if is not None:
      #self.msg_if.pub_warn("Publishing Status: " + str(status_msg))
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

