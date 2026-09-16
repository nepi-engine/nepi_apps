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
import time
import subprocess
import threading
import copy


from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from sensor_msgs.msg import Image
from nepi_app_image_viewer.msg import NepiAppImageViewerStatus
from nepi_interfaces.msg import StringArray

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_controls


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF


#########################################

# Factory Control Values

FACTORY_NUM_WINDOWS = 1
MIN_NUM_WINDOWS = 1
MAX_NUM_WINDOWS = 4
NO_TOPIC = "None"


#########################################
# Controls
#########################################

CONTROLS_NAME         = 'controls'
CONTROLS_DISPLAY_NAME = 'Image Viewer Controls'
CONTROLS_DESCRIPTION  = 'Window count and the image topic shown in each window'

# Control name per window, index 0..3. Indexed rather than rebuilt from a number
# in three separate places.
TOPIC_CONTROL_NAMES = ['image_topic_1', 'image_topic_2',
                       'image_topic_3', 'image_topic_4']

# Key order is display order: create_controls_dict iterates the init dict and
# the RUI renders controls_msg_list in that order.
CONTROLS_INIT_DICT = {

    'num_windows': {
        'type': 'Int', 'default': FACTORY_NUM_WINDOWS,
        'bounds': [MIN_NUM_WINDOWS, MAX_NUM_WINDOWS],
        'display_name': 'Windows',
        'description': 'Number of image windows shown, 1 to 4'},

    # Selection, not Menu: the option list is every Image topic currently on the
    # graph and is rebuilt once a second, so a Menu index would re-point at a
    # different camera as soon as one appeared or went away.
    'image_topic_1': {
        'type': 'Selection', 'default': NO_TOPIC, 'options': [NO_TOPIC],
        'display_name': 'Window 1 Topic',
        'description': 'Image topic shown in window 1'},

    'image_topic_2': {
        'type': 'Selection', 'default': NO_TOPIC, 'options': [NO_TOPIC],
        'display_name': 'Window 2 Topic',
        'description': 'Image topic shown in window 2'},

    'image_topic_3': {
        'type': 'Selection', 'default': NO_TOPIC, 'options': [NO_TOPIC],
        'display_name': 'Window 3 Topic',
        'description': 'Image topic shown in window 3'},

    'image_topic_4': {
        'type': 'Selection', 'default': NO_TOPIC, 'options': [NO_TOPIC],
        'display_name': 'Window 4 Topic',
        'description': 'Image topic shown in window 4'},
}


#########################################
# Node Class
#########################################

UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

class NepiImageViewerApp(object):

  FACTORY_SELECTED_TOPICS = ["None","None","None","None"]

  node_if = None
  controls_if = None

  update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ
  update_save_data_check_interval_sec = float(1)/UPDATE_SAVE_DATA_CHECK_RATE_HZ

  data_products = ["image1","image2","image3","image4"]

  available_image_topics = []
  single_image_topic = "None"
  selected_image_topics = ["None","None","None","None"]
  num_windows = 1

  #selected_image_topics = selected_image_topics
    
  #######################
  ### Node Initialization



  DEFAULT_NODE_NAME = "app_image_viewer" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()
    self.data_products_list = self.data_products


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
    # Window count and the four topic selections moved to ControlsIF, which
    # registers and persists its own param under the controls namespace. The
    # node keeps no params of its own, but it keeps its CFGS_DICT: NodeClassIF
    # builds the config IF from configs_dict alone, so <node>/save_config,
    # /reset_config and /factory_reset_config stay advertised, and a save there
    # dumps the whole node subtree -- which includes <node>/controls.
    self.PARAMS_DICT = None

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': NepiAppImageViewerStatus,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    # These five stay on the wire deliberately. The image windows themselves are
    # drawn by NepiIFImageViewersSelector, a SHARED component under nepi_rui,
    # whose own control bar publishes exactly these topics -- so removing them
    # would break the viewer this app is built around. They no longer hold state
    # of their own: each callback writes THROUGH the control set, so the controls
    # dict stays the single store and the two input paths cannot disagree.
    self.SUBS_DICT = {
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

    ready = self.node_if.wait_for_ready()

    ##############################
    # Controls. Mounted after the node's own NodeClassIF is ready and before
    # anything reads app state, because initCb below sources every value from
    # it. Given no node_if so it builds and owns its own: sharing the node's
    # would merge both registries and a generic key would silently orphan a
    # sibling's publisher (2026-07 DECISION LOG).
    self.setupControls()

    ##############################
    self.initCb(do_updates = True)


    time.sleep(1)
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)
    nepi_sdk.start_timer_process(1.0, self.statusPublishCb)
    # Give publishers time to setup
    time.sleep(1)

    #nepi_sdk.start_timer_process(self.update_image_subs_interval_sec, self.updateImageSubsThread)
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")

    #Set up node shutdown
    nepi_sdk.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_sdk.spin()







  ###################
  ## App Callbacks



  def updaterCb(self,timer):
    ######################
    topics = nepi_sdk.find_topics_by_msg('Image')
    available_image_topics = []
    for topic in topics:
      available_image_topics.append(topic)
    if available_image_topics != self.available_image_topics:
      self.available_image_topics = available_image_topics
      # The Selection option lists are discovered state, so the node owns them:
      # ControlsIF only carries whatever list it is given. 'None' stays first so
      # a selection that has left the graph falls back to it rather than to some
      # other camera's topic.
      options = [NO_TOPIC] + available_image_topics
      for control_name in TOPIC_CONTROL_NAMES:
        self.setControlOptions(control_name, options)

    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


  #######################
  ### Controls

  def setupControls(self):
    self.checkControlsInitDict()
    try:
      self.controls_if = ControlsIF(
          controls_name = CONTROLS_NAME,
          controls_display_name = CONTROLS_DISPLAY_NAME,
          controls_description = CONTROLS_DESCRIPTION,
          controls_init_dict = CONTROLS_INIT_DICT,
          controls_updated_callback = self.controlsUpdatedCb,
          pub_status = True,
          save_params = True,
          msg_if = self.msg_if,
      )
      self.controls_if.wait_for_controls_ready(timeout = 10)
    except Exception as e:
      # Degrade to None rather than failing to start: every read goes through
      # getControlValue, which falls back to the factory value, so the viewer
      # still runs at one window with no topic selected.
      self.msg_if.pub_warn("Image Viewer: controls unavailable: " + str(e))
      self.controls_if = None

  def checkControlsInitDict(self):
    # create_controls_dict drops a malformed control with a log warning rather
    # than raising, so a typo above costs one widget and nothing else says so.
    try:
      controls_dict = nepi_controls.create_controls_dict(CONTROLS_INIT_DICT)
    except Exception as e:
      self.msg_if.pub_warn("Image Viewer: could not validate controls init dict: " + str(e))
      return
    missing = [name for name in CONTROLS_INIT_DICT.keys() if name not in controls_dict.keys()]
    if len(missing) > 0:
      self.msg_if.pub_warn("Image Viewer: controls dropped at registration: " + str(missing))

  def getControlValue(self, control_name, fallback = None):
    if self.controls_if is None:
      return fallback
    value = None
    try:
      value = self.controls_if.get_control_value(control_name)
    except Exception as e:
      self.msg_if.pub_warn("Image Viewer: failed to read control " +
                           str(control_name) + ": " + str(e))
    if value is None:
      return fallback
    return value

  def setControlValue(self, control_name, value):
    if self.controls_if is None:
      return
    try:
      self.controls_if.set_control_value(control_name, value)
    except Exception as e:
      self.msg_if.pub_warn("Image Viewer: failed to write control " +
                           str(control_name) + ": " + str(e))

  def setControlOptions(self, control_name, options):
    if self.controls_if is None:
      return
    try:
      if self.controls_if.get_control_options(control_name) != options:
        self.controls_if.set_control_options(control_name, options)
    except Exception as e:
      self.msg_if.pub_warn("Image Viewer: failed to set options for control " +
                           str(control_name) + ": " + str(e))

  def applyControls(self):
    # The single point where a control value becomes running app state. Cheap
    # enough to re-run on every update, which keeps the updated callback from
    # having to know which control feeds which attribute.
    num_windows = int(self.getControlValue('num_windows', FACTORY_NUM_WINDOWS))
    if num_windows < MIN_NUM_WINDOWS:
      num_windows = MIN_NUM_WINDOWS
    if num_windows > MAX_NUM_WINDOWS:
      num_windows = MAX_NUM_WINDOWS
    self.num_windows = num_windows
    topics = []
    for control_name in TOPIC_CONTROL_NAMES:
      topics.append(str(self.getControlValue(control_name, NO_TOPIC)))
    self.selected_image_topics = topics
    # The app has always tracked window 1's topic twice: once in the four-entry
    # list and once on its own, with publish_status preferring the standalone
    # value while num_windows is 1. One control feeds both so they cannot drift.
    self.single_image_topic = topics[0]

  def controlsUpdatedCb(self, control_name):
    # Called by ControlsIF with the control name AFTER its dict is updated and
    # its status published.
    self.applyControls()
    if self.node_if is not None:
      # Matches what the removed set_param calls did: persist on change. The
      # config IF debounces this onto its own timer.
      self.node_if.save_config()
    self.publish_status()


  #######################
  ### Node Control Callbacks
  #
  # The shared image viewer component still publishes these. Each one writes
  # THROUGH the control set rather than assigning app state directly, so the
  # controls dict remains the single store: set_control_value publishes controls
  # status, calls controlsUpdatedCb, and persists, and applyControls is what
  # refreshes self.num_windows / self.selected_image_topics.

  def setImageTopicCb(self, msg, img_index):
    if img_index < 0 or img_index >= len(TOPIC_CONTROL_NAMES):
      return
    self.setControlValue(TOPIC_CONTROL_NAMES[img_index], msg.data)

  def setImageTopic1Cb(self,msg):
    self.setImageTopicCb(msg, 0)

  def setImageTopic2Cb(self,msg):
    self.setImageTopicCb(msg, 1)

  def setImageTopic3Cb(self,msg):
    self.setImageTopicCb(msg, 2)

  def setImageTopic4Cb(self,msg):
    self.setImageTopicCb(msg, 3)

  def setNumWindowsCb(self,msg):
    num_windows = msg.data
    if num_windows >= MIN_NUM_WINDOWS and num_windows <= MAX_NUM_WINDOWS:
      self.setControlValue('num_windows', num_windows)


  #######################
  ### Config Functions

  def initCb(self,do_updates = False):
    # Runs twice at startup: once from NodeClassIF's init_configs, before
    # ControlsIF exists, and once explicitly after setupControls. The first pass
    # falls back to the factory values, the second picks up whatever the config
    # manager restored.
    self.applyControls()
    if do_updates == True:
      pass
    self.publish_status()

  def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Reseting")
      # ControlsIF owns its own config tier under its own namespace, so the app
      # level reset has to hand the reset down to it or the controls keep their
      # current values while the rest of the app resets.
      if self.controls_if is not None:
        try:
          self.controls_if.reset()
        except Exception as e:
          self.msg_if.pub_warn("Image Viewer: controls reset failed: " + str(e))
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      if self.controls_if is not None:
        try:
          self.controls_if.factory_reset()
        except Exception as e:
          self.msg_if.pub_warn("Image Viewer: controls factory reset failed: " + str(e))
      self.initCb(do_updates = do_updates)


  ###################
  ## Status Publishers

  def statusPublishCb(self,timer):
      self.publish_status()

  def publish_status(self):
    topics = copy.deepcopy(self.selected_image_topics)
    # for i, topic in enumerate(topics):
    #    if nepi_sdk.check_for_topic(topic) == False:
    #       topics[i] = 'None'
    status_msg = NepiAppImageViewerStatus()     
    image_topics = copy.deepcopy(self.selected_image_topics)
    status_msg.num_windows = self.num_windows
    if self.num_windows == 1:
        image_topics[0] = self.single_image_topic
    for i, topic in enumerate(image_topics):
       if topic != 'None':
          if topic not in self.available_image_topics:
             image_topics[i] = 'None'
    status_msg.selected_image_topics = image_topics
    if self.node_if is not None:
      self.node_if.publish_pub('status_pub',status_msg)


             

  #######################
  # Utility Funcitons   
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("IMG_VIEW_APP:  Shutting down: Executing script cleanup actions")
    if self.controls_if is not None:
      try:
        self.controls_if.unregister()
      except Exception:
        pass
      self.controls_if = None


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiImageViewerApp()





