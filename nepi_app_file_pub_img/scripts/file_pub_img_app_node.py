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
import open3d as o3d
import random
import copy
import threading



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img 
from nepi_sdk import nepi_controls


from nepi_app_file_pub_img.msg import FilePubImgStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ColorImageIF
from nepi_api.system_if import ControlsIF


#########################################
# Controls
#########################################

CONTROLS_NAME         = 'controls'
CONTROLS_DISPLAY_NAME = 'Image File Publisher Controls'
CONTROLS_DESCRIPTION  = 'Playback, image format and overlay settings'

# Button controls, keyed here so the updated callback can tell a command press
# from a value edit without restating the names inline.
BUTTON_CONTROLS = ['start_pub', 'stop_pub', 'step_forward', 'step_backward']

# Row widths for the grouped playback line.
_ROW_VALUE_WIDTH = 70


class ControlValue:
  """Stand-in for the std_msgs value the app's setter callbacks expect.

  The setters kept their original signatures through this migration, so the
  control routes hand them an object with a .data attribute exactly as the ROS
  subscribers did. Nothing about their per-field clamping moved.
  """

  def __init__(self, data):
    self.data = data


def build_controls_init_dict(size_options, encoding_options,
                             factory_size, factory_encoding,
                             min_rate, max_rate, factory_rate):
  """Build the app's control set.

  Key order is display order: create_controls_dict iterates the init dict and
  the RUI renders controls_msg_list in that order. Controls sharing a non-empty
  display_group render on ONE horizontal line, in that same order.

  Args:
      size_options: image size strings the node accepts.
      encoding_options: image encodings the node accepts.
      factory_size: factory image size.
      factory_encoding: factory image encoding.
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
        'description': 'Start publishing images from the current folder'},

    'stop_pub': {
        'type': 'Button',
        'display_name': 'Stop Publishing', 'display_group': 'publish',
        'description': 'Stop publishing and release the image publishers'},

    # One row: the pause toggle, the rate box and the random toggle. Rate and
    # Random hide while paused and the two step Buttons appear -- see
    # syncControlVisibility, which mirrors what the hand-written panel did.
    'paused': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Paused', 'display_group': 'playback',
        'description': 'Hold on the current image instead of advancing'},

    'rate_hz': {
        'type': 'Float', 'default': float(factory_rate),
        'bounds': [min_rate, max_rate],
        'round': 2, 'display_round': 2,
        'display_name': 'Rate (Hz)', 'display_group': 'playback',
        'display_width': _ROW_VALUE_WIDTH,
        'description': 'Rate images are published at while not paused'},

    'random': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Random', 'display_group': 'playback',
        'description': 'Pick the next image at random rather than in order'},

    'step_forward': {
        'type': 'Button',
        'display_name': 'Forward', 'display_group': 'step',
        'display_hidden': True,
        'description': 'While paused, advance one image'},

    'step_backward': {
        'type': 'Button',
        'display_name': 'Back', 'display_group': 'step',
        'display_hidden': True,
        'description': 'While paused, go back one image'},

    # Selection, not Menu: a Menu value is the INDEX into its option list, so a
    # reordered list would silently re-point the stored size at a different one.
    'size': {
        'type': 'Selection', 'default': factory_size,
        'options': list(size_options),
        'display_name': 'Image Size',
        'description': 'Size published images are resized to'},

    'encoding': {
        'type': 'Selection', 'default': factory_encoding,
        'options': list(encoding_options),
        'display_name': 'Image Encoding',
        'description': 'Encoding published images are converted to'},

    'overlay': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Overlay Filename',
        'description': 'Draw the source filename on each published image'},
  }


#########################################
# Node Class
#########################################

class NepiFilePubImgApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  SUPPORTED_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']

  #Set Initial Values
  MIN_RATE = 0.1
  MAX_RATE = 20
  FACTORY_IMG_PUB_RATE = 1.0
  MIN_SIZE = 240
  MAX_SIZE = 3700
  STANDARD_IMAGE_SIZES = ['Original','240 x 320', '480 x 640', '630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']
  FACTORY_IMG_SIZE = '630 x 900'
  IMG_PUB_ENCODING_OPTIONS = ["bgr8","rgb8","mono8"]
  FACTORY_IMG_ENCODING_OPTION = "bgr8" 

  UPDATER_DELAY_SEC = 1.0
  
  node_if = None
  controls_if = None
  
  if os.path.exists(HOME_FOLDER + '/sample_data'):
    current_folder = HOME_FOLDER + '/sample_data'
  else:
    current_folder = HOME_FOLDER

  last_folder = ""
  current_folders = []
  current_file = 'None'
  current_ind = 0
  last_folder = ""
  file_count = 0


  image_if = None

  paused = False
  oneshot_offset = 1
  
  width = 0
  height = 0

  width_deg = 100
  height_deg = 70


  size = FACTORY_IMG_SIZE
  encoding = FACTORY_IMG_ENCODING_OPTION
  random = False
  overlay = False
  rate = FACTORY_IMG_PUB_RATE
  running = False

  restart = False
  update_img = False

  file2open = None
  cv2_img = None
  cv2_lock = threading.Lock()


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_img" # Can be overwitten by luanch command
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
    # size, encoding, random, overlay and rate all moved to ControlsIF, which
    # registers and persists its own param under the controls namespace. The two
    # that stay are node-WRITTEN state rather than operator-typed values:
    # current_folder is set by the folder navigation commands, and running is a
    # side effect of start/stop that seeds the restart on the next launch.
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
            'msg': FilePubImgStatus,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    # What stays here are the COMMANDS. Folder navigation carries a relative
    # name plus a traversal verb, which a control set -- where each value is
    # written independently -- cannot express atomically, and start/stop are the
    # app's programmatic publishing API. The Button controls call the same
    # private methods these callbacks do; neither path reimplements the other.
    #
    # Removed here and now driven over <node>/controls/update_control:
    # set_size, set_encoding, set_rate, set_random, set_overlay, pause_pub,
    # step_forward and step_backward.
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
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT
    )

    self.node_if.wait_for_ready()

    ##############################
    # Controls. Mounted after the node's own NodeClassIF is ready and before
    # anything reads app state, because initCb below sources every value from
    # it. Like ColorImageIF it is given no node_if and builds its own: sharing
    # the node's would merge both registries and a generic key would silently
    # orphan a sibling's publisher (2026-07 DECISION LOG).
    self.setupControls()

  
    image_ns = self.node_namespace
    data_product = 'color_image'
    self.image_ns = os.path.join(image_ns,data_product)
    self.image_if = ColorImageIF(namespace = image_ns, 
                data_product = data_product, 
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
                log_name = data_product,
                msg_if = self.msg_if
                )
    ready = self.image_if.wait_for_ready()
    self.image_if.unregister_pubs()
    self.image_if.set_image_callback('needs_update_callback', self.publish_img)

    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Start updater process

    
    nepi_sdk.start_timer_process(self.UPDATER_DELAY_SEC, self.updaterCb)
    nepi_sdk.start_timer_process(1, self.imagePublishCb, oneshot = True)
    nepi_sdk.start_timer_process(1.0, self.statusPublishCb)



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
    # Runs twice at startup: once from NodeClassIF's init_configs, before
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
    # setSize parses the selected size into self.width / self.height; the
    # control only carries the string.
    self.setSize(self.size)
    self.syncControlVisibility()
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
          self.msg_if.pub_warn("File Pub Img: controls reset failed: " + str(e))
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      if self.controls_if is not None:
        try:
          self.controls_if.factory_reset()
        except Exception as e:
          self.msg_if.pub_warn("File Pub Img: controls factory reset failed: " + str(e))
      self.initCb(do_updates = do_updates)


  #######################
  ### Controls

  def setupControls(self):
    self.controls_init_dict = build_controls_init_dict(
        self.STANDARD_IMAGE_SIZES, self.IMG_PUB_ENCODING_OPTIONS,
        self.FACTORY_IMG_SIZE, self.FACTORY_IMG_ENCODING_OPTION,
        self.MIN_RATE, self.MAX_RATE, self.FACTORY_IMG_PUB_RATE)
    self.controls_routes = self.controlRoutes()
    self.checkControlsInitDict()
    try:
      self.controls_if = ControlsIF(
          controls_name = CONTROLS_NAME,
          controls_display_name = CONTROLS_DISPLAY_NAME,
          controls_description = CONTROLS_DESCRIPTION,
          controls_init_dict = self.controls_init_dict,
          controls_updated_callback = self.controlsUpdatedCb,
          pub_status = True,
          save_params = True,
          msg_if = self.msg_if,
      )
      self.controls_if.wait_for_controls_ready(timeout = 10)
    except Exception as e:
      # Degrade to None rather than failing to start: every read goes through
      # getControlValue, which falls back to the factory value, so the app still
      # publishes at factory settings.
      self.msg_if.pub_warn("File Pub Img: controls unavailable: " + str(e))
      self.controls_if = None

  def checkControlsInitDict(self):
    # create_controls_dict drops a malformed control with a log warning rather
    # than raising, so a typo in the init dict costs one widget and nothing else
    # says so. Run it here first and name what went missing.
    try:
      controls_dict = nepi_controls.create_controls_dict(self.controls_init_dict)
    except Exception as e:
      self.msg_if.pub_warn("File Pub Img: could not validate controls init dict: " + str(e))
      return
    missing = [name for name in self.controls_init_dict.keys() if name not in controls_dict.keys()]
    if len(missing) > 0:
      self.msg_if.pub_warn("File Pub Img: controls dropped at registration: " + str(missing))

  def controlRoutes(self):
    # control name -> the app callback that already owns that value. Routing
    # rather than reimplementing is what keeps setSize's parse-and-range-check
    # and setRateCb's clamp exactly where they were.
    return {
        'paused':     self.pausePubCb,
        'rate_hz':    self.setRateCb,
        'random':     self.setRandomCb,
        'size':       self.setSizeCb,
        'encoding':   self.setEncodingCb,
        'overlay':    self.setOverlayCb,
    }

  def getControlValue(self, control_name, fallback = None):
    if self.controls_if is None:
      return fallback
    value = None
    try:
      value = self.controls_if.get_control_value(control_name)
    except Exception as e:
      self.msg_if.pub_warn("File Pub Img: failed to read control " +
                           str(control_name) + ": " + str(e))
    if value is None:
      return fallback
    return value

  def setControlHidden(self, control_name, hidden):
    if self.controls_if is None:
      return
    try:
      self.controls_if.set_control_hidden(control_name, hidden)
    except Exception:
      pass

  def applyControls(self):
    # The single point where a control value becomes running app state.
    self.paused = bool(self.getControlValue('paused', False))
    self.rate = float(self.getControlValue('rate_hz', self.FACTORY_IMG_PUB_RATE))
    self.random = bool(self.getControlValue('random', False))
    self.size = str(self.getControlValue('size', self.FACTORY_IMG_SIZE))
    self.encoding = str(self.getControlValue('encoding', self.FACTORY_IMG_ENCODING_OPTION))
    self.overlay = bool(self.getControlValue('overlay', False))

  def syncControlVisibility(self):
    # Mirrors what the hand-written panel did: rate and random are shown while
    # running forward, the two step Buttons while paused. Re-derived after every
    # update rather than special-casing the pause control's name.
    paused = (self.paused == True)
    self.setControlHidden('rate_hz', paused)
    self.setControlHidden('random', paused)
    self.setControlHidden('step_forward', paused == False)
    self.setControlHidden('step_backward', paused == False)

  def controlsUpdatedCb(self, control_name):
    # Called by ControlsIF with the control name AFTER its dict is updated and
    # its status published.
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

    # applyControls runs after the route so an in-callback clamp (setRateCb) is
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
    self.update_img = True
    self.publish_status()

  def stepForwardPubCb(self,msg):
    if self.paused:
      self.oneshot_offset = 1

  def stepBackwardPubCb(self,msg):
    if self.paused:
      self.oneshot_offset = -1



  #############################
  ## Image callbacks

  def setSizeCb(self,msg):
    size_str = msg.data
    self.setSize(size_str)
  
  def setSize(self,size_str):
    # The control persists the size string; this only parses it into the
    # width/height the resize path uses, and range-checks it as it always did.
    new_size = size_str
    success = False
    if new_size == 'Original':
      self.size = new_size
    else:
      try:
        size_list = new_size.split("x")
        h = int(size_list[0])
        w = int(size_list[1])
        success = True
      except Exception as e:
        self.msg_if.pub_warn( "Unable to parse size message: " + new_size + " " + str(e) )

      if success:
        if h >= self.MIN_SIZE and h <= self.MAX_SIZE and w >= self.MIN_SIZE and w <= self.MAX_SIZE:
          self.size = new_size
          self.publish_status()
          self.width = w
          self.height = h
          self.msg_if.pub_warn( "Received width,height to: " + str([self.width,self.height]) )
        else:
          self.msg_if.pub_warn( "Received size out of range: " + new_size )
    self.publish_status()

  def setEncodingCb(self,msg):
    new_encoding = msg.data
    if new_encoding in self.IMG_PUB_ENCODING_OPTIONS:
      self.encoding = new_encoding
    self.publish_status()

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


  def updateFolderInfo(self, folder):
    if folder != self.last_folder:
      self.stopPub()
      #self.paused = True

      if os.path.exists(folder):
        self.current_folder = folder
        #self.msg_if.pub_warn("Current Folder Exists")
        current_paths = nepi_utils.get_folder_list(folder)
        current_folders = []
        for path in current_paths:
          current_folders.append(os.path.basename(path))
        self.current_folders = sorted(current_folders)
        # self.publish_status()
        self.msg_if.pub_warn("Folders: " + str(self.current_folders))
        num_files = 0
        for f_type in self.SUPPORTED_FILE_TYPES:
          file_count = nepi_utils.get_file_count(folder, ext_str = f_type)
          self.msg_if.pub_warn("Add file type count to current count: " + " : " + str(folder) + " : " + str(f_type) + " : " + str(file_count) + " : " + str(num_files))
          num_files += file_count
        self.file_count =  num_files
        self.msg_if.pub_warn("File Count: " + str(self.file_count))
        # self.publish_status()
        if self.file_count > 0:
          self.startPub()
    self.last_folder = copy.deepcopy(self.current_folder)

  def updaterCb(self,timer):
    update_status = False
    # Get settings from param server
    current_folder = copy.deepcopy(self.current_folder)
    #self.msg_if.pub_warn("Current Folder: " + str(current_folder))
    #self.msg_if.pub_warn("Last Folder: " + str(self.last_folder))
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


  def img_needs_update(self):
    #self.msg_if.pub_info('Got update image request')
    self.update_img = True

  def startPubCb(self,msg):
    self.msg_if.pub_info('Got start publishing msg: ' + str(msg))
    self.startPub()

  def startPub(self):
    self.msg_if.pub_warn("Start Pub Called")


    current_folder = self.current_folder
    # Now start publishing images
    self.file_list = []
    self.num_files = 0
    if os.path.exists(current_folder):
      for f_type in self.SUPPORTED_FILE_TYPES:
        [file_list, num_files] = nepi_utils.get_file_list(current_folder,f_type)
        self.file_list.extend(file_list)
        self.num_files += num_files
        #self.msg_if.pub_warn("File Pub List: " + str(self.file_list))
        #self.msg_if.pub_warn("File Pub Count: " + str(self.num_files))
      if self.num_files > 0:
        if self.image_if is not None:
          self.msg_if.pub_warn("Registering Image IF pubs")
          self.image_if.register_pubs()
        self.current_ind = 0
        self.running = True
        self.msg_if.pub_warn("Set Running to True")
        self.publish_status()
        if self.node_if is not None:
          self.node_if.set_param('running',True)
      else:
        self.msg_if.pub_info("No image files found in folder " + current_folder + " not found")
    else:
      self.msg_if.pub_info("Folder " + current_folder + " not found")
    self.publish_status()

  def stopPubCb(self,msg):
    self.stopPub()

  def stopPub(self):
    self.running = False
    self.cv2_lock.acquire()
    self.cv2_img = None
    self.cv2_lock.release()
    self.publish_status()
    self.current_file = "None"
    if self.image_if is not None:
      self.msg_if.pub_warn("Unregistering Image IF pubs")
      self.image_if.unregister_pubs()
    
    if self.node_if is not None:
      self.node_if.set_param('running',False)
    


  def imagePublishCb(self,timer):
    running = self.running
    size = self.size
    encoding = self.encoding
    set_random = self.random
    overlay = self.overlay
    oneshot_offset = copy.deepcopy(self.oneshot_offset)
    self.oneshot_offset = 0
    current_ind = copy.deepcopy(self.current_ind)
    
    if self.paused:
      step = oneshot_offset
    else:
      step = 1
    if running and (step != 0 or self.update_img == True):
      #self.msg_if.pub_info("Will Publishing")
      self.update_img = False
      #self.msg_if.pub_info("Start index and onshot: " + str([current_ind,oneshot_offset]))
      #self.msg_if.pub_info("running")
      if self.image_if != None and len(self.file_list) > 0:
        #self.msg_if.pub_info("pass check")
        # Set current index
        
        if set_random == True and self.paused == True and step != 0:
          current_ind = int(random.random() * self.num_files)
        else:
          current_ind += step
        # Check ind bounds
        if current_ind > (len(self.file_list)-1):
          current_ind = 0 # Start over
        elif current_ind < 0:
          current_ind = self.num_files-1
        
        #self.msg_if.pub_info("pub index: " + str([current_ind]))
        file2open = self.file_list[current_ind]
        self.current_file = file2open.split('/')[-1]
        self.current_ind = current_ind
        #self.msg_if.pub_info("Opening File: " + file2open)
        cv2_img = None
        if file2open != self.file2open:
          cv2_img = cv2.imread(file2open)
          self.file2open = file2open
        #self.msg_if.pub_info("pub file: " + str(file2open))
        if cv2_img is not None:
          if self.size != 'Original' and self.width > 100 and self.height > 100:
            cv2_img = cv2.resize(cv2_img,(self.width,self.height))
          else:
            [self.height,self.width] = cv2_img.shape[0:2]

          # Overlay Label
          if overlay == True:
            # Overlay text data on OpenCV image
            font                   = cv2.FONT_HERSHEY_DUPLEX
            fontScale, thickness  = nepi_img.get_optimal_font_dims(cv2_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
            fontColor = (0, 255, 0)
            lineType = 1
            text2overlay=self.current_file
            bottomLeftCornerOfText = (int(self.width*.05),int(self.height*.1))
            cv2.putText(cv2_img,text2overlay, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                thickness,
                lineType)
          img_shape = cv2_img.shape     
          if encoding == 'mono8' and img_shape[2] == 3:
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
          if encoding != 'mono8' and img_shape[2] == 1:
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_GRAY2BGR)
          self.cv2_lock.acquire()
          self.cv2_img = cv2_img
          self.cv2_lock.release()

          self.publish_img()
          #self.publish_status()


    delay = 0.1
    running = self.running
    if running == True and self.paused == False:
        delay = 1.0 / self.rate
    

    #self.msg_if.pub_info("Delay: " + str(delay)) 
    nepi_sdk.start_timer_process(delay, self.imagePublishCb, oneshot = True)



  def publish_img(self):
      #self.msg_if.pub_info("Got pub img request") 
      if self.cv2_img is not None:
        encoding = self.encoding

        if self.image_if is not None:
          self.cv2_lock.acquire()
          self.image_if.publish_cv2_img(self.cv2_img, encoding = encoding,
                                          width_deg = self.width_deg,
                                          height_deg = self.height_deg,
                                          pub_twice = self.paused)
          self.cv2_lock.release()
        #self.msg_if.pub_info("Published")




  def statusPublishCb(self,timer):
    self.publish_status()

             ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = FilePubImgStatus()

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
    file = self.current_file
    status_msg.current_file =  file

    status_msg.paused = self.paused

    status_msg.size_options_list = self.STANDARD_IMAGE_SIZES
    status_msg.set_size = self.size
    status_msg.encoding_options_list = self.IMG_PUB_ENCODING_OPTIONS
    status_msg.set_encoding = self.encoding 


    status_msg.file_count = self.file_count
    status_msg.set_random = self.random 
    status_msg.set_overlay = self.overlay 
    status_msg.min_max_rate = [self.MIN_RATE, self.MAX_RATE]
    status_msg.set_rate = self.rate 
    status_msg.running = self.running
    if self.node_if is not None:
      self.node_if.publish_pub('status_pub', status_msg)

   
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info(" Shutting down: Executing script cleanup actions")
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
  NepiFilePubImgApp()







