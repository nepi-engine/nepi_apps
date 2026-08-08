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


from nepi_app_file_pub_depthmap.msg import FilePubDepthmapStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ColorImageIF
from nepi_api.data_if import DepthMapIF
from nepi_api.data_if import ImageIF




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

    self.color_if = ColorImageIF(namespace = data_ns,
                data_product = 'color_image',
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
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
                log_name = 'depth_map',
                msg_if = self.msg_if
                )
    self.depth_map_if.wait_for_ready()
    self.depth_map_if.unregister_pubs()
    self.depth_map_if.set_image_callback('needs_update_callback', self.publish_collection)

    # ImageIF publishes the pre-rendered depth_map_image png directly (no
    # colorization / no processing), unlike DepthMapImageIF which would
    # colorize a raw depth array.
    self.depth_map_image_if = ImageIF(namespace = data_ns,
                data_product = 'depth_map_image',
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
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
