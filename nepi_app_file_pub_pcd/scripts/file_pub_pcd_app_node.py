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
import cv2
import open3d as o3d
import yaml

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_pc 

from nepi_app_file_pub_pcd.msg import FilePubPcdStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import PointCloud2

from nepi_interfaces.msg import Frame3DTransform, Frame3DTransformUpdate


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import PointcloudIF




#########################################
# Node File
#########################################

class NepiFilePubPcdApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  SUPPORTED_FILE_TYPES = ['pcd']

  #Set Initial Values
  MAX_FILES = 20
  MAX_PUBS = 5
  MIN_DELAY = 0.05
  MAX_DELAY = 5.0
  FACTORY_IMG_PUB_DELAY = 1.0

  UPDATER_DELAY_SEC = 1.0

  ZERO_TRANSFORM_DICT = {
    'x_m': 0, 
    'y_m': 0,
    'z_m': 0,
    'roll_deg': 0,
    'pitch_deg': 0,
    'yaw_deg': 0,
    'heading_deg': 0,
  }
  
  node_if = None

  paused = False

  last_folder = ""
  current_folders = []
  current_file_list = []
  current_topic_list = []

  available_files_list = []
  sel_all = False

  running = False
  file_count = 0
  pc_if = None

  pcds_dict = dict()
  tf_subs_list = []



  current_folder = self.HOME_FOLDER
  sel_files = []
  delay = self.FACTORY_IMG_PUB_DELAY
  pub_transforms = False
  create_transforms = False
  running = False

    

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_pcd" # Can be overwitten by luanch command
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
        'sel_files': {
            'namespace': self.node_namespace,
            'factory_val': []
        },
        'delay': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_IMG_PUB_DELAY
        },
        'pub_transforms': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'create_transforms': {
            'namespace': self.node_namespace,
            'factory_val': False
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
            'msg': FilePubPcdStatus,
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
            'qsize': 10,
            'callback': self.selectFolderCb, 
            'callback_args': ()
        },
        'home_folder': {
            'namespace': self.node_namespace,
            'topic': 'home_folder',
            'msg': Empty,
            'qsize': 10,
            'callback': self.homeFolderCb, 
            'callback_args': ()
        },
        'back_folder': {
            'namespace': self.node_namespace,
            'topic': 'back_folder',
            'msg': Empty,
            'qsize': 10,
            'callback': self.backFolderCb, 
            'callback_args': ()
        },
        'add_all_pcd_files': {
            'namespace': self.node_namespace,
            'topic': 'add_all_pcd_files',
            'msg': Empty,
            'qsize': 10,
            'callback': self.addAllFilesCb, 
            'callback_args': ()
        },
        'remove_all_pcd_files': {
            'namespace': self.node_namespace,
            'topic': 'remove_all_pcd_files',
            'msg': Empty,
            'qsize': 10,
            'callback': self.removeAllFilesCb, 
            'callback_args': ()
        },
        'add_pcd_file': {
            'namespace': self.node_namespace,
            'topic': 'add_pcd_file',
            'msg': String,
            'qsize': 10,
            'callback': self.addFileCb, 
            'callback_args': ()
        },        
        'remove_pcd_file': {
            'namespace': self.node_namespace,
            'topic': 'remove_pcd_file',
            'msg': String,
            'qsize': 1,
            'callback': self.removeFileCb, 
            'callback_args': ()
        },
        'set_delay': {
            'namespace': self.node_namespace,
            'topic': 'set_delay',
            'msg': Float32,
            'qsize': None,
            'callback': self.setDelayCb, 
            'callback_args': ()
        },
        'set_pub_transforms': {
            'namespace': self.node_namespace,
            'topic': 'set_pub_transforms',
            'msg': Bool,
            'qsize': None,
            'callback': self.setPubTransformsCb, 
            'callback_args': ()
        },
        'set_create_transforms': {
            'namespace': self.node_namespace,
            'topic': 'set_create_transforms',
            'msg': Bool,
            'qsize': None,
            'callback': self.setCreateTransformsCb, 
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
    self.initCb(do_updates = True)
    # Start updater process
    nepi_sdk.start_timer_process(self.UPDATER_DELAY_SEC, self.updaterCb)


    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")
    self.publish_status()
    # Spin forever (until object is detected)
    nepi_sdk.spin()

  #######################
  ### App Config Functions


  def initCb(self,do_updates = False):
      if self.node_if is not None:
        #current_folder = self.node_if.get_param('current_folder')
        #if os.path.exists(current_folder) == False:
        #  current_folder = self.HOME_FOLDER
        #self.current_folder = current_folder
        pass

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


  def setCurrentAsDefault(self):
    self.initCb(do_updates = False)


  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = FilePubPcdStatus()

    status_msg.home_folder = self.HOME_FOLDER
    current_folder = self.node_if.get_param('current_folder')
    status_msg.current_folder = current_folder
    if current_folder == self.HOME_FOLDER:
      selected_folder = 'Home'
    else:
      selected_folder = os.path.basename(current_folder)
    status_msg.selected_folder = selected_folder
    status_msg.current_folders = self.current_folders
    status_msg.supported_file_types = self.SUPPORTED_FILE_TYPES
    status_msg.file_count = self.file_count

    status_msg.max_files = self.MAX_FILES
    status_msg.available_files_list = self.available_files_list
    status_msg.selected_files_list = self.node_if.get_param('sel_files')

    status_msg.current_file_list = self.current_file_list
    status_msg.current_topic_list = self.current_topic_list


    status_msg.max_pubs = self.MAX_PUBS
    status_msg.min_max_delay = [self.MIN_DELAY, self.MAX_DELAY]
    status_msg.set_delay = self.node_if.get_param('delay')

    status_msg.pub_transforms = self.node_if.get_param('pub_transforms')
    status_msg.create_transforms = self.node_if.get_param('create_transforms')

    status_msg.running = self.node_if.get_param('running')

    if self.node_if is not None:
      self.node_if.publish_pub('status_pub', status_msg)


  #############################
  ## APP callbacks

  def updaterCb(self,timer):
    update_status = False
    # Get settings from param server
    current_folder = self.node_if.get_param('current_folder')
    #self.msg_if.pub_warn("Current Folder: " + str(current_folder))
    #self.msg_if.pub_warn("Last Folder: " + str(self.last_folder))
    # Update folder info
    if current_folder != self.last_folder:
      self.stopPub()
      update_status = True
      if os.path.exists(current_folder):
        #self.msg_if.pub_warn("Current Folder Exists")
        current_paths = nepi_utils.get_folder_list(current_folder)
        current_folders = []
        for path in current_paths:
          folder = os.path.basename(path)
          if folder[0] != ".":
            current_folders.append(folder)
        self.current_folders = sorted(current_folders)
        #self.msg_if.pub_warn("Folders: " + str(self.current_folders))
        num_files = 0
        for f_type in self.SUPPORTED_FILE_TYPES:
          num_files = num_files + nepi_utils.get_file_count(current_folder,f_type)
        self.file_count =  num_files
      self.last_folder = current_folder

      if self.sel_all == True:
        self.addAllFiles()
      # Now start publishing images
      self.file_list = []
      self.num_files = 0
      if os.path.exists(current_folder):
        for ind, f_type in enumerate(self.SUPPORTED_FILE_TYPES):
          [file_list, num_files] = nepi_utils.get_file_list(current_folder,f_type)
          self.file_list.extend(file_list)
          self.num_files += num_files
          #self.msg_if.pub_warn("File Pub List: " + str(self.file_list))
          #self.msg_if.pub_warn("File Pub Count: " + str(self.num_files))
        if self.num_files > self.MAX_FILES: 
          self.available_files_list = file_list[:self.MAX_FILES] # Take first MAX_PUBS files
        else:
          self.available_files_list = file_list
        update_sel_files = []
        for count, sel_file in enumerate(file_list):
          if sel_file in self.available_files_list and count < self.MAX_PUBS:
            update_sel_files.append(sel_file)
        self.node_if.set_param('sel_files', update_sel_files)
        
    # Start publishing if needed
    running = self.node_if.get_param('running')
    if running == False:
      self.startPub()
      update_status = True
    # Publish status if needed
    if update_status == True:
      self.publish_status()

  def selectFolderCb(self,msg):
    current_folder = self.node_if.get_param('current_folder')
    new_folder = msg.data
    new_path = os.path.join(current_folder,new_folder)
    if os.path.exists(new_path):
      self.last_folder = current_folder
      self.node_if.set_param('current_folder',new_path)
    self.sel_all = True
    self.publish_status()


  def homeFolderCb(self,msg):
    self.node_if.set_param('current_folder',self.HOME_FOLDER)
    self.sel_all = True
    self.publish_status()

  def backFolderCb(self,msg):
    current_folder = self.node_if.get_param('current_folder')
    if current_folder != self.HOME_FOLDER:
      new_folder = os.path.dirname(current_folder )
      if os.path.exists(new_folder):
        self.last_folder = current_folder
        self.node_if.set_param('current_folder',new_folder)
        self.sel_all = True
    self.publish_status()






  #############################
  ## Callbacks



  def addAllFilesCb(self,msg):
    self.addAllFiles()

  def addAllFiles(self):
    ##self.msg_if.pub_info(msg)
    sel_files = self.available_files_list
    if len(sel_files) > self.MAX_PUBS:
      sel_files = sel_files[:self.MAX_PUBS]
    self.node_if.set_param('sel_files', sel_files)
    self.publish_status()

  def removeAllFilesCb(self,msg):
    ##self.msg_if.pub_info(msg)
    self.node_if.set_param('sel_files', [])
    self.publish_status()

  def addFileCb(self,msg):
    sel_files = self.node_if.get_param('sel_files')
    ##self.msg_if.pub_info(msg)
    file_name = msg.data
    if len(sel_files) < self.MAX_PUBS:
      if file_name in self.available_files_list:
        sel_files.append(file_name)
    self.node_if.set_param('sel_files', sel_files)
    self.publish_status()

  def removeFileCb(self,msg):
    sel_files = self.node_if.get_param('sel_files')
    ##self.msg_if.pub_info(msg)
    file_name = msg.data
    if file_name in sel_files:
      sel_files.remove(file_name)
    self.node_if.set_param('sel_files', sel_files)
    self.publish_status()


  def pausePubCb(self,msg):
    ##self.msg_if.pub_info(msg)
    self.paused = msg.data
    self.publish_status()


  def setDelayCb(self,msg):
    ##self.msg_if.pub_info(msg)
    delay = msg.data
    if delay < self.MIN_DELAY:
      delay = self.MIN_DELAY
    if delay > self.MAX_DELAY:
      delay = self.MAX_DELAY
    self.node_if.set_param('delay',delay)
    self.publish_status()

  def setPubTransformsCb(self,msg):
    val = msg.data
    self.node_if.set_param('pub_transforms',  val)
    self.publish_status()

  def setCreateTransformsCb(self,msg):
    val = msg.data
    self.node_if.set_param('create_transforms',  val)
    self.publish_status()

  def startPubCb(self,msg):
    self.startPub()

  def startPub(self):
    create_tfs = self.node_if.get_param('create_transforms')
    current_folder = self.node_if.get_param('current_folder')
    sel_files = self.node_if.get_param('sel_files')
    if self.node_if.get_param('running') == False:
      self.current_file_list = []
      self.current_topic_list = []
      self.pcds_dict = dict()
      for pcd_filename in sel_files:
        pcd_file = os.path.join(current_folder,pcd_filename)
        if os.path.exists(pcd_file):
          pcd_name = os.path.basename(pcd_file)
          o3d_pc = None
          try:
            o3d_pc = o3d.io.read_point_cloud(pcd_file)
            pcd_topic_name = os.path.basename(pcd_file).split('.')[0]
            pcd_topic_name = pcd_topic_name.replace('-','_')
            pcd_topic_name = pcd_topic_name.replace('.','')
            pcd_namespace = os.path.join(self.base_namespace,self.node_name,pcd_topic_name)
          except Exception as e:
            self.msg_if.pub_warn("Failed to read pointcloud from file: " + pcd_file + " " + str(e))
          
          if o3d_pc != None:
            self.pcds_dict[pcd_name] = dict()
            self.pcds_dict[pcd_name]['file'] = pcd_file 
            self.pcds_dict[pcd_name]['topic'] = pcd_topic_name
            self.pcds_dict[pcd_name]['o3d_pc'] = o3d_pc
            self.msg_if.pub_info("creating publisher for file: " + pcd_file)
            pc_if = PointcloudIF(namespace = self.node_namespace, topic = pcd_topic_name)
            self.pcds_dict[pcd_name]['pc_if'] = pc_if
            self.current_file_list.append(pcd_name)
            self.current_topic_list.append(os.path.join(self.node_name,pcd_topic_name))
            # Process Transform Data
            tf_dict = self.ZERO_TRANSFORM_DICT

            transform_file = pcd_file.replace('.pcd','_transform.yaml')
            if os.path.exists(transform_file):
              try:
                with open(transform_file, "r") as file:
                    tf_dict = yaml.safe_load(file)
              except Exception as e:
                tf_dict = self.ZERO_TRANSFORM_DICT
                self.msg_if.pub_warn("Failed to read transform from file: " + transform_file  + " " + str(e))
            else:
              if create_tfs:
                self.msg_if.pub_warn("No transform file found, so creating one")
                try:
                  with open(transform_file, 'w') as f:
                    yaml.dump(tf_dict, f)
                except Exception as e:
                  self.msg_if.pub_warn("Failed to write transform to file: " + transform_file + " " + str(e))
            tf_msg = Frame3DTransform()
            tf_msg.translate_vector.x =  tf_dict['x_m']
            tf_msg.translate_vector.y  =  tf_dict['y_m']
            tf_msg.translate_vector.z  =  tf_dict['z_m']
            tf_msg.rotate_vector.x =  tf_dict['roll_deg']
            tf_msg.rotate_vector.y =  tf_dict['pitch_deg']
            tf_msg.rotate_vector.z =  tf_dict['yaw_deg']
            tf_msg.heading_offset =  tf_dict['heading_deg']  
            self.pcds_dict[pcd_name]['tf_msg'] = tf_msg
            tfu_msg = Frame3DTransformUpdate()
            tfu_msg.topic_namespace = pcd_namespace
            tfu_msg.transform = tfu_msg
            self.pcds_dict[pcd_name]['tfu_msg'] = tfu_msg
            # Find transform subscribers
            for tf_sub in self.tf_subs_list:
              try:
                tf_sub.unregister()
              except:
                pass
            self.tf_subs_list = []
            tf_subs = nepi_sdk.find_topics_by_msg('Frame3DTransformUpdate')
            for tf_sub in tf_subs:
              self.tf_subs_list.append(nepi_sdk.create_publisher(tf_sub, Frame3DTransformUpdate, queue_size=1))
        else:
          self.msg_if.pub_info("Could not find file " + pcd_file)
        if len(self.pcds_dict.keys()) > 0:
          nepi_sdk.sleep(1,10)
          self.node_if.set_param('running', True)
          nepi_sdk.start_timer_process(1, self.publishCb, oneshot = True)
          self.node_if.set_param('running',True)


    self.publish_status()

  def stopPubCb(self,msg):
    self.stopPub()

  def stopPub(self):
    running = self.node_if.get_param('running')
    self.node_if.set_param('running',False)
    time.sleep(1)
    for pcd in self.pcds_dict.keys():
      pcd_dict = self.pcds_dict[pcd]
      pc_if = pcd_dict['pc_if']
      if pc_if != None:
        pc_if.unregister()
    # unsubscribe transform subscribers
    for tf_sub in self.tf_subs_list:
      try:
        tf_sub.unregister()
      except:
        pass
    time.sleep(1)
    self.pcds_dict = dict()
    self.tf_subs_list = []
    self.current_file_list = []
    self.current_topic_list = []
    self.node_if.set_param('running', False)
    self.publish_status()


  def publishCb(self,timer):
    pub_tfs = self.node_if.get_param('pub_transforms')
    running = self.node_if.get_param('running')
    pcd_count = len(self.pcds_dict.keys())
    if running and self.paused == False:
      self.node_if.set_param('running', True)
      for pcd_name in self.pcds_dict.keys():
        get_msg_timestamp = nepi_sdk.get_msg_time()
        pc_if = None
        try:
          pc_if = self.pcds_dict[pcd_name]['pc_if']
          if not nepi_sdk.is_shutdown():
            o3d_pc = self.pcds_dict[pcd_name]['o3d_pc']
            pc_if.publish_o3d_pc(o3d_pc, timestamp = get_msg_timestamp, frame_id = 'base_link')
        except Exception as e:
          self.msg_if.pub_warn("Failed to publish pcd: " + pcd_name + " " + str(e))
        if pub_tfs:
          for tf_sub in self.tf_subs_list:
            try:
              tfu_msg = self.pcds_dict[pcd_name]['tfu_msg']
              tfu_msg.header.stamp = get_msg_timestamp
              tfu_msg.header.frame_id = 'base_link'
              if not nepi_sdk.is_shutdown():
                tf_sub.publish(tfu_msg)
            except Exception as e:
              self.msg_if.pub_warn("Failed to publish pcd: " + pcd_name + " " + str(e))
    running = self.node_if.get_param('running')
    if running == True:
      delay = self.node_if.get_param('delay')
      if delay < 0:
        delay == 0
      nepi_sdk.sleep(delay)
      nepi_sdk.start_timer_process(.001, self.publishCb, oneshot = True)
    else:
      self.stopPub()

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info(" Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePubPcdApp()







