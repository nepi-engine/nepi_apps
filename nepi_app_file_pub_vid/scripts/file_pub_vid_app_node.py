#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

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


from nepi_app_file_pub_vid.msg import FilePubVidStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ColorImageIF




#########################################
# Node Class
#########################################

class NepiFilePubVidApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  SUPPORTED_FILE_TYPES = ['avi','AVI']

  #Set Initial Values
  MIN_SIZE = 240
  MAX_SIZE = 3700
  STANDARD_IMAGE_SIZES = ['240 x 320', '480 x 640', '630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']
  FACTORY_IMG_SIZE = '630 x 900'
  IMG_PUB_ENCODING_OPTIONS = ["bgr8","rgb8","mono8"]
  FACTORY_IMG_ENCODING_OPTION = "bgr8" 

  UPDATER_DELAY_SEC = 1.0
  
  node_if = None
  
  paused = False
  last_folder = ""
  current_folders = []
  current_file = 'None'
  last_folder = ""
  current_fps = '0'

  running = False
  file_count = 0
  img_pub = None

  oneshot = False

  image_if = None

  default_size = FACTORY_IMG_SIZE.split('x')
  width = int(default_size[1])
  height = int(default_size[0])

  vidcap = None

  width_deg = 100
  height_deg = 70 

  current_folder = HOME_FOLDER
  overlay = False
  size = FACTORY_IMG_SIZE
  encoding = FACTORY_IMG_ENCODING_OPTION
  random = False
  running = False

  restart = False

  update_img = False

  cv2_img = None
  cv2_lock = threading.Lock()
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_vid" # Can be overwitten by luanch command
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
        'overlay': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'size': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_IMG_SIZE
        },
        'encoding': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_IMG_ENCODING_OPTION
        },
        'random': {
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
            'msg': FilePubVidStatus,
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
        'set_size': {
            'namespace': self.node_namespace,
            'topic': 'set_size',
            'msg': String,
            'qsize': None,
            'callback': self.setSizeCb, 
            'callback_args': ()
        },
        'set_encoding': {
            'namespace': self.node_namespace,
            'topic': 'set_encoding',
            'msg': String,
            'qsize': None,
            'callback': self.setEncodingCb, 
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
        'set_overlay': {
            'namespace': self.node_namespace,
            'topic': 'set_overlay',
            'msg': Bool,
            'qsize': None,
            'callback': self.setOverlayCb, 
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
        }
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT
    )


    image_ns = self.node_namespace
    data_product = 'color_image'
    self.image_if = ColorImageIF(namespace = image_ns, 
                data_product_name = data_product, 
                data_source_description = 'file',
                data_ref_description = 'source',
                perspective = 'pov',
                log_name = data_product,
                msg_if = self.msg_if
                )
    ready = self.image_if.wait_for_ready()
    self.image_if.unregister_pubs()
    

    ##############################
    self.initCb(do_updates = True)


    ##############################

    # Start updater process
    nepi_sdk.start_timer_process(self.UPDATER_DELAY_SEC, self.updaterCb)
    nepi_sdk.start_timer_process(1.0, self.statusPublishCb)

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

      current_folder = self.node_if.get_param('current_folder')
      if os.path.exists(current_folder) == False:
        current_folder = self.HOME_FOLDER
      self.current_folder = current_folder
      self.size = self.node_if.get_param('size')
      self.encoding = self.node_if.get_param('encoding')
      self.random = self.node_if.get_param('random')
      self.overlay = self.node_if.get_param('overlay')
      self.restart = self.node_if.get_param('running')

    if do_updates == True:
      if self.node_if is not None and self.running == True:
        self.startPub()
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


  #############################
  ## APP callbacks

  def updateFolderInfo(self, folder):
    if folder != self.last_folder:
      self.stopPub()
      if os.path.exists(folder):
        self.current_folder = folder
        #self.msg_if.pub_warn("Current Folder Exists")
        current_paths = nepi_utils.get_folder_list(folder)
        current_folders = []
        for path in current_paths:
          folder = os.path.basename(path)
          current_folders.append(folder)
        self.current_folders = sorted(current_folders)
        self.publish_status()
        #self.msg_if.pub_warn("Folders: " + str(self.current_folders))
        num_files = 0
        for f_type in self.SUPPORTED_FILE_TYPES:
          num_files = num_files + nepi_utils.get_file_count(folder,ext_str=f_type)
        self.file_count =  num_files
        self.publish_status()
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
    if self.paused == True and self.image_if is not None:
      self.image_if.set_image_callback('needs_update_callback', self.publish_img)
    elif self.image_if is not None:
      self.image_if.set_image_callback('needs_update_callback', None)
    self.publish_status()

  def stepForwardPubCb(self,msg):
    if self.paused:
      self.oneshot = True




  #############################
  ## Image callbacks

  def setSizeCb(self,msg):
    new_size = msg.data
    success = False
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
        if self.node_if is not None:
          self.node_if.set_param('size',new_size)
        self.width = w
        self.height = h
      else:
        self.msg_if.pub_warn( "Received size out of range: " + new_size )
    self.publish_status()

  def setEncodingCb(self,msg):
    new_encoding = msg.data
    if new_encoding in self.IMG_PUB_ENCODING_OPTIONS:
      self.encoding = new_encoding
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('encoding',new_encoding)
    self.publish_status()

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


  def img_needs_update(self):
    #self.msg_if.pub_info('Got update image request')
    self.update_img = True


  def startPubCb(self,msg):
    self.startPub()


  def startPub(self):
    self.msg_if.pub_warn("Start Pub called")
      
    current_folder = self.current_folder
    #self.msg_if.pub_warn("OK to run in folder: " + str(current_folder))
    # Now start publishing images
    self.file_list = []
    self.num_files = 0
    if os.path.exists(current_folder):
      for f_type in self.SUPPORTED_FILE_TYPES:
        [file_list, num_files] = nepi_utils.get_file_list(current_folder,f_type)
        self.file_list.extend(file_list)
        self.num_files += num_files
        self.msg_if.pub_warn("File Pub List: " + str(self.file_list))
        self.msg_if.pub_warn("File Pub Count: " + str(self.num_files))
      if self.num_files > 0:
        self.running = True
        self.publish_status()
        if self.image_if is not None:
          self.msg_if.pub_warn("Registering Image IF pubs")
          self.image_if.register_pubs()
        self.current_ind = 0
        
        self.msg_if.pub_warn("Calling publish callback with running enabled")

        nepi_sdk.start_timer_process(1, self.publishCb, oneshot = True)
        if self.node_if is not None:
          self.msg_if.pub_warn("File Pub Count: " + str(self.num_files))
          self.node_if.set_param('running',True)
      else:
        self.msg_if.pub_info("No image files found in folder " + current_folder)
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
    if self.image_if is not None:
      #self.msg_if.pub_warn("Unregistering Image Pubs")
      self.image_if.unregister_pubs()

    if self.node_if is not None:
      self.node_if.set_param('running',False)
    self.current_file = "None"
    self.current_fps = "0"
    self.publish_status()


  def publishCb(self,timer):
    running = self.running
    size = self.size
    encoding = self.encoding
    set_random = self.random
    overlay = self.overlay

    if running == True:
      if self.image_if != None:
        # Set current index
        if set_random == True and self.paused == False:
          self.current_ind = int(random.random() * self.num_files)
        else:
          self.current_ind = self.current_ind + 1
        # Check ind bounds
        if self.current_ind > (self.num_files-1):
          self.current_ind = 0 # Start over
        elif self.current_ind < 0:
          self.current_ind = self.num_files-1
        file2open = self.file_list[self.current_ind]
        self.current_file = file2open.split('/')[-1]
        #self.msg_if.pub_info("Opening File: " + file2open)
        if os.path.isfile(file2open):
          self.msg_if.pub_info("Opening File: " + file2open)
          self.vidcap = cv2.VideoCapture(file2open)
          if self.vidcap.isOpened() == True:
            success,image = self.vidcap.read()
            shape_str = str(image.shape)
            self.msg_if.pub_info('Image size: ' + shape_str)
            fps = self.vidcap.get(5)
            self.current_fps = str(round(fps, 2))
            self.msg_if.pub_info('Frames per second : ' + self.current_fps)

            frame_count = self.vidcap.get(7)
            self.msg_if.pub_info('Frame count : ' + str(frame_count))

            cv2_img = None
            while success == True and running == True and not nepi_sdk.is_shutdown():
                running = self.running
                size = self.size
                encoding = self.encoding
                set_random = self.random
                overlay = self.overlay
                if cv2_img is None or self.paused == False or self.oneshot == True or self.img_needs_update == True:
                  self.img_needs_update = False
                  self.oneshot = False
                  # Publish video at native fps
                  success,cv2_img = self.vidcap.read()
                  if success == False:
                    self.msg_if.pub_warn("Failed to Get Video Frame") 
                    self.vidcap.release()
                    time.sleep(1)
                    self.vidcap = None
                  else: 
                    cv2_img = cv2.resize(cv2_img,(self.width,self.height))
                    # Overlay Label
                    if overlay == True:
                      # Overlay text data on OpenCV image
                      font                   = cv2.FONT_HERSHEY_DUPLEX
                      fontScale, thickness  = nepi_img.optimal_font_dims(cv2_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
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
                  if success:
                    # Publish new image to ros
                    img_shape = cv2_img.shape     
                    if encoding == 'mono8' and img_shape[2] == 3:
                      cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
                    if encoding != 'mono8' and img_shape[2] == 1:
                      cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_GRAY2BGR)
                    
                    self.cv2_lock.acquire()
                    self.cv2_img = cv2_img
                    self.cv2_lock.release()
                    self.publish_img()
                    self.publish_status()


    if running == True:
      delay = 1
      #self.msg_if.pub_info("Delay: " + str(delay)) 
      nepi_sdk.start_timer_process(delay, self.publishCb, oneshot = True)



  def publish_img(self):
    if self.cv2_img is not None:
      encoding = self.encoding

      if self.image_if is not None:
        self.cv2_lock.acquire()
        self.image_if.publish_cv2_img(self.cv2_img, encoding = encoding,
                                        width_deg = self.width_deg,
                                        height_deg = self.height_deg,
                                        pub_twice = self.paused)
        self.cv2_lock.release()


  ###################
  ## Status Publisher

  def statusPublishCb(self,timer):
      self.publish_status()

  def publish_status(self):
    status_msg = FilePubVidStatus()

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
    status_msg.file_count = self.file_count
    status_msg.current_file =  self.current_file
    status_msg.current_fps = self.current_fps

    status_msg.paused = self.paused

    status_msg.size_options_list = self.STANDARD_IMAGE_SIZES
    
    status_msg.set_size = self.size
    status_msg.encoding_options_list = self.IMG_PUB_ENCODING_OPTIONS
    status_msg.set_encoding = self.encoding
    status_msg.set_random = self.random
    status_msg.set_overlay = self.overlay

    status_msg.running = self.running

    #self.msg_if.pub_warn("Pub Status Msg: " + str(status_msg))
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
  NepiFilePubVidApp()







