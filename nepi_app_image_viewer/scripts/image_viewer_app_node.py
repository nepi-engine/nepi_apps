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
import time
import subprocess
import threading
import copy


from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from sensor_msgs.msg import Image
from nepi_app_image_viewer.msg import ImageSelection
from nepi_interfaces.msg import StringArray

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import SaveDataIF


#########################################

# Factory Control Values

#########################################
# Node Class
#########################################

UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

class NepiImageViewerApp(object):

  FACTORY_SELECTED_TOPICS = ["None","None","None","None"]

  node_if = None

  update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ
  update_save_data_check_interval_sec = float(1)/UPDATE_SAVE_DATA_CHECK_RATE_HZ

  data_products = ["image1","image2","image3","image4"]
  img_subs_dict = dict()

  selected_topics = ["None","None","None","None"]

  #selected_topics = selected_topics
    
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
    self.PARAMS_DICT = {
        'selected_topics': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_topics
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': StringArray,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'set_topic': {
            'namespace': self.node_namespace,
            'topic': 'set_topic',
            'msg': ImageSelection,
            'qsize': 10,
            'callback': self.setImageTopicCb, 
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

    # Setup Save Data IF Class ####################
    self.msg_if.pub_info("Starting Save Data IF Initialization")
    factory_data_rates= {}
    for d in self.data_products_list:
        factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
    if 'image1' in self.data_products_list:
        factory_data_rates['color_2d_image'] = [1.0, 0.0, 100] 
    self.msg_if.pub_warn("Starting data products list: " + str(self.data_products_list))

    factory_filename_dict = {
        'prefix': "", 
        'add_timestamp': True, 
        'add_ms': True,
        'add_us': False,
        'suffix': "",
        'add_node_name': True
        }


    self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                            factory_rate_dict = factory_data_rates,
                            factory_filename_dict = factory_filename_dict)



    # Publish Status
    self.publish_status()

    time.sleep(1)
    nepi_sdk.start_timer_process(0.5, self.statusPublishCb)
    # Give publishers time to setup
    time.sleep(1)

    nepi_sdk.start_timer_process(self.update_image_subs_interval_sec, self.updateImageSubsThread)
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")

    #Set up node shutdown
    nepi_sdk.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_sdk.spin()







  ###################
  ## App Callbacks


  def setImageTopicCb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = msg.image_index
    img_topic = msg.image_topic
    if img_index < len(self.selected_topics):
      self.selected_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_topics', self.selected_topics)
      




  #######################
  ### Config Functions

  def initCb(self,do_updates = False):
    if self.node_if is not None:
      self.selected_topics = self.node_if.get_param('selected_topics')

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


  ###################
  ## Status Publishers

  def statusPublishCb(self,timer):
      self.publish_status()

  def publish_status(self):
    topics = copy.deepcopy(self.selected_topics)
    for i, topic in enumerate(selected_topics):
       if nepi_sdk.check_for_topic(topic) == false:
          topics[i] = 'None'
          
    status_msg = topics
    if self.node_if is not None:
      self.node_if.publish_pub('status_pub',status_msg)


  #######################
  # Update Image Topic Subscribers Thread

  def updateImageSubsThread(self,timer):
    # Subscribe to topic image topics if not subscribed
    sel_topics = self.selected_topics
    #self.msg_if.pub_warn("Selected images: " + str(sel_topics))
    #self.msg_if.pub_warn("Subs dict keys: " + str(self.img_subs_dict.keys()))
    for i, sel_topic in enumerate(sel_topics):
      if sel_topic != "" and sel_topic != "None" and sel_topic not in self.img_subs_dict.keys():
        if nepi_sdk.check_for_topic(sel_topic):
          topic_uid = sel_topic.replace('/','')
          exec('self.' + topic_uid + '_img = None')
          exec('self.' + topic_uid + '_timestamp = None')
          exec('self.' + topic_uid + '_frame = None')
          exec('self.' + topic_uid + '_lock = threading.Lock()')
          self.msg_if.pub_info("Subscribing to topic: " + sel_topic)
          self.msg_if.pub_info("with topic_uid: " + topic_uid)
          data_product = "image" + str(i + 1)
          img_sub = nepi_sdk.create_subscriber(sel_topic, Image, self.imageCb, queue_size = 10, callback_args=data_product)
          self.img_subs_dict[sel_topic] = img_sub
          self.msg_if.pub_info("IMG_VIEW_APP:  Image: " + sel_topic + " registered")
    # Unregister image subscribers if not in selected images list
    unreg_topic_list = []
    for topic in self.img_subs_dict.keys():
      if topic not in sel_topics:
          img_sub = self.img_subs_dict[topic]
          img_sub.unregister()
          self.msg_if.pub_info("IMG_VIEW_APP: Image: " + topic + " unregistered")
          unreg_topic_list.append(topic) # Can't change dictionary while looping through dictionary
    for topic in unreg_topic_list: 
          self.img_subs_dict.pop(topic)
    

  def imageCb(self,img_msg, args):
    data_product = args
    cv2_img = nepi_img.rosimg_to_cv2img(img_msg)
    timestamp = nepi_sdk.sec_from_timestamp(img_msg.header.stamp)
    self.save_data_if.save(data_product,cv2_img,timestamp = timestamp)


 
      


             

  #######################
  # Utility Funcitons   
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("IMG_VIEW_APP:  Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiImageViewerApp()





