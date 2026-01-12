#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import time
import sys
import numpy as np
import cv2
import open3d as o3d
import random



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus

from nepi_app_file_pub_vid.msg import FilePubVidStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ImageIF
from nepi_api.connect_node_if import ConnectNodeClassIF


APP_NODE_NAME = 'app_file_pub_vid'

class ConnectAppFilePubVidIF:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

 
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                ):
        ####  IF INIT SETUP ####
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

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace,APP_NODE_NAME)
        else:
            namespace = namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'sub_name': {
            'namespace': self.node_namespace,
            'topic': 'select_folder',
            'msg': String,
            'qsize': None,
                'latch': False
            },
            'home_folder': {
                'namespace': self.node_namespace,
                'topic': 'home_folder',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'back_folder': {
                'namespace': self.node_namespace,
                'topic': 'back_folder',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'set_size': {
                'namespace': self.node_namespace,
                'topic': 'set_size',
                'msg': String,
                'qsize': None,
                'latch': False
            },
            'set_encoding': {
                'namespace': self.node_namespace,
                'topic': 'set_encoding',
                'msg': String,
                'qsize': None,
                'latch': False
            },
            'set_random': {
                'namespace': self.node_namespace,
                'topic': 'set_random',
                'msg': Bool,
                'qsize': None,
                'latch': False
            },
            'sub_name': {
                'namespace': self.node_namespace,
                'topic': 'select_folder',
                'msg': String,
                'qsize': None,
                'latch': False
            },
            'set_overlay': {
                'namespace': self.node_namespace,
                'topic': 'set_overlay',
                'msg': Bool,
                'qsize': None,
                'qsize': None,
                'latch': False
            },
            'start_pub': {
                'namespace': self.node_namespace,
                'topic': 'start_pub',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'stop_pub': {
                'namespace': self.node_namespace,
                'topic': 'stop_pub',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'pause_pub': {
                'namespace': self.node_namespace,
                'topic': 'pause_pub',
                'msg': Bool,
                'qsize': None,
                'latch': False
            },
            'step_forward': {
                'namespace': self.node_namespace,
                'topic': 'step_forward',
                'msg': Empty,
                'qsize': None,
                'qsize': None,
                'latch': False
            },
        'save_config': {
            'namespace': self.node_namespace,
            'topic': 'save_config',
            'msg': Empty,
            'qsize': None,
            'latch': False
        },
        'reset_config': {
            'namespace': self.node_namespace,
            'topic': 'reset_config',
            'msg': Empty,
            'qsize': None,
            'latch': False
        },
        'factory_reset_config': {
            'namespace': self.node_namespace,
            'topic': 'factory_reset_config',
            'msg': Empty,
            'qsize': None,
            'latch': False
        }
    }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': FilePubVidStatus,
            'qsize': 1,
                'callback': self._statusCb
            }
        }


        # Create Node Class ####################
        
        self.con_node_if = ConnectNodeClassIF(
                        namespace = self.namespace,
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        

        self.con_node_if.wait_for_ready()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def unregister(self):
        self._unsubscribeTopic()

    def save_data_prefix_pub(self,folder_name):
        pub_name = 'select_folder'
        msg = folder_name
        self.con_node_if.publish_pub(pub_name,msg)

    def go_home_folder(self):
        pub_name = 'home_folder'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def go_back_folder(self):
        pub_name = 'back_folder'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_video_size(self,img_size):
        pub_name = 'set_size'
        msg = img_size
        self.con_node_if.publish_pub(pub_name,msg)

    def set_encoding(self,encoding):
        pub_name = 'set_encoding'
        msg = encoding
        self.con_node_if.publish_pub(pub_name,msg)

    def set_video_random(self,set_random):
        pub_name = 'set_random'
        msg = set_random
        self.con_node_if.publish_pub(pub_name,msg)        

    def enable_publishing(self):
        pub_name = 'start_pub'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def disable_publishing(self):
        pub_name = 'stop_pub'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)
        
    def pause_publishing(self,pause_pub):
        pub_name = 'pause_pub'
        msg = pause_pub
        self.con_node_if.publish_pub(pub_name,msg)   

    def next_video(self):
        pub_name = 'step_forward'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_video_overlay(self,set_overlay):
        pub_name = 'set_overlay'
        msg = set_overlay
        self.con_node_if.publish_pub(pub_name,msg)  

    def save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())
        
    #################
    ## Save Data Functions

    def get_data_products(self):
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_status_dict(self):
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_pub(self,enable):
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def snapshot_pub(self):
        self.con_save_data_if.publish_pub()

    def reset_pub(self):
        self.con_save_data_if.publish_pub(pub_name,msg)

    def factory_reset_pub(self):
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg
