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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img 

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus
from nepi_interfaces.msg import ControlsStatus, UpdateControl

from nepi_app_file_pub_img.msg import FilePubImgStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ImageIF

from nepi_api.connect_node_if import ConnectNodeClassIF

#########################################
# Node Class
#########################################

APP_NODE_NAME = 'app_file_pub_img'

# Leaf of the app's ControlsIF namespace. ControlsIF roots itself at
# create_namespace(node_namespace, controls_name), so the controls topics sit
# one level below the app node namespace, not on it.
CONTROLS_NAME = 'controls'

class ConnectAppFilePubImgIF:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    controls_namespace = ''
    controls_status_msg = None

 
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
        self.controls_namespace = nepi_sdk.create_namespace(self.namespace, CONTROLS_NAME)


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        # The app's adjustable state moved to ControlsIF, so every setter below
        # publishes one UpdateControl instead of its own typed topic. What is
        # still on the app namespace are the COMMANDS: folder navigation, which
        # carries a relative name plus a traversal verb, and start/stop, which a
        # scripted caller uses as the publishing API.
        self.PUBS_DICT = {
            'update_control': {
                'namespace': self.controls_namespace,
                'topic': 'update_control',
                'msg': UpdateControl,
                'qsize': 1,
                'latch': False
            },
            'select_folder': {
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
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': FilePubImgStatus,
                'qsize': 1,
                'callback': self._statusCb
            },
            'controls_status_sub': {
                'namespace': self.controls_namespace,
                'topic': 'status',
                'msg': ControlsStatus,
                'qsize': 1,
                'callback': self._controlsStatusCb
            }
        }


        # Create Node Class ####################
        
        self.con_node_if = ConnectNodeClassIF(
                        namespace = self.namespace,
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True,
                        msg_if = self.msg_if
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

    def get_controls_namespace(self):
        """Return the namespace the app's controls status and update_control topics sit on."""
        return self.controls_namespace

    def get_controls_status_dict(self):
        """Return the app's last received ControlsStatus message as a dict.

        Returns:
            dict: The controls status as a dict, or None if no status has arrived yet.
        """
        if self.controls_status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.controls_status_msg)
        return None

    def set_control_value(self, control_name, value, index = None):
        """Update one of the app's controls.

        Args:
            control_name (str): Name of the control, as it appears in the app's ControlsStatus.
            value: New value. Lists are sent entry by entry; anything else is sent as one value.
            index (int, optional): Component index for a multi-value control. Defaults to None,
                which replaces the whole value.
        """
        msg = UpdateControl()
        msg.name = str(control_name)
        if isinstance(value, (list, tuple)):
            msg.value = [str(item) for item in value]
        else:
            msg.value = [str(value)]
        msg.index = '' if index is None else str(index)
        self.con_node_if.publish_pub('update_control', msg)

    def set_image_size(self,img_size):
        """Set the size published images are resized to."""
        self.set_control_value('size', img_size)

    def set_encoding(self,encoding):
        """Set the encoding published images are converted to."""
        self.set_control_value('encoding', encoding)

    def set_image_rate(self,rate_hz):
        """Set the image publish rate in Hz (clamped to the control bounds)."""
        self.set_control_value('rate_hz', float(rate_hz))

    def set_image_delay(self,image_delay):
        """Set the image publish rate in Hz.

        Kept under its original name and signature. The topic it used to publish
        on, set_delay, was never subscribed by the app node -- the node has only
        ever had set_rate -- so this call did nothing before. It now sets the
        rate control, which is what the name has always meant to the caller.
        """
        self.set_image_rate(image_delay)

    def set_image_random(self,set_random):
        """Enable or disable random image order."""
        self.set_control_value('random', bool(set_random))

    def enable_publishing(self):
        """Start publishing images from the current folder."""
        pub_name = 'start_pub'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def disable_publishing(self):
        """Stop publishing and release the image publishers."""
        pub_name = 'stop_pub'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)
        
    def pause_publishing(self,pause_pub):
        """Hold on the current image instead of advancing."""
        self.set_control_value('paused', bool(pause_pub))

    def next_image(self):
        """While paused, advance one image."""
        self.set_control_value('step_forward', 'TRIGGER')

    def previous_image(self):
        """While paused, go back one image."""
        self.set_control_value('step_backward', 'TRIGGER')

    def set_image_overlay(self,set_overlay):
        """Draw the source filename on each published image."""
        self.set_control_value('overlay', bool(set_overlay))

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


    def _controlsStatusCb(self,status_msg):
        self.controls_status_msg = status_msg

    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg
