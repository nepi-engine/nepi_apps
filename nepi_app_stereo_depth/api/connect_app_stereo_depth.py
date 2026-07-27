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

from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32

from nepi_app_stereo_depth.msg import StereoDepthAppStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_system_if import ConnectSaveDataIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_stereo_depth'


class ConnectAppStereoDepth:
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
                 namespace=None,
                 ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Initialize Class Variables
        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
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
            'set_left_topic': {
                'namespace': self.namespace,
                'topic': 'set_left_topic',
                'msg': String,
                'qsize': 1
            },
            'set_right_topic': {
                'namespace': self.namespace,
                'topic': 'set_right_topic',
                'msg': String,
                'qsize': 1
            },
            'set_sync_tolerance': {
                'namespace': self.namespace,
                'topic': 'set_sync_tolerance',
                'msg': Float32,
                'qsize': 1
            },
            'set_baseline': {
                'namespace': self.namespace,
                'topic': 'set_baseline',
                'msg': Float32,
                'qsize': 1
            },
            'set_checkerboard_cols': {
                'namespace': self.namespace,
                'topic': 'set_checkerboard_cols',
                'msg': Int32,
                'qsize': 1
            },
            'set_checkerboard_rows': {
                'namespace': self.namespace,
                'topic': 'set_checkerboard_rows',
                'msg': Int32,
                'qsize': 1
            },
            'set_square_size': {
                'namespace': self.namespace,
                'topic': 'set_square_size',
                'msg': Float32,
                'qsize': 1
            },
            'set_capture_target': {
                'namespace': self.namespace,
                'topic': 'set_capture_target',
                'msg': Int32,
                'qsize': 1
            },
            'set_num_disparities': {
                'namespace': self.namespace,
                'topic': 'set_num_disparities',
                'msg': Int32,
                'qsize': 1
            },
            'set_block_size': {
                'namespace': self.namespace,
                'topic': 'set_block_size',
                'msg': Int32,
                'qsize': 1
            },
            'set_min_range': {
                'namespace': self.namespace,
                'topic': 'set_min_range',
                'msg': Float32,
                'qsize': 1
            },
            'set_max_range': {
                'namespace': self.namespace,
                'topic': 'set_max_range',
                'msg': Float32,
                'qsize': 1
            },
            'start_calibration': {
                'namespace': self.namespace,
                'topic': 'start_calibration',
                'msg': Empty,
                'qsize': 1
            },
            'capture_calibration': {
                'namespace': self.namespace,
                'topic': 'capture_calibration',
                'msg': Empty,
                'qsize': 1
            },
            'compute_calibration': {
                'namespace': self.namespace,
                'topic': 'compute_calibration',
                'msg': Empty,
                'qsize': 1
            },
            'cancel_calibration': {
                'namespace': self.namespace,
                'topic': 'cancel_calibration',
                'msg': Empty,
                'qsize': 1
            },
            'start_pub': {
                'namespace': self.namespace,
                'topic': 'start_pub',
                'msg': Empty,
                'qsize': 1
            },
            'stop_pub': {
                'namespace': self.namespace,
                'topic': 'stop_pub',
                'msg': Empty,
                'qsize': 1
            },
            'save_config': {
                'namespace': self.namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'reset_config': {
                'namespace': self.namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'factory_reset_config': {
                'namespace': self.namespace,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': StereoDepthAppStatus,
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
            namespace=self.namespace,
            configs_dict=self.CFGS_DICT,
            services_dict=self.SRVS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
            log_class_name=True,
            msg_if=self.msg_if
        )

        self.con_node_if.wait_for_ready()

        self.con_save_data_if = ConnectSaveDataIF(namespace=self.namespace)

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

    def wait_for_ready(self, timeout=float('inf')):
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

    def check_status_connection(self):
        return self.status_connected

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def unregister(self):
        self._unsubscribeTopic()

    #################
    ## Camera Selection

    def set_left_topic(self, topic):
        """Select the left-camera image topic by name."""
        self.con_node_if.publish_pub('set_left_topic', topic)

    def set_right_topic(self, topic):
        """Select the right-camera image topic by name."""
        self.con_node_if.publish_pub('set_right_topic', topic)

    def set_sync_tolerance(self, seconds):
        """Set the maximum left/right timestamp difference (seconds) for frame pairing."""
        self.con_node_if.publish_pub('set_sync_tolerance', seconds)

    #################
    ## Calibration Geometry

    def set_baseline(self, meters):
        """Set the measured stereo baseline in meters (sanity check only)."""
        self.con_node_if.publish_pub('set_baseline', meters)

    def set_checkerboard_cols(self, cols):
        """Set the number of inner checkerboard corners across."""
        self.con_node_if.publish_pub('set_checkerboard_cols', cols)

    def set_checkerboard_rows(self, rows):
        """Set the number of inner checkerboard corners down."""
        self.con_node_if.publish_pub('set_checkerboard_rows', rows)

    def set_square_size(self, meters):
        """Set the checkerboard square size in meters."""
        self.con_node_if.publish_pub('set_square_size', meters)

    def set_capture_target(self, count):
        """Set the target number of calibration captures."""
        self.con_node_if.publish_pub('set_capture_target', count)

    #################
    ## Disparity / Range

    def set_num_disparities(self, num):
        """Set the StereoSGBM disparity search range (snapped to a multiple of 16)."""
        self.con_node_if.publish_pub('set_num_disparities', num)

    def set_block_size(self, size):
        """Set the StereoSGBM matched block size (forced odd)."""
        self.con_node_if.publish_pub('set_block_size', size)

    def set_min_range(self, meters):
        """Set the minimum output depth range in meters."""
        self.con_node_if.publish_pub('set_min_range', meters)

    def set_max_range(self, meters):
        """Set the maximum output depth range in meters."""
        self.con_node_if.publish_pub('set_max_range', meters)

    #################
    ## Calibration Mode

    def start_calibration(self):
        """Begin a checkerboard calibration capture session."""
        self.con_node_if.publish_pub('start_calibration', Empty())

    def capture_calibration(self):
        """Capture the current synchronized frame pair for calibration."""
        self.con_node_if.publish_pub('capture_calibration', Empty())

    def compute_calibration(self):
        """Compute and save the stereo calibration from captured frames."""
        self.con_node_if.publish_pub('compute_calibration', Empty())

    def cancel_calibration(self):
        """Cancel the calibration capture session and discard captures."""
        self.con_node_if.publish_pub('cancel_calibration', Empty())

    #################
    ## Run Mode

    def start_pub(self):
        """Start the run-time depth/pointcloud pipeline."""
        self.con_node_if.publish_pub('start_pub', Empty())

    def stop_pub(self):
        """Stop the run-time depth/pointcloud pipeline."""
        self.con_node_if.publish_pub('stop_pub', Empty())

    #################
    ## Config

    def save_config(self):
        """Persist the current app configuration to disk."""
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        """Reset the app configuration to the last saved values."""
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        """Reset the app configuration to factory defaults."""
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    #################
    ## Save Data Functions

    def get_data_products(self):
        """Return the list of save-data products exposed by this app."""
        return self.con_save_data_if.get_data_products()

    def save_data_pub(self, enable):
        """Enable or disable data saving for this app."""
        self.con_save_data_if.save_data_pub(enable)

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
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister: " + str(e))
        return success

    def _statusCb(self, status_msg):
        self.status_connected = True
        self.status_msg = status_msg
