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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus

from nepi_interfaces.msg import ControlsStatus, UpdateControl

from nepi_app_file_pub_depthmap.msg import FilePubDepthmapStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import ImageIF

from nepi_api.connect_node_if import ConnectNodeClassIF

#########################################
# Node Class
#########################################

APP_NODE_NAME = 'app_file_pub_depthmap'

# The app's THREE ControlsIF set names, matching the CONTROLS_NAME_* constants
# in file_pub_depthmap_app_node.py. ControlsIF roots itself at
# create_namespace(node_namespace, controls_name), so each set sits one level
# below the app node namespace, not on it.
#
# The third is navpose_SOURCE, not navpose: the node's NavPoseIF already roots
# itself at <node>/navpose, and a control set of that name would collide with it.
CONTROLS_NAME_PLAYBACK        = 'controls'
CONTROLS_NAME_FOLDER_SETTINGS = 'folder_settings'
CONTROLS_NAME_NAVPOSE         = 'navpose_source'

class ConnectAppFilePubDepthmapIF:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    controls_namespaces = None
    controls_status_msgs = None


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
        # control name -> which set owns it, so a caller (and set_control_value)
        # never has to know which of the three a value lives in.
        self.controls_namespaces = {}
        self.controls_status_msgs = {}
        for controls_name in [CONTROLS_NAME_PLAYBACK, CONTROLS_NAME_FOLDER_SETTINGS,
                              CONTROLS_NAME_NAVPOSE]:
            self.controls_namespaces[controls_name] = nepi_sdk.create_namespace(
                self.namespace, controls_name)
            self.controls_status_msgs[controls_name] = None


        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        # The app's adjustable state moved to three ControlsIF sets, so every
        # setter below publishes one UpdateControl on the right set instead of
        # its own typed topic. What is still on the app namespace are the
        # COMMANDS: folder navigation, which carries a relative name plus a
        # traversal verb, and start/stop, which a scripted caller uses as the
        # publishing API.
        self.PUBS_DICT = {
            'update_control_' + CONTROLS_NAME_PLAYBACK: {
                'namespace': self.controls_namespaces[CONTROLS_NAME_PLAYBACK],
                'topic': 'update_control',
                'msg': UpdateControl,
                'qsize': 1,
                'latch': False
            },
            'update_control_' + CONTROLS_NAME_FOLDER_SETTINGS: {
                'namespace': self.controls_namespaces[CONTROLS_NAME_FOLDER_SETTINGS],
                'topic': 'update_control',
                'msg': UpdateControl,
                'qsize': 1,
                'latch': False
            },
            'update_control_' + CONTROLS_NAME_NAVPOSE: {
                'namespace': self.controls_namespaces[CONTROLS_NAME_NAVPOSE],
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
                'msg': FilePubDepthmapStatus,
                'qsize': 1,
                'callback': self._statusCb
            },
            'controls_status_' + CONTROLS_NAME_PLAYBACK: {
                'namespace': self.controls_namespaces[CONTROLS_NAME_PLAYBACK],
                'topic': 'status',
                'msg': ControlsStatus,
                'qsize': 1,
                'callback': self._playbackControlsStatusCb
            },
            'controls_status_' + CONTROLS_NAME_FOLDER_SETTINGS: {
                'namespace': self.controls_namespaces[CONTROLS_NAME_FOLDER_SETTINGS],
                'topic': 'status',
                'msg': ControlsStatus,
                'qsize': 1,
                'callback': self._folderControlsStatusCb
            },
            'controls_status_' + CONTROLS_NAME_NAVPOSE: {
                'namespace': self.controls_namespaces[CONTROLS_NAME_NAVPOSE],
                'topic': 'status',
                'msg': ControlsStatus,
                'qsize': 1,
                'callback': self._navposeControlsStatusCb
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
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def unregister(self):
        self._unsubscribeTopic()


    def select_folder(self,folder_name):
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

    def get_controls_namespace(self, controls_name):
        """Return the namespace one of the app's three control sets sits on.

        Args:
            controls_name (str): 'controls', 'folder_settings' or 'navpose_source'.

        Returns:
            str: The namespace, or None if the name is not one of the three.
        """
        return self.controls_namespaces.get(controls_name, None)

    def get_controls_status_dict(self, controls_name):
        """Return one control set's last received ControlsStatus as a dict.

        Args:
            controls_name (str): 'controls', 'folder_settings' or 'navpose_source'.

        Returns:
            dict: The controls status as a dict, or None if none has arrived yet.
        """
        status_msg = self.controls_status_msgs.get(controls_name, None)
        if status_msg is not None:
            return nepi_sdk.convert_msg2dict(status_msg)
        return None

    def set_control_value(self, controls_name, control_name, value, index = None):
        """Update one control in one of the app's three control sets.

        Args:
            controls_name (str): 'controls', 'folder_settings' or 'navpose_source'.
            control_name (str): Name of the control, as it appears in that set's
                ControlsStatus.
            value: New value. Lists are sent entry by entry; anything else is sent
                as one value.
            index (int, optional): Component index for a multi-value control.
                Defaults to None, which replaces the whole value.
        """
        if controls_name not in self.controls_namespaces:
            self.msg_if.pub_warn("Unknown controls set: " + str(controls_name))
            return
        msg = UpdateControl()
        msg.name = str(control_name)
        if isinstance(value, (list, tuple)):
            msg.value = [str(item) for item in value]
        else:
            msg.value = [str(value)]
        msg.index = '' if index is None else str(index)
        self.con_node_if.publish_pub('update_control_' + controls_name, msg)

    def set_rate(self,rate_hz):
        """Set the collection publish rate in Hz (clamped by the control bounds)."""
        self.set_control_value(CONTROLS_NAME_PLAYBACK, 'rate_hz', float(rate_hz))

    def set_random(self,set_random):
        """Enable or disable random collection order."""
        self.set_control_value(CONTROLS_NAME_PLAYBACK, 'random', bool(set_random))

    def enable_publishing(self):
        pub_name = 'start_pub'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def disable_publishing(self):
        pub_name = 'stop_pub'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def pause_publishing(self,pause_pub):
        """Hold on the current collection instead of advancing."""
        self.set_control_value(CONTROLS_NAME_PLAYBACK, 'paused', bool(pause_pub))

    def next_collection(self):
        """While paused, advance one collection."""
        self.set_control_value(CONTROLS_NAME_PLAYBACK, 'step_forward', 'TRIGGER')

    def previous_collection(self):
        """While paused, go back one collection."""
        self.set_control_value(CONTROLS_NAME_PLAYBACK, 'step_backward', 'TRIGGER')

    def set_overlay(self,set_overlay):
        """Draw the source filename on each published color image."""
        self.set_control_value(CONTROLS_NAME_PLAYBACK, 'overlay', bool(set_overlay))

    def set_width_deg(self,width_deg):
        """Set the angular width the published products declare."""
        self.set_control_value(CONTROLS_NAME_FOLDER_SETTINGS, 'width_deg', float(width_deg))

    def set_height_deg(self,height_deg):
        """Set the angular height the published products declare."""
        self.set_control_value(CONTROLS_NAME_FOLDER_SETTINGS, 'height_deg', float(height_deg))

    def set_apply_folder_settings(self,apply_settings):
        """Read a collection folder settings sidecar and apply the field of view it declares."""
        self.set_control_value(CONTROLS_NAME_FOLDER_SETTINGS,
                               'apply_folder_settings', bool(apply_settings))

    #################
    ## NavPose Source
    #
    # The app publishes <node>/navpose from one of two sources. In 'system' mode
    # navpose_mgr's pose is forwarded unchanged and the static values and frames
    # below are ignored; in 'static' mode the app authors the pose from them.
    # get_navpose_active_mode() reports which is actually publishing.

    def get_navpose_source_mode(self):
        """Return the operator's nav pose source mode setting.

        Returns:
            str: 'auto', 'system' or 'static', or None if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.navpose_source_mode

    def get_navpose_source_mode_options(self):
        """Return the nav pose source modes the app accepts.

        Returns:
            list: Mode strings, or an empty list if no status has arrived.
        """
        if self.status_msg is None:
            return []
        return list(self.status_msg.navpose_source_mode_options)

    def get_navpose_active_mode(self):
        """Return the nav pose source that is actually publishing.

        Returns:
            str: 'system' or 'static', or None if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.navpose_active_mode

    def get_navpose_system_available(self):
        """Return whether a fresh system nav pose is available to forward.

        Returns:
            bool: True if a system nav pose arrived within the staleness window.
        """
        if self.status_msg is None:
            return False
        return self.status_msg.navpose_system_available

    def get_navpose_system_timeout(self):
        """Return the system nav pose staleness window.

        Returns:
            float: Window in seconds, or None if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.navpose_system_timeout_sec

    def get_navpose_pub_rate(self):
        """Return the steady rate the app republishes its nav pose at.

        Returns:
            float: Publish rate in Hz, or None if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.navpose_pub_rate

    def get_navpose_static_pose(self):
        """Return the operator-authored static pose values.

        Keys are the NEPI navpose dict key names, so the result can be merged
        straight into a navpose dict.

        Returns:
            dict: Static pose values, or None if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return {
            'latitude': self.status_msg.navpose_static_latitude,
            'longitude': self.status_msg.navpose_static_longitude,
            'heading_deg': self.status_msg.navpose_static_heading_deg,
            'roll_deg': self.status_msg.navpose_static_roll_deg,
            'pitch_deg': self.status_msg.navpose_static_pitch_deg,
            'yaw_deg': self.status_msg.navpose_static_yaw_deg,
            'x_m': self.status_msg.navpose_static_x_m,
            'y_m': self.status_msg.navpose_static_y_m,
            'z_m': self.status_msg.navpose_static_z_m,
            'altitude_m': self.status_msg.navpose_static_altitude_m,
            'depth_m': self.status_msg.navpose_static_depth_m
        }

    def get_navpose_static_frames(self):
        """Return the frames the static pose is declared in.

        These apply only while the active mode is 'static'; a forwarded system
        pose keeps the frames its author set.

        Returns:
            dict: Keys 'frame_nav', 'frame_altitude' and 'frame_depth', or None
                if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return {
            'frame_nav': self.status_msg.navpose_static_frame_nav,
            'frame_altitude': self.status_msg.navpose_static_frame_altitude,
            'frame_depth': self.status_msg.navpose_static_frame_depth
        }

    def get_navpose_frame_options(self):
        """Return the frame options the app will accept for the static pose.

        Returns:
            dict: Keys 'frame_nav', 'frame_altitude' and 'frame_depth', each a
                list of option strings, or None if no status has arrived.
        """
        if self.status_msg is None:
            return None
        return {
            'frame_nav': list(self.status_msg.navpose_frame_nav_options),
            'frame_altitude': list(self.status_msg.navpose_frame_altitude_options),
            'frame_depth': list(self.status_msg.navpose_frame_depth_options)
        }

    def set_navpose_source_mode(self,source_mode):
        """Set the nav pose source mode.

        Args:
            source_mode (str): 'auto', 'system' or 'static'. A value the node
                does not recognize is rejected and the previous mode kept.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_source_mode', source_mode)

    def set_navpose_system_timeout(self,timeout_sec):
        """Set the system nav pose staleness window.

        Args:
            timeout_sec (float): Window in seconds. Clamped by the node.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_system_timeout_sec', float(timeout_sec))

    def set_navpose_static_latitude(self,latitude):
        """Set the static pose latitude.

        Args:
            latitude (float): Latitude in degrees.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_latitude', float(latitude))

    def set_navpose_static_longitude(self,longitude):
        """Set the static pose longitude.

        Args:
            longitude (float): Longitude in degrees.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_longitude', float(longitude))

    def set_navpose_static_heading(self,heading_deg):
        """Set the static pose heading.

        Args:
            heading_deg (float): Heading in degrees true north.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_heading_deg', float(heading_deg))

    def set_navpose_static_roll(self,roll_deg):
        """Set the static pose roll.

        Args:
            roll_deg (float): Roll in degrees in the selected nav frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_roll_deg', float(roll_deg))

    def set_navpose_static_pitch(self,pitch_deg):
        """Set the static pose pitch.

        Args:
            pitch_deg (float): Pitch in degrees in the selected nav frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_pitch_deg', float(pitch_deg))

    def set_navpose_static_yaw(self,yaw_deg):
        """Set the static pose yaw.

        Args:
            yaw_deg (float): Yaw in degrees in the selected nav frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_yaw_deg', float(yaw_deg))

    def set_navpose_static_x(self,x_m):
        """Set the static pose X position.

        Args:
            x_m (float): X position in meters in the selected nav frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_x_m', float(x_m))

    def set_navpose_static_y(self,y_m):
        """Set the static pose Y position.

        Args:
            y_m (float): Y position in meters in the selected nav frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_y_m', float(y_m))

    def set_navpose_static_z(self,z_m):
        """Set the static pose Z position.

        Args:
            z_m (float): Z position in meters in the selected nav frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_z_m', float(z_m))

    def set_navpose_static_altitude(self,altitude_m):
        """Set the static pose altitude.

        Args:
            altitude_m (float): Altitude in meters in the selected altitude frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_altitude_m', float(altitude_m))

    def set_navpose_static_depth(self,depth_m):
        """Set the static pose depth.

        Args:
            depth_m (float): Depth in positive meters in the selected depth frame.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_depth_m', float(depth_m))

    def set_navpose_static_frame_nav(self,frame_nav):
        """Set the nav frame the static pose is declared in.

        Applies only in static mode. A value not in get_navpose_frame_options()
        is rejected by the node and the previous frame kept.

        Args:
            frame_nav (str): Nav frame identifier, e.g. 'ENU'.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_frame_nav', frame_nav)

    def set_navpose_static_frame_altitude(self,frame_altitude):
        """Set the altitude frame the static pose is declared in.

        Applies only in static mode. A value not in get_navpose_frame_options()
        is rejected by the node and the previous frame kept.

        Args:
            frame_altitude (str): Altitude frame identifier, e.g. 'WGS84'.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_frame_altitude', frame_altitude)

    def set_navpose_static_frame_depth(self,frame_depth):
        """Set the depth frame the static pose is declared in.

        Applies only in static mode. A value not in get_navpose_frame_options()
        is rejected by the node and the previous frame kept.

        Args:
            frame_depth (str): Depth frame identifier, e.g. 'DEPTH'.
        """
        self.set_control_value(CONTROLS_NAME_NAVPOSE, 'navpose_static_frame_depth', frame_depth)


    def save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())


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
                self.msg_if.pub_warn("Failed to unregister:  " + str(e))
        return success


    def _playbackControlsStatusCb(self,status_msg):
        self.controls_status_msgs[CONTROLS_NAME_PLAYBACK] = status_msg

    def _folderControlsStatusCb(self,status_msg):
        self.controls_status_msgs[CONTROLS_NAME_FOLDER_SETTINGS] = status_msg

    def _navposeControlsStatusCb(self,status_msg):
        self.controls_status_msgs[CONTROLS_NAME_NAVPOSE] = status_msg

    def _statusCb(self,status_msg):
        self.status_connected = True
        self.status_msg = status_msg
