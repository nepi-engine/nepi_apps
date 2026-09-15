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

import time

from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import ControlsStatus, UpdateControl

from nepi_app_fake_gps.msg import NepiAppFakeGpsStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_fake_gps'

# Leaf of the app's ControlsIF namespace. ControlsIF roots itself at
# create_namespace(node_namespace, controls_name), so the controls topics sit
# one level below the app node namespace, not on it.
CONTROLS_NAME = 'controls'


class ConnectAppFakeGps:
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

    def __init__(self, namespace=None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.CFGS_DICT = {'namespace': self.namespace}
        self.SRVS_DICT = None

        ns = self.namespace
        self.controls_namespace = nepi_sdk.create_namespace(ns, CONTROLS_NAME)
        controls_ns = self.controls_namespace

        # The app's adjustable state moved to ControlsIF, so every setter below
        # publishes one UpdateControl instead of its own typed topic. What is
        # still on the app namespace are the COMMANDS, which carry a whole
        # geopoint or offset in a single message.
        self.PUBS_DICT = {
            'update_control':    {'namespace': controls_ns, 'topic': 'update_control', 'msg': UpdateControl, 'qsize': 1},
            'reset':             {'namespace': ns, 'topic': 'reset',             'msg': GeoPoint, 'qsize': 1},
            'go_stop':           {'namespace': ns, 'topic': 'go_stop',           'msg': Empty,    'qsize': 1},
            'goto_position':     {'namespace': ns, 'topic': 'goto_position',     'msg': Point,    'qsize': 1},
            'goto_location':     {'namespace': ns, 'topic': 'goto_location',     'msg': GeoPoint, 'qsize': 1},
            'save_config':          {'namespace': ns, 'topic': 'save_config',          'msg': Empty, 'qsize': None, 'latch': False},
            'reset_config':         {'namespace': ns, 'topic': 'reset_config',         'msg': Empty, 'qsize': None, 'latch': False},
            'factory_reset_config': {'namespace': ns, 'topic': 'factory_reset_config', 'msg': Empty, 'qsize': None, 'latch': False},
        }

        self.SUBS_DICT = {
            'status_sub': {
                'namespace': ns,
                'topic':     'status',
                'msg':       NepiAppFakeGpsStatus,
                'qsize':     1,
                'callback':  self._statusCb,
            },
            'controls_status_sub': {
                'namespace': controls_ns,
                'topic':     'status',
                'msg':       ControlsStatus,
                'qsize':     1,
                'callback':  self._controlsStatusCb,
            }
        }

        self.con_node_if = ConnectNodeClassIF(
            namespace=self.namespace,
            configs_dict=self.CFGS_DICT,
            services_dict=self.SRVS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
            log_class_name=True,
            msg_if=self.msg_if,
        )
        self.con_node_if.wait_for_ready()

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        return self.ready

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def check_status_connection(self):
        return self.status_connected

    def get_status_dict(self):
        if self.status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.status_msg)
        return None

    def get_controls_namespace(self):
        """Return the app's ControlsIF namespace.

        Returns:
            str: The namespace the app's controls status and update_control topics sit on.
        """
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

    def select_mavros_node(self, node_namespace):
        """Select the target mavros (mavlink) node namespace to inject GPS_INPUT into."""
        self.set_control_value('mavros_node', node_namespace)

    def set_enabled(self, enabled):
        """Enable or disable the fake GPS GPS_INPUT injection."""
        self.set_control_value('enabled', bool(enabled))

    def set_satellites_visible(self, sat_count):
        """Set the satellite count reported in the injected GPS_INPUT message."""
        self.set_control_value('satellites_visible', int(sat_count))

    def set_start_location(self, latitude, longitude, altitude):
        """Set the start location the simulated position seeds and resets to."""
        self.set_control_value('start_latitude', float(latitude))
        self.set_control_value('start_longitude', float(longitude))
        self.set_control_value('start_altitude_m', float(altitude))

    def reset_location(self, latitude, longitude, altitude):
        """Reset the simulated GPS home position to a new WGS84 geopoint."""
        msg = GeoPoint()
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude)
        self.con_node_if.publish_pub('reset', msg)

    def go_stop(self):
        """Stop any active simulated move and hold the current position."""
        self.con_node_if.publish_pub('go_stop', Empty())

    def goto_position(self, x, y, z):
        """Move the simulated position by an ENU offset in meters (east, north, up)."""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.con_node_if.publish_pub('goto_position', msg)

    def goto_location(self, latitude, longitude, altitude):
        """Move the simulated position to an absolute WGS84 geopoint."""
        msg = GeoPoint()
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude)
        self.con_node_if.publish_pub('goto_location', msg)

    def set_gps_pub_rate(self, rate_hz):
        """Set the fake GPS publish rate in Hz (clamped to 1-100 by the control bounds)."""
        self.set_control_value('gps_pub_rate_hz', float(rate_hz))

    def save_config(self):
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    def unregister(self):
        self._unregisterNode()


    ###############################
    # Class Private Methods
    ###############################

    def _unregisterNode(self):
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister: " + str(e))

    def _statusCb(self, status_msg):
        self.status_connected = True
        self.connected = True
        self.status_msg = status_msg

    def _controlsStatusCb(self, status_msg):
        self.controls_status_msg = status_msg
