#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import time

from std_msgs.msg import Bool, Empty, Float32

from nepi_app_nav_sim.msg import NepiAppNavSimStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_nav_sim'


class ConnectAppNavSim:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    #######################
    ### IF Initialization

    def __init__(self, namespace=None):
        self.class_name     = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name      = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.CFGS_DICT = {'namespace': self.namespace}
        self.SRVS_DICT = None

        ns = self.node_namespace
        self.PUBS_DICT = {
            'set_nmea_enabled': {'namespace': ns, 'topic': 'set_nmea_enabled', 'msg': Bool,    'qsize': 1},
            'set_hnav_enabled': {'namespace': ns, 'topic': 'set_hnav_enabled', 'msg': Bool,    'qsize': 1},
            'set_latitude':     {'namespace': ns, 'topic': 'set_latitude',     'msg': Float32, 'qsize': 1},
            'set_longitude':    {'namespace': ns, 'topic': 'set_longitude',    'msg': Float32, 'qsize': 1},
            'set_altitude':     {'namespace': ns, 'topic': 'set_altitude',     'msg': Float32, 'qsize': 1},
            'set_depth':        {'namespace': ns, 'topic': 'set_depth',        'msg': Float32, 'qsize': 1},
            'set_heading':      {'namespace': ns, 'topic': 'set_heading',      'msg': Float32, 'qsize': 1},
            'set_roll':         {'namespace': ns, 'topic': 'set_roll',         'msg': Float32, 'qsize': 1},
            'set_pitch':        {'namespace': ns, 'topic': 'set_pitch',        'msg': Float32, 'qsize': 1},
            'set_speed':        {'namespace': ns, 'topic': 'set_speed',        'msg': Float32, 'qsize': 1},
            'save_config':          {'namespace': ns, 'topic': 'save_config',          'msg': Empty, 'qsize': None, 'latch': False},
            'reset_config':         {'namespace': ns, 'topic': 'reset_config',         'msg': Empty, 'qsize': None, 'latch': False},
            'factory_reset_config': {'namespace': ns, 'topic': 'factory_reset_config', 'msg': Empty, 'qsize': None, 'latch': False},
        }

        self.SUBS_DICT = {
            'status_sub': {
                'namespace': ns,
                'topic':     'status',
                'msg':       NepiAppNavSimStatus,
                'qsize':     1,
                'callback':  self._statusCb,
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

    def set_nmea_enabled(self, enabled):
        """Enable or disable the NMEA TCP simulator server."""
        msg = Bool(); msg.data = enabled
        self.con_node_if.publish_pub('set_nmea_enabled', msg)

    def set_hnav_enabled(self, enabled):
        """Enable or disable the HNav TCP simulator server."""
        msg = Bool(); msg.data = enabled
        self.con_node_if.publish_pub('set_hnav_enabled', msg)

    def set_latitude(self, lat):
        """Set the simulated latitude (WGS84 decimal degrees)."""
        msg = Float32(); msg.data = float(lat)
        self.con_node_if.publish_pub('set_latitude', msg)

    def set_longitude(self, lon):
        """Set the simulated longitude (WGS84 decimal degrees)."""
        msg = Float32(); msg.data = float(lon)
        self.con_node_if.publish_pub('set_longitude', msg)

    def set_altitude(self, alt_m):
        """Set the simulated altitude in meters."""
        msg = Float32(); msg.data = float(alt_m)
        self.con_node_if.publish_pub('set_altitude', msg)

    def set_depth(self, depth_m):
        """Set the simulated depth in meters (HNav only)."""
        msg = Float32(); msg.data = float(depth_m)
        self.con_node_if.publish_pub('set_depth', msg)

    def set_heading(self, heading_deg):
        """Set the simulated true heading in degrees (0–360)."""
        msg = Float32(); msg.data = float(heading_deg)
        self.con_node_if.publish_pub('set_heading', msg)

    def set_roll(self, roll_deg):
        """Set the simulated roll in degrees (HNav only)."""
        msg = Float32(); msg.data = float(roll_deg)
        self.con_node_if.publish_pub('set_roll', msg)

    def set_pitch(self, pitch_deg):
        """Set the simulated pitch in degrees (HNav only)."""
        msg = Float32(); msg.data = float(pitch_deg)
        self.con_node_if.publish_pub('set_pitch', msg)

    def set_speed(self, speed_ms):
        """Set the dead-reckoning speed in m/s (0 = stationary)."""
        msg = Float32(); msg.data = float(speed_ms)
        self.con_node_if.publish_pub('set_speed', msg)

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
                self.namespace   = None
                self.status_connected = False
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister: " + str(e))

    def _statusCb(self, status_msg):
        self.status_connected = True
        self.connected = True
        self.status_msg = status_msg
