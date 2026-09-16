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

import time

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import UpdateControl, UpdateString

from nepi_app_nav_sim.msg import NepiAppNavSimMasterStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_nav_sim'

# Instance kinds this app simulates, and the path segment each is rooted at.
KIND_NMEA = 'nmea'
KIND_HNAV = 'hnav'
KIND_GPS  = 'gps'
KINDS = (KIND_NMEA, KIND_HNAV, KIND_GPS)

# Section suffixes of the per-instance control sets, matching the
# CONTROLS_SUFFIX_* constants in nav_sim_app_node.py and NepiAppNavSim.js.
CONTROLS_SUFFIX_POSITION      = 'position'
CONTROLS_SUFFIX_ORIENTATION   = 'orientation'
CONTROLS_SUFFIX_DEADRECKONING = 'dead_reckoning'
CONTROLS_SUFFIX_MOVE          = 'move'
CONTROLS_SUFFIX_OUTPUT        = 'output'

# Which section owns which control, so a caller can name a control without also
# having to know which set it lives in. A control name is unique across an
# instance's sets, which is what makes this table possible.
_CONTROL_SECTIONS = {
    # NMEA / HNav generated rows are resolved by prefix below; these are the
    # GPS kind's hand-written controls.
    'start_latitude':       CONTROLS_SUFFIX_POSITION,
    'start_longitude':      CONTROLS_SUFFIX_POSITION,
    'start_altitude_m':     CONTROLS_SUFFIX_POSITION,
    'use_current_location': CONTROLS_SUFFIX_POSITION,
    'set_location':         CONTROLS_SUFFIX_POSITION,
    'goto_latitude':        CONTROLS_SUFFIX_MOVE,
    'goto_longitude':       CONTROLS_SUFFIX_MOVE,
    'goto_altitude_m':      CONTROLS_SUFFIX_MOVE,
    'goto_location':        CONTROLS_SUFFIX_MOVE,
    'goto_position_m':      CONTROLS_SUFFIX_MOVE,
    'goto_position':        CONTROLS_SUFFIX_MOVE,
    'stop':                 CONTROLS_SUFFIX_MOVE,
    'mavros_node':          CONTROLS_SUFFIX_OUTPUT,
    'gps_pub_rate_hz':      CONTROLS_SUFFIX_OUTPUT,
    'satellites_visible':   CONTROLS_SUFFIX_OUTPUT,
}

# Field-name tails that place an NMEA or HNav row control in a section.
_SECTION_BY_FIELD_TAIL = (
    ('latitude',    CONTROLS_SUFFIX_POSITION),
    ('longitude',   CONTROLS_SUFFIX_POSITION),
    ('altitude_m',  CONTROLS_SUFFIX_POSITION),
    ('depth_m',     CONTROLS_SUFFIX_POSITION),
    ('heading_deg', CONTROLS_SUFFIX_ORIENTATION),
    ('roll_deg',    CONTROLS_SUFFIX_ORIENTATION),
    ('pitch_deg',   CONTROLS_SUFFIX_ORIENTATION),
    ('speed_ms',    CONTROLS_SUFFIX_DEADRECKONING),
)

# Row-control prefixes, stripped before the tail lookup above.
_ROW_PREFIXES = ('enable_move_', 'move_step_', 'move_rate_hz_',
                 'enable_sin_', 'sin_amplitude_', 'sin_period_s_',
                 'enable_wave_', 'sin_spread_')


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

        # The app namespace, not the calling node's. The previous revision of
        # this file built every publisher on nepi_sdk.get_node_namespace(),
        # which is the CONSUMER's node, so nothing it published ever reached the
        # app.
        ns = self.namespace

        # Instance management, plus the app-level config topics. Everything a
        # caller can ADJUST is per instance and lives in that instance's control
        # sets, so it is published dynamically through update_control rather
        # than registered here -- the set of instances is not known until the
        # master status arrives.
        self.PUBS_DICT = {
            'add_nmea_instance':    {'namespace': ns, 'topic': 'add_nmea_instance',    'msg': String,       'qsize': 1},
            'remove_nmea_instance': {'namespace': ns, 'topic': 'remove_nmea_instance', 'msg': String,       'qsize': 1},
            'rename_nmea_instance': {'namespace': ns, 'topic': 'rename_nmea_instance', 'msg': UpdateString, 'qsize': 1},
            'add_hnav_instance':    {'namespace': ns, 'topic': 'add_hnav_instance',    'msg': String,       'qsize': 1},
            'remove_hnav_instance': {'namespace': ns, 'topic': 'remove_hnav_instance', 'msg': String,       'qsize': 1},
            'rename_hnav_instance': {'namespace': ns, 'topic': 'rename_hnav_instance', 'msg': UpdateString, 'qsize': 1},
            'add_gps_instance':     {'namespace': ns, 'topic': 'add_gps_instance',     'msg': String,       'qsize': 1},
            'remove_gps_instance':  {'namespace': ns, 'topic': 'remove_gps_instance',  'msg': String,       'qsize': 1},
            'rename_gps_instance':  {'namespace': ns, 'topic': 'rename_gps_instance',  'msg': UpdateString, 'qsize': 1},
            'save_config':          {'namespace': ns, 'topic': 'save_config',          'msg': Empty, 'qsize': None, 'latch': False},
            'reset_config':         {'namespace': ns, 'topic': 'reset_config',         'msg': Empty, 'qsize': None, 'latch': False},
            'factory_reset_config': {'namespace': ns, 'topic': 'factory_reset_config', 'msg': Empty, 'qsize': None, 'latch': False},
        }

        self.SUBS_DICT = {
            'status_sub': {
                'namespace': ns,
                'topic':     'status',
                'msg':       NepiAppNavSimMasterStatus,
                'qsize':     1,
                'callback':  self._statusCb,
            }
        }

        # Publishers built on demand for per-instance topics and control sets,
        # keyed by full topic namespace. ConnectNodeClassIF's registry is fixed
        # at construction, and the instance list is not known until the master
        # status arrives, so these are created lazily and cached.
        self._dyn_pubs = {}

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

    def get_instance_names(self, kind):
        """Return the names of the app's instances of one simulator kind.

        Args:
            kind (str): One of 'nmea', 'hnav' or 'gps'.

        Returns:
            list: Instance names, or an empty list if no master status has
                arrived yet or the kind is not one this app simulates.
        """
        if self.status_msg is None or kind not in KINDS:
            return []
        return list(getattr(self.status_msg, kind + '_instance_names', []))

    def get_instance_namespace(self, kind, instance_name):
        """Return the ROS namespace of one simulator instance.

        Args:
            kind (str): One of 'nmea', 'hnav' or 'gps'.
            instance_name (str): Name of the instance.

        Returns:
            str: The instance namespace, e.g. <app>/gps_instances/gps_0.
        """
        return self._instanceNamespace(kind, instance_name)

    def get_controls_namespace(self, kind, instance_name, suffix):
        """Return the namespace of one of an instance's control sets.

        A ControlsIF is always a direct child of the NODE namespace, so the
        instance identity is carried in the set NAME rather than the path. This
        is the same derivation instanceControlsName() does in the node and
        getControlsNamespace() does in the RUI.

        Args:
            kind (str): One of 'nmea', 'hnav' or 'gps'.
            instance_name (str): Name of the instance.
            suffix (str): Section suffix, e.g. 'position' or 'move'.

        Returns:
            str: The control set's namespace, e.g.
                <app>/gps_instances_gps_0_position.
        """
        return self._controlsNamespace(kind, instance_name, suffix)

    def add_instance(self, kind, instance_name=''):
        """Add a simulator instance.

        Args:
            kind (str): One of 'nmea', 'hnav' or 'gps'.
            instance_name (str, optional): Name for the new instance. Defaults
                to '', which lets the app generate the next free name.
        """
        if kind not in KINDS:
            return
        msg = String(); msg.data = str(instance_name)
        self.con_node_if.publish_pub('add_' + kind + '_instance', msg)

    def remove_instance(self, kind, instance_name):
        """Remove a simulator instance. The app refuses to remove the last one."""
        if kind not in KINDS:
            return
        msg = String(); msg.data = str(instance_name)
        self.con_node_if.publish_pub('remove_' + kind + '_instance', msg)

    def rename_instance(self, kind, instance_name, new_name):
        """Rename a simulator instance, carrying its current state across."""
        if kind not in KINDS:
            return
        msg = UpdateString()
        msg.name  = str(instance_name)
        msg.value = str(new_name)
        self.con_node_if.publish_pub('rename_' + kind + '_instance', msg)

    def set_enabled(self, kind, instance_name, enabled):
        """Enable or disable one simulator instance.

        For the NMEA and HNav kinds this starts or stops the TCP server. For the
        GPS kind it starts or stops publishing the simulated fix and injecting
        GPS_INPUT into the selected mavros node.
        """
        topic = self._instanceNamespace(kind, instance_name)
        if topic is None:
            return
        msg = Bool(); msg.data = bool(enabled)
        self._publishDynamic(topic + '/set_' + kind + '_enabled', Bool, msg)

    def set_control_value(self, kind, instance_name, control_name, value, index=None):
        """Update one control of one simulator instance.

        Args:
            kind (str): One of 'nmea', 'hnav' or 'gps'.
            instance_name (str): Name of the instance.
            control_name (str): Name of the control, as it appears in that
                instance's ControlsStatus.
            value: New value. Lists are sent entry by entry; anything else is
                sent as one value.
            index (int, optional): Component index for a multi-value control.
                Defaults to None, which replaces the whole value.
        """
        suffix = self._controlSection(control_name)
        if suffix is None:
            self.msg_if.pub_warn("Unknown control name: " + str(control_name))
            return
        controls_ns = self._controlsNamespace(kind, instance_name, suffix)
        if controls_ns is None:
            return
        msg = UpdateControl()
        msg.name = str(control_name)
        if isinstance(value, (list, tuple)):
            msg.value = [str(item) for item in value]
        else:
            msg.value = [str(value)]
        msg.index = '' if index is None else str(index)
        self._publishDynamic(controls_ns + '/update_control', UpdateControl, msg)

    def press_button(self, kind, instance_name, control_name):
        """Press one of an instance's Button controls.

        A Button press arrives as the non-numeric sentinel 'TRIGGER', which the
        control value cleaner stamps with the current time.
        """
        self.set_control_value(kind, instance_name, control_name, 'TRIGGER')

    def set_latitude(self, kind, instance_name, lat):
        """Set the simulated latitude (WGS84 decimal degrees)."""
        self.set_control_value(kind, instance_name,
                               self._geoControl(kind, 'latitude'), float(lat))

    def set_longitude(self, kind, instance_name, lon):
        """Set the simulated longitude (WGS84 decimal degrees)."""
        self.set_control_value(kind, instance_name,
                               self._geoControl(kind, 'longitude'), float(lon))

    def set_altitude(self, kind, instance_name, alt_m):
        """Set the simulated altitude in meters."""
        self.set_control_value(kind, instance_name,
                               self._geoControl(kind, 'altitude_m'), float(alt_m))

    def set_depth(self, instance_name, depth_m):
        """Set the simulated depth in meters (HNav only)."""
        self.set_control_value(KIND_HNAV, instance_name, 'hnav_depth_m', float(depth_m))

    def set_heading(self, kind, instance_name, heading_deg):
        """Set the simulated true heading in degrees (NMEA and HNav only)."""
        if kind not in (KIND_NMEA, KIND_HNAV):
            return
        self.set_control_value(kind, instance_name,
                               kind + '_heading_deg', float(heading_deg))

    def set_roll(self, instance_name, roll_deg):
        """Set the simulated roll in degrees (HNav only)."""
        self.set_control_value(KIND_HNAV, instance_name, 'hnav_roll_deg', float(roll_deg))

    def set_pitch(self, instance_name, pitch_deg):
        """Set the simulated pitch in degrees (HNav only)."""
        self.set_control_value(KIND_HNAV, instance_name, 'hnav_pitch_deg', float(pitch_deg))

    def set_speed(self, kind, instance_name, speed_ms):
        """Set the dead-reckoning speed in m/s (NMEA and HNav only; 0 = stationary)."""
        if kind not in (KIND_NMEA, KIND_HNAV):
            return
        self.set_control_value(kind, instance_name, kind + '_speed_ms', float(speed_ms))

    #######################
    # GPS instance surface
    #
    # Carried over from ConnectAppFakeGps, which this app absorbed. The method
    # bodies are the same publishes; every signature gained the instance name,
    # because the GPS simulator is multi-instance here and was not there.

    def select_mavros_node(self, instance_name, node_namespace):
        """Select the target mavros (mavlink) node namespace to inject GPS_INPUT into."""
        self.set_control_value(KIND_GPS, instance_name, 'mavros_node', node_namespace)

    def set_satellites_visible(self, instance_name, sat_count):
        """Set the satellite count reported in the injected GPS_INPUT message."""
        self.set_control_value(KIND_GPS, instance_name, 'satellites_visible', int(sat_count))

    def set_gps_pub_rate(self, instance_name, rate_hz):
        """Set the simulated GPS publish rate in Hz (clamped to 1-100 by the control bounds)."""
        self.set_control_value(KIND_GPS, instance_name, 'gps_pub_rate_hz', float(rate_hz))

    def set_start_location(self, instance_name, latitude, longitude, altitude):
        """Set the start location the simulated GPS position seeds and resets to."""
        self.set_control_value(KIND_GPS, instance_name, 'start_latitude',   float(latitude))
        self.set_control_value(KIND_GPS, instance_name, 'start_longitude',  float(longitude))
        self.set_control_value(KIND_GPS, instance_name, 'start_altitude_m', float(altitude))

    def use_current_location(self, instance_name):
        """Copy the live simulated position into the instance's start and goto controls."""
        self.press_button(KIND_GPS, instance_name, 'use_current_location')

    def reset_location(self, instance_name, latitude, longitude, altitude):
        """Reset one GPS instance's simulated home position to a new WGS84 geopoint.

        A command, not a control: it carries a whole geopoint in one message,
        which a control set cannot write atomically.
        """
        ns = self._instanceNamespace(KIND_GPS, instance_name)
        if ns is None:
            return
        msg = GeoPoint()
        msg.latitude  = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude  = float(altitude)
        self._publishDynamic(ns + '/reset_location', GeoPoint, msg)

    def go_stop(self, instance_name):
        """Stop one GPS instance's active simulated move and hold the current position."""
        ns = self._instanceNamespace(KIND_GPS, instance_name)
        if ns is None:
            return
        self._publishDynamic(ns + '/go_stop', Empty, Empty())

    def goto_position(self, instance_name, x, y, z):
        """Move one GPS instance's simulated position by an ENU offset in meters."""
        ns = self._instanceNamespace(KIND_GPS, instance_name)
        if ns is None:
            return
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self._publishDynamic(ns + '/goto_position', Point, msg)

    def goto_location(self, instance_name, latitude, longitude, altitude):
        """Move one GPS instance's simulated position to an absolute WGS84 geopoint."""
        ns = self._instanceNamespace(KIND_GPS, instance_name)
        if ns is None:
            return
        msg = GeoPoint()
        msg.latitude  = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude  = float(altitude)
        self._publishDynamic(ns + '/goto_location', GeoPoint, msg)

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

    def _instanceNamespace(self, kind, instance_name):
        if kind not in KINDS or not instance_name:
            return None
        return nepi_sdk.create_namespace(
            self.namespace, kind + '_instances/' + str(instance_name))

    def _controlsNamespace(self, kind, instance_name, suffix):
        # Mirror of instanceControlsName() in the node: the instance's path
        # below the app namespace, flattened, plus the section suffix. The set
        # hangs off the APP namespace, not the instance namespace, because a
        # ControlsIF can only root itself at the node.
        if kind not in KINDS or not instance_name:
            return None
        tail = kind + '_instances_' + str(instance_name) + '_' + suffix
        return nepi_sdk.create_namespace(self.namespace, tail)

    def _controlSection(self, control_name):
        if control_name in _CONTROL_SECTIONS:
            return _CONTROL_SECTIONS[control_name]
        field = control_name
        for prefix in _ROW_PREFIXES:
            if field.startswith(prefix):
                field = field[len(prefix):]
                break
        for tail, suffix in _SECTION_BY_FIELD_TAIL:
            if field.endswith(tail):
                return suffix
        return None

    def _geoControl(self, kind, axis):
        if kind == KIND_GPS:
            return 'start_' + axis
        return kind + '_' + axis

    def _publishDynamic(self, topic, msg_type, msg):
        pub = self._dyn_pubs.get(topic, None)
        if pub is None:
            try:
                pub = nepi_sdk.create_publisher(topic, msg_type, queue_size=1)
                self._dyn_pubs[topic] = pub
                # A brand new publisher has no subscriber connection yet, so the
                # first message on it is dropped without this settle.
                time.sleep(0.5)
            except Exception as e:
                self.msg_if.pub_warn("Failed to create publisher " + str(topic) + ": " + str(e))
                return
        try:
            pub.publish(msg)
        except Exception as e:
            self.msg_if.pub_warn("Failed to publish " + str(topic) + ": " + str(e))

    def _unregisterNode(self):
        self.connected = False
        for topic in list(self._dyn_pubs.keys()):
            try:
                self._dyn_pubs[topic].unregister()
            except Exception:
                pass
        self._dyn_pubs = {}
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
