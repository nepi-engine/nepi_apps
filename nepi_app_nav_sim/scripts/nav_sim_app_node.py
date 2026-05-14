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
# Nav Sim App: runs NMEA and/or HNav TCP simulator servers that the
# corresponding NPX driver nodes (npx_nmea_udp_node, npx_hnav_tcu_node)
# can connect to.  Position/orientation/speed are independently configurable
# for each simulator via ROS topics and the NEPI RUI.

import copy
import datetime
import math
import socket
import struct
import threading
import time

from std_msgs.msg import Bool, Float32

from nepi_app_nav_sim.msg import NepiAppNavSimStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import NavPoseIF


#########################################
# Factory Defaults

FACTORY_LATITUDE     = 47.6205
FACTORY_LONGITUDE    = -122.3493
FACTORY_ALTITUDE_M   = 10.0
FACTORY_DEPTH_M      = 0.0
FACTORY_HEADING_DEG  = 0.0
FACTORY_ROLL_DEG     = 0.0
FACTORY_PITCH_DEG    = 0.0
FACTORY_SPEED_MS     = 0.0
FACTORY_NMEA_ENABLED = False
FACTORY_HNAV_ENABLED = False
FACTORY_NMEA_PORT    = 50000
FACTORY_HNAV_PORT    = 16718

STATUS_RATE_HZ = 1.0

#########################################
# HNav packet helpers
#########################################

_PACKET_SIZE     = 67
_START_BYTE1     = 0xAA
_START_BYTE2     = 0xBF
_PROTO_VER       = 0x00
_DATA_SIZE_LO    = 0x37
_DATA_SIZE_HI    = 0x00
_LAT_SCALE       = 90.0  / 2147483648.0
_LON_SCALE       = 180.0 / 2147483648.0
_DEPTH_SCALE     = 0.001
_ALT_SCALE       = 0.01
_ORIENT_SCALE    = 0.01
_VEL_SCALE       = 0.001
_SOUND_VEL_SCALE = 0.1
_TEMP_SCALE      = 0.01
_DEFAULT_STATUS  = 0x0002
_DATA_FMT        = '<BQiiiHhhHhhhhhhHhfHHH'


def _reflect(value, bit_num):
    out, bit = 0, 1
    for i in range(bit_num - 1, -1, -1):
        if value & (1 << i):
            out |= bit
        bit <<= 1
    return out


def _crc16(data_bytes):
    crc = 0xFFFF
    for byte in data_bytes:
        current = _reflect(byte, 8)
        j = 0x80
        while j:
            bit = crc & 0x8000
            crc = (crc << 1) & 0xFFFF
            if current & j:
                bit ^= 0x8000
            if bit:
                crc ^= 0x1021
            j >>= 1
    crc = _reflect(crc, 16)
    crc ^= 0xFFFF
    return crc & 0xFFFF


def _build_hnav_packet(lat_deg, lon_deg, depth_m, alt_m,
                       roll_deg, pitch_deg, heading_deg,
                       vel_fwd_ms=0.0, sound_vel_ms=1500.0,
                       temp_c=12.0, pos_qual_m=1.0,
                       status=_DEFAULT_STATUS):
    utc_us = int(time.time() * 1e6)

    def c16s(v): return max(-32768, min(32767, v))
    def c16u(v): return max(0,      min(65535, v))

    data = struct.pack(
        _DATA_FMT,
        0,
        utc_us & 0xFFFFFFFFFFFFFFFF,
        int(lat_deg  / _LAT_SCALE),
        int(lon_deg  / _LON_SCALE),
        int(depth_m  / _DEPTH_SCALE),
        c16u(max(0, int(alt_m    / _ALT_SCALE))),
        c16s(int(roll_deg  / _ORIENT_SCALE)),
        c16s(int(pitch_deg / _ORIENT_SCALE)),
        c16u(int((heading_deg % 360.0) / _ORIENT_SCALE)),
        c16s(int(vel_fwd_ms / _VEL_SCALE)),
        0, 0, 0, 0, 0,
        c16u(int(sound_vel_ms / _SOUND_VEL_SCALE)),
        c16s(int(temp_c      / _TEMP_SCALE)),
        float(pos_qual_m),
        10, 10,
        c16u(status),
    )
    header = bytes([_START_BYTE1, _START_BYTE2, _PROTO_VER,
                    0x00, 0x00, _DATA_SIZE_LO, _DATA_SIZE_HI,
                    0x00, 0x00, 0x00])
    body = header + data
    return body + struct.pack('<H', _crc16(body))


#########################################
# NMEA sentence helpers
#########################################

def _dd_to_nmea(lat, lon):
    alat, alon = abs(lat), abs(lon)
    lat_deg, lon_deg = int(alat), int(alon)
    lat_min = (alat - lat_deg) * 60.0
    lon_min = (alon - lon_deg) * 60.0
    ns = 'N' if lat >= 0 else 'S'
    ew = 'E' if lon >= 0 else 'W'
    return f"{lat_deg:02d}{lat_min:07.4f}", ns, f"{lon_deg:03d}{lon_min:07.4f}", ew


def _nmea_checksum(s):
    c = 0
    for ch in s:
        c ^= ord(ch)
    return f"{c:02X}"


def _make_GGA(lat, lon, alt_m):
    now = datetime.datetime.utcnow().strftime("%H%M%S")
    ls, ns, lons, ew = _dd_to_nmea(lat, lon)
    parts = ["GPGGA", now, ls, ns, lons, ew, "1", "08", "1.0",
             f"{alt_m:.1f}", "M", "0.0", "M", "", ""]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


def _make_RMC(lat, lon, sog_kts, cog_deg):
    now = datetime.datetime.utcnow()
    ts, ds = now.strftime("%H%M%S"), now.strftime("%d%m%y")
    ls, ns, lons, ew = _dd_to_nmea(lat, lon)
    parts = ["GPRMC", ts, "A", ls, ns, lons, ew,
             f"{sog_kts:.1f}", f"{cog_deg:.1f}", ds, "", ""]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


def _make_VTG(cog_deg, sog_kts):
    parts = ["GPVTG", f"{cog_deg:.1f}", "T", "", "M",
             f"{sog_kts:.1f}", "N", f"{sog_kts*1.852:.1f}", "K"]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


def _make_HDG(heading_deg):
    parts = ["HCHDG", f"{heading_deg:.1f}", "", "", "", ""]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


#########################################
# Node Class
#########################################

class NepiNavSimApp(object):

    DEFAULT_NODE_NAME = "app_nav_sim"

    nmea_sim_enabled = FACTORY_NMEA_ENABLED
    hnav_sim_enabled = FACTORY_HNAV_ENABLED

    nmea_latitude    = FACTORY_LATITUDE
    nmea_longitude   = FACTORY_LONGITUDE
    nmea_altitude_m  = FACTORY_ALTITUDE_M
    nmea_heading_deg = FACTORY_HEADING_DEG
    nmea_speed_ms    = FACTORY_SPEED_MS
    nmea_port        = FACTORY_NMEA_PORT

    hnav_latitude    = FACTORY_LATITUDE
    hnav_longitude   = FACTORY_LONGITUDE
    hnav_altitude_m  = FACTORY_ALTITUDE_M
    hnav_depth_m     = FACTORY_DEPTH_M
    hnav_heading_deg = FACTORY_HEADING_DEG
    hnav_roll_deg    = FACTORY_ROLL_DEG
    hnav_pitch_deg   = FACTORY_PITCH_DEG
    hnav_speed_ms    = FACTORY_SPEED_MS
    hnav_port        = FACTORY_HNAV_PORT

    nmea_connected = False
    hnav_connected = False

    node_if = None

    _lock            = threading.Lock()
    _nmea_stop       = None
    _hnav_stop       = None
    _nmea_navpose_if = None
    _hnav_navpose_if = None

    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name     = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name      = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        self.CFGS_DICT = {
            'init_callback':          self.initCb,
            'reset_callback':         self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs':           True,
            'namespace':              self.node_namespace,
        }

        ns = self.node_namespace
        self.PARAMS_DICT = {
            'nmea_sim_enabled': {'namespace': ns, 'factory_val': self.nmea_sim_enabled},
            'hnav_sim_enabled': {'namespace': ns, 'factory_val': self.hnav_sim_enabled},
            'nmea_latitude':    {'namespace': ns, 'factory_val': self.nmea_latitude},
            'nmea_longitude':   {'namespace': ns, 'factory_val': self.nmea_longitude},
            'nmea_altitude_m':  {'namespace': ns, 'factory_val': self.nmea_altitude_m},
            'nmea_heading_deg': {'namespace': ns, 'factory_val': self.nmea_heading_deg},
            'nmea_speed_ms':    {'namespace': ns, 'factory_val': self.nmea_speed_ms},
            'nmea_port':        {'namespace': ns, 'factory_val': self.nmea_port},
            'hnav_latitude':    {'namespace': ns, 'factory_val': self.hnav_latitude},
            'hnav_longitude':   {'namespace': ns, 'factory_val': self.hnav_longitude},
            'hnav_altitude_m':  {'namespace': ns, 'factory_val': self.hnav_altitude_m},
            'hnav_depth_m':     {'namespace': ns, 'factory_val': self.hnav_depth_m},
            'hnav_heading_deg': {'namespace': ns, 'factory_val': self.hnav_heading_deg},
            'hnav_roll_deg':    {'namespace': ns, 'factory_val': self.hnav_roll_deg},
            'hnav_pitch_deg':   {'namespace': ns, 'factory_val': self.hnav_pitch_deg},
            'hnav_speed_ms':    {'namespace': ns, 'factory_val': self.hnav_speed_ms},
            'hnav_port':        {'namespace': ns, 'factory_val': self.hnav_port},
        }

        self.PUBS_DICT = {
            'status_pub': {
                'namespace': ns, 'topic': 'status',
                'msg': NepiAppNavSimStatus, 'qsize': 1, 'latch': True,
            }
        }

        self.SUBS_DICT = {
            'set_nmea_enabled':   {'namespace': ns, 'topic': 'set_nmea_enabled',   'msg': Bool,    'qsize': 10, 'callback': self.setNmeaEnabledCb,   'callback_args': ()},
            'set_hnav_enabled':   {'namespace': ns, 'topic': 'set_hnav_enabled',   'msg': Bool,    'qsize': 10, 'callback': self.setHnavEnabledCb,   'callback_args': ()},
            'set_nmea_latitude':  {'namespace': ns, 'topic': 'set_nmea_latitude',  'msg': Float32, 'qsize': 10, 'callback': self.setNmeaLatitudeCb,  'callback_args': ()},
            'set_nmea_longitude': {'namespace': ns, 'topic': 'set_nmea_longitude', 'msg': Float32, 'qsize': 10, 'callback': self.setNmeaLongitudeCb, 'callback_args': ()},
            'set_nmea_altitude':  {'namespace': ns, 'topic': 'set_nmea_altitude',  'msg': Float32, 'qsize': 10, 'callback': self.setNmeaAltitudeCb,  'callback_args': ()},
            'set_nmea_heading':   {'namespace': ns, 'topic': 'set_nmea_heading',   'msg': Float32, 'qsize': 10, 'callback': self.setNmeaHeadingCb,   'callback_args': ()},
            'set_nmea_speed':     {'namespace': ns, 'topic': 'set_nmea_speed',     'msg': Float32, 'qsize': 10, 'callback': self.setNmeaSpeedCb,     'callback_args': ()},
            'set_hnav_latitude':  {'namespace': ns, 'topic': 'set_hnav_latitude',  'msg': Float32, 'qsize': 10, 'callback': self.setHnavLatitudeCb,  'callback_args': ()},
            'set_hnav_longitude': {'namespace': ns, 'topic': 'set_hnav_longitude', 'msg': Float32, 'qsize': 10, 'callback': self.setHnavLongitudeCb, 'callback_args': ()},
            'set_hnav_altitude':  {'namespace': ns, 'topic': 'set_hnav_altitude',  'msg': Float32, 'qsize': 10, 'callback': self.setHnavAltitudeCb,  'callback_args': ()},
            'set_hnav_depth':     {'namespace': ns, 'topic': 'set_hnav_depth',     'msg': Float32, 'qsize': 10, 'callback': self.setHnavDepthCb,     'callback_args': ()},
            'set_hnav_heading':   {'namespace': ns, 'topic': 'set_hnav_heading',   'msg': Float32, 'qsize': 10, 'callback': self.setHnavHeadingCb,   'callback_args': ()},
            'set_hnav_roll':      {'namespace': ns, 'topic': 'set_hnav_roll',      'msg': Float32, 'qsize': 10, 'callback': self.setHnavRollCb,      'callback_args': ()},
            'set_hnav_pitch':     {'namespace': ns, 'topic': 'set_hnav_pitch',     'msg': Float32, 'qsize': 10, 'callback': self.setHnavPitchCb,     'callback_args': ()},
            'set_hnav_speed':     {'namespace': ns, 'topic': 'set_hnav_speed',     'msg': Float32, 'qsize': 10, 'callback': self.setHnavSpeedCb,     'callback_args': ()},
        }

        self.node_if = NodeClassIF(
            configs_dict=self.CFGS_DICT,
            params_dict=self.PARAMS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
        )
        self.node_if.wait_for_ready()

        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(1.0 / STATUS_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    ###################
    ## Control Callbacks

    def setNmeaEnabledCb(self, msg):
        with self._lock:
            self.nmea_sim_enabled = msg.data
        self._apply_nmea_state()
        self._save_param('nmea_sim_enabled', self.nmea_sim_enabled)

    def setHnavEnabledCb(self, msg):
        with self._lock:
            self.hnav_sim_enabled = msg.data
        self._apply_hnav_state()
        self._save_param('hnav_sim_enabled', self.hnav_sim_enabled)

    def setNmeaLatitudeCb(self, msg):
        with self._lock:
            self.nmea_latitude = float(msg.data)
        self._save_param('nmea_latitude', self.nmea_latitude)
        self.publish_status()

    def setNmeaLongitudeCb(self, msg):
        with self._lock:
            self.nmea_longitude = float(msg.data)
        self._save_param('nmea_longitude', self.nmea_longitude)
        self.publish_status()

    def setNmeaAltitudeCb(self, msg):
        with self._lock:
            self.nmea_altitude_m = float(msg.data)
        self._save_param('nmea_altitude_m', self.nmea_altitude_m)
        self.publish_status()

    def setNmeaHeadingCb(self, msg):
        with self._lock:
            self.nmea_heading_deg = float(msg.data) % 360.0
        self._save_param('nmea_heading_deg', self.nmea_heading_deg)
        self.publish_status()

    def setNmeaSpeedCb(self, msg):
        with self._lock:
            self.nmea_speed_ms = max(0.0, float(msg.data))
        self._save_param('nmea_speed_ms', self.nmea_speed_ms)
        self.publish_status()

    def setHnavLatitudeCb(self, msg):
        with self._lock:
            self.hnav_latitude = float(msg.data)
        self._save_param('hnav_latitude', self.hnav_latitude)
        self.publish_status()

    def setHnavLongitudeCb(self, msg):
        with self._lock:
            self.hnav_longitude = float(msg.data)
        self._save_param('hnav_longitude', self.hnav_longitude)
        self.publish_status()

    def setHnavAltitudeCb(self, msg):
        with self._lock:
            self.hnav_altitude_m = float(msg.data)
        self._save_param('hnav_altitude_m', self.hnav_altitude_m)
        self.publish_status()

    def setHnavDepthCb(self, msg):
        with self._lock:
            self.hnav_depth_m = float(msg.data)
        self._save_param('hnav_depth_m', self.hnav_depth_m)
        self.publish_status()

    def setHnavHeadingCb(self, msg):
        with self._lock:
            self.hnav_heading_deg = float(msg.data) % 360.0
        self._save_param('hnav_heading_deg', self.hnav_heading_deg)
        self.publish_status()

    def setHnavRollCb(self, msg):
        with self._lock:
            self.hnav_roll_deg = float(msg.data)
        self._save_param('hnav_roll_deg', self.hnav_roll_deg)
        self.publish_status()

    def setHnavPitchCb(self, msg):
        with self._lock:
            self.hnav_pitch_deg = float(msg.data)
        self._save_param('hnav_pitch_deg', self.hnav_pitch_deg)
        self.publish_status()

    def setHnavSpeedCb(self, msg):
        with self._lock:
            self.hnav_speed_ms = max(0.0, float(msg.data))
        self._save_param('hnav_speed_ms', self.hnav_speed_ms)
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            with self._lock:
                self.nmea_sim_enabled = self.node_if.get_param('nmea_sim_enabled')
                self.hnav_sim_enabled = self.node_if.get_param('hnav_sim_enabled')
                self.nmea_latitude    = float(self.node_if.get_param('nmea_latitude'))
                self.nmea_longitude   = float(self.node_if.get_param('nmea_longitude'))
                self.nmea_altitude_m  = float(self.node_if.get_param('nmea_altitude_m'))
                self.nmea_heading_deg = float(self.node_if.get_param('nmea_heading_deg'))
                self.nmea_speed_ms    = float(self.node_if.get_param('nmea_speed_ms'))
                self.nmea_port        = int(self.node_if.get_param('nmea_port'))
                self.hnav_latitude    = float(self.node_if.get_param('hnav_latitude'))
                self.hnav_longitude   = float(self.node_if.get_param('hnav_longitude'))
                self.hnav_altitude_m  = float(self.node_if.get_param('hnav_altitude_m'))
                self.hnav_depth_m     = float(self.node_if.get_param('hnav_depth_m'))
                self.hnav_heading_deg = float(self.node_if.get_param('hnav_heading_deg'))
                self.hnav_roll_deg    = float(self.node_if.get_param('hnav_roll_deg'))
                self.hnav_pitch_deg   = float(self.node_if.get_param('hnav_pitch_deg'))
                self.hnav_speed_ms    = float(self.node_if.get_param('hnav_speed_ms'))
                self.hnav_port        = int(self.node_if.get_param('hnav_port'))
        if do_updates:
            self._apply_nmea_state()
            self._apply_hnav_state()
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        self.initCb(do_updates=do_updates)


    ###################
    ## Status Publisher

    def statusPublishCb(self, timer):
        self.publish_status()
        self._publishNavpose()

    def publish_status(self):
        msg = NepiAppNavSimStatus()
        with self._lock:
            msg.nmea_sim_enabled = self.nmea_sim_enabled
            msg.hnav_sim_enabled = self.hnav_sim_enabled
            msg.nmea_connected   = self.nmea_connected
            msg.hnav_connected   = self.hnav_connected
            msg.nmea_port        = self.nmea_port
            msg.hnav_port        = self.hnav_port
            msg.nmea_latitude    = self.nmea_latitude
            msg.nmea_longitude   = self.nmea_longitude
            msg.nmea_altitude_m  = self.nmea_altitude_m
            msg.nmea_heading_deg = self.nmea_heading_deg
            msg.nmea_speed_ms    = self.nmea_speed_ms
            msg.hnav_latitude    = self.hnav_latitude
            msg.hnav_longitude   = self.hnav_longitude
            msg.hnav_altitude_m  = self.hnav_altitude_m
            msg.hnav_depth_m     = self.hnav_depth_m
            msg.hnav_heading_deg = self.hnav_heading_deg
            msg.hnav_roll_deg    = self.hnav_roll_deg
            msg.hnav_pitch_deg   = self.hnav_pitch_deg
            msg.hnav_speed_ms    = self.hnav_speed_ms
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', msg)


    #######################
    ## TCP Server Management

    def _apply_nmea_state(self):
        with self._lock:
            enabled = self.nmea_sim_enabled
            port    = self.nmea_port
        if enabled and self._nmea_stop is None:
            self._nmea_navpose_if = NavPoseIF(
                namespace=self.node_namespace + '/nmea',
                data_source_description='nav_sim_nmea',
                data_ref_description='WGS84',
                pub_navpose=True,
                pub_location=True,
                pub_heading=True,
                pub_altitude=True,
                msg_if=self.msg_if,
            )
            self._start_nmea_server(port)
        elif not enabled and self._nmea_stop is not None:
            self._stop_server('nmea')
            self._nmea_navpose_if = None
        self.publish_status()

    def _apply_hnav_state(self):
        with self._lock:
            enabled = self.hnav_sim_enabled
            port    = self.hnav_port
        if enabled and self._hnav_stop is None:
            self._hnav_navpose_if = NavPoseIF(
                namespace=self.node_namespace + '/hnav',
                data_source_description='nav_sim_hnav',
                data_ref_description='WGS84',
                pub_navpose=True,
                pub_location=True,
                pub_heading=True,
                pub_orientation=True,
                pub_altitude=True,
                pub_depth=True,
                msg_if=self.msg_if,
            )
            self._start_hnav_server(port)
        elif not enabled and self._hnav_stop is not None:
            self._stop_server('hnav')
            self._hnav_navpose_if = None
        self.publish_status()

    def _start_nmea_server(self, port):
        stop_evt = threading.Event()
        self._nmea_stop = stop_evt
        t = threading.Thread(
            target=self._nmea_server_loop,
            args=('127.0.0.1', port, stop_evt),
            daemon=True,
        )
        t.start()
        self.msg_if.pub_info(f"NMEA simulator started on port {port}")

    def _start_hnav_server(self, port):
        stop_evt = threading.Event()
        self._hnav_stop = stop_evt
        t = threading.Thread(
            target=self._hnav_server_loop,
            args=('127.0.0.1', port, stop_evt),
            daemon=True,
        )
        t.start()
        self.msg_if.pub_info(f"HNav simulator started on port {port}")

    def _stop_server(self, which):
        if which == 'nmea' and self._nmea_stop is not None:
            self._nmea_stop.set()
            self._nmea_stop = None
            with self._lock:
                self.nmea_connected = False
            self.msg_if.pub_info("NMEA simulator stopped")
        elif which == 'hnav' and self._hnav_stop is not None:
            self._hnav_stop.set()
            self._hnav_stop = None
            with self._lock:
                self.hnav_connected = False
            self.msg_if.pub_info("HNav simulator stopped")


    #######################
    ## NMEA TCP Server Loop

    def _nmea_server_loop(self, host, port, stop_evt):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((host, port))
            srv.listen(1)
            srv.settimeout(0.5)
            while not stop_evt.is_set():
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with self._lock:
                    self.nmea_connected = True
                self.publish_status()
                self._serve_nmea_client(conn, stop_evt)
                with self._lock:
                    self.nmea_connected = False
                self.publish_status()
        except Exception as e:
            self.msg_if.pub_warn(f"NMEA server error: {e}")
        finally:
            try:
                srv.close()
            except Exception:
                pass

    def _serve_nmea_client(self, conn, stop_evt):
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat     = self.nmea_latitude
                    lon     = self.nmea_longitude
                    alt     = self.nmea_altitude_m
                    heading = self.nmea_heading_deg
                    speed   = self.nmea_speed_ms
                sog_kts = speed * 1.944
                lines = [
                    _make_GGA(lat, lon, alt),
                    _make_RMC(lat, lon, sog_kts, heading),
                    _make_VTG(heading, sog_kts),
                    _make_HDG(heading),
                ]
                payload = ("\r\n".join(lines) + "\r\n").encode("ascii")
                conn.sendall(payload)
                if speed > 0:
                    self._deadReckonNmea(1.0 / 5)
                time.sleep(1.0 / 5)
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass


    #######################
    ## HNav TCP Server Loop

    def _hnav_server_loop(self, host, port, stop_evt):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((host, port))
            srv.listen(1)
            srv.settimeout(0.5)
            while not stop_evt.is_set():
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with self._lock:
                    self.hnav_connected = True
                self.publish_status()
                self._serve_hnav_client(conn, stop_evt)
                with self._lock:
                    self.hnav_connected = False
                self.publish_status()
        except Exception as e:
            self.msg_if.pub_warn(f"HNav server error: {e}")
        finally:
            try:
                srv.close()
            except Exception:
                pass

    def _serve_hnav_client(self, conn, stop_evt):
        rate_hz = 10
        period  = 1.0 / rate_hz
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat     = self.hnav_latitude
                    lon     = self.hnav_longitude
                    alt     = self.hnav_altitude_m
                    depth   = self.hnav_depth_m
                    heading = self.hnav_heading_deg
                    roll    = self.hnav_roll_deg
                    pitch   = self.hnav_pitch_deg
                    speed   = self.hnav_speed_ms
                packet = _build_hnav_packet(
                    lat_deg=lat, lon_deg=lon,
                    depth_m=depth, alt_m=alt,
                    roll_deg=roll, pitch_deg=pitch,
                    heading_deg=heading,
                    vel_fwd_ms=speed,
                )
                conn.sendall(packet)
                if speed > 0:
                    self._deadReckonHnav(period)
                time.sleep(period)
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass


    #######################
    ## Dead-Reckoning

    def _deadReckonNmea(self, dt_sec):
        with self._lock:
            heading_rad = math.radians(self.nmea_heading_deg)
            speed       = self.nmea_speed_ms
            lat         = self.nmea_latitude
            lon         = self.nmea_longitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) \
                 * math.sin(heading_rad)
        with self._lock:
            self.nmea_latitude  += dlat
            self.nmea_longitude += dlon

    def _deadReckonHnav(self, dt_sec):
        with self._lock:
            heading_rad = math.radians(self.hnav_heading_deg)
            speed       = self.hnav_speed_ms
            lat         = self.hnav_latitude
            lon         = self.hnav_longitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) \
                 * math.sin(heading_rad)
        with self._lock:
            self.hnav_latitude  += dlat
            self.hnav_longitude += dlon


    #######################
    ## NavPose Publishing

    def _publishNavpose(self):
        if self._nmea_navpose_if is not None:
            self._nmea_navpose_if.publish_navpose(self._buildNmeaNavposeDict())
        if self._hnav_navpose_if is not None:
            self._hnav_navpose_if.publish_navpose(self._buildHnavNavposeDict())

    def _buildNmeaNavposeDict(self):
        with self._lock:
            lat     = self.nmea_latitude
            lon     = self.nmea_longitude
            alt     = self.nmea_altitude_m
            heading = self.nmea_heading_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']  = True;  d['latitude']    = lat;     d['longitude']   = lon;  d['time_location']  = t
        d['has_heading']   = True;  d['heading_deg'] = heading;                          d['time_heading']   = t
        d['has_altitude']  = True;  d['altitude_m']  = alt;                              d['time_altitude']  = t
        return d

    def _buildHnavNavposeDict(self):
        with self._lock:
            lat     = self.hnav_latitude
            lon     = self.hnav_longitude
            alt     = self.hnav_altitude_m
            depth   = self.hnav_depth_m
            heading = self.hnav_heading_deg
            roll    = self.hnav_roll_deg
            pitch   = self.hnav_pitch_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']    = True;  d['latitude']    = lat;    d['longitude']  = lon;   d['time_location']    = t
        d['has_heading']     = True;  d['heading_deg'] = heading;                         d['time_heading']     = t
        d['has_orientation'] = True;  d['roll_deg']    = roll;   d['pitch_deg']  = pitch; d['yaw_deg'] = heading; d['time_orientation'] = t
        d['has_altitude']    = True;  d['altitude_m']  = alt;                             d['time_altitude']    = t
        d['has_depth']       = True;  d['depth_m']     = depth;                           d['time_depth']       = t
        return d


    #######################
    ## Utility

    def _save_param(self, key, val):
        if self.node_if is not None:
            self.node_if.set_param(key, val)
            self.node_if.save_config()

    def cleanup_actions(self):
        self._stop_server('nmea')
        self._stop_server('hnav')
        self.msg_if.pub_info("NAV_SIM_APP: Shutting down")


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiNavSimApp()
