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

import copy
import datetime
import math
import os
import random
import socket
import struct
import threading
import time
import yaml

from std_msgs.msg import Bool, Empty, Float32, String
from nepi_interfaces.msg import UpdateString

from nepi_app_nav_sim.msg import (
    NepiAppNmeaSimStatus,
    NepiAppHNavSimStatus,
    NepiAppNavSimMasterStatus,
)

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav

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

_NMEA_MOVE_FIELDS = (
    'nmea_latitude', 'nmea_longitude', 'nmea_altitude_m',
    'nmea_heading_deg', 'nmea_speed_ms',
)

_HNAV_MOVE_FIELDS = (
    'hnav_latitude', 'hnav_longitude', 'hnav_altitude_m', 'hnav_depth_m',
    'hnav_heading_deg', 'hnav_roll_deg', 'hnav_pitch_deg', 'hnav_speed_ms',
)

_SIN_FIELDS = ('hnav_heading_deg', 'hnav_roll_deg', 'hnav_pitch_deg')


#########################################
# HNav packet helpers

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
# NMEA sim instance

class NmeaSimInstance:
    """Self-contained NMEA nav sim: own ROS subs/pub, TCP server, move thread."""

    def __init__(self, name, base_ns, msg_if):
        self.name    = name
        self._ns     = base_ns + '/nmea_instances/' + name
        self._msg_if = msg_if
        self._lock   = threading.Lock()

        self.nmea_sim_enabled = FACTORY_NMEA_ENABLED
        self.nmea_latitude    = FACTORY_LATITUDE
        self.nmea_longitude   = FACTORY_LONGITUDE
        self.nmea_altitude_m  = FACTORY_ALTITUDE_M
        self.nmea_heading_deg = FACTORY_HEADING_DEG
        self.nmea_speed_ms    = FACTORY_SPEED_MS
        self.nmea_port        = FACTORY_NMEA_PORT
        self.nmea_connected   = False

        for f in _NMEA_MOVE_FIELDS:
            setattr(self, 'enable_move_' + f,   False)
            setattr(self, 'move_step_' + f,      0.0)
            setattr(self, 'move_rate_hz_' + f,   1.0)

        self._stop_evt       = None
        self._navpose_if     = None
        self._move_last_tick = {f: 0.0 for f in _NMEA_MOVE_FIELDS}
        self._alive          = True

        self._status_pub = nepi_sdk.create_publisher(
            self._ns + '/status', NepiAppNmeaSimStatus, queue_size=1, latch=True
        )
        self._subs = []
        self._registerSubs(self._ns)
        threading.Thread(target=self._moveThreadLoop, daemon=True).start()

    def _registerSubs(self, ns):
        S = self._subs.append
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_enabled',   Bool,    self._setNmeaEnabledCb))
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_latitude',  Float32, self._setLatitudeCb))
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_longitude', Float32, self._setLongitudeCb))
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_altitude',  Float32, self._setAltitudeCb))
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_heading',   Float32, self._setHeadingCb))
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_speed',     Float32, self._setSpeedCb))
        for field in _NMEA_MOVE_FIELDS:
            S(nepi_sdk.create_subscriber(ns + '/set_enable_move_' + field,
                               Bool,    lambda msg, f=field: self._setEnableMoveCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_move_step_' + field,
                               Float32, lambda msg, f=field: self._setMoveStepCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_move_rate_hz_' + field,
                               Float32, lambda msg, f=field: self._setMoveRateHzCb(msg, f)))

    def _setNmeaEnabledCb(self, msg):
        with self._lock: self.nmea_sim_enabled = msg.data
        self._applyState()

    def _setLatitudeCb(self, msg):
        with self._lock: self.nmea_latitude = float(msg.data)
        self.publish_status()

    def _setLongitudeCb(self, msg):
        with self._lock: self.nmea_longitude = float(msg.data)
        self.publish_status()

    def _setAltitudeCb(self, msg):
        with self._lock: self.nmea_altitude_m = float(msg.data)
        self.publish_status()

    def _setHeadingCb(self, msg):
        with self._lock: self.nmea_heading_deg = float(msg.data) % 360.0
        self.publish_status()

    def _setSpeedCb(self, msg):
        with self._lock: self.nmea_speed_ms = max(0.0, float(msg.data))
        self.publish_status()

    def _setEnableMoveCb(self, msg, field):
        with self._lock: setattr(self, 'enable_move_' + field, msg.data)
        self.publish_status()

    def _setMoveStepCb(self, msg, field):
        with self._lock: setattr(self, 'move_step_' + field, float(msg.data))
        self.publish_status()

    def _setMoveRateHzCb(self, msg, field):
        with self._lock: setattr(self, 'move_rate_hz_' + field, float(msg.data))
        self.publish_status()

    def _moveThreadLoop(self):
        while self._alive:
            now = time.time()
            changed = False
            for field in _NMEA_MOVE_FIELDS:
                if not getattr(self, 'enable_move_' + field):
                    continue
                rate = getattr(self, 'move_rate_hz_' + field)
                if rate <= 0.0:
                    continue
                if (now - self._move_last_tick[field]) >= 1.0 / rate:
                    with self._lock:
                        setattr(self, field,
                                getattr(self, field) + getattr(self, 'move_step_' + field))
                    self._move_last_tick[field] = now
                    changed = True
            if changed:
                self.publish_status()
            time.sleep(1.0 / 20)

    def _applyState(self):
        with self._lock:
            enabled = self.nmea_sim_enabled
            port    = self.nmea_port
        if enabled and self._stop_evt is None:
            self._navpose_if = NavPoseIF(
                namespace=self._ns + '/navpose',
                data_source_description=f'nav_sim_{self.name}',
                data_ref_description='WGS84',
                pub_navpose=True, pub_location=True,
                pub_heading=True, pub_altitude=True,
                msg_if=self._msg_if,
            )
            stop_evt = threading.Event()
            self._stop_evt = stop_evt
            threading.Thread(target=self._serverLoop,
                             args=('127.0.0.1', port, stop_evt), daemon=True).start()
        elif not enabled and self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
            with self._lock: self.nmea_connected = False
            self._navpose_if = None
        self.publish_status()

    def _serverLoop(self, host, port, stop_evt):
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
                with self._lock: self.nmea_connected = True
                self.publish_status()
                self._serveClient(conn, stop_evt)
                with self._lock: self.nmea_connected = False
                self.publish_status()
        except Exception as e:
            self._msg_if.pub_warn(f"NMEA server error on {self.name}: {e}")
        finally:
            try: srv.close()
            except Exception: pass

    def _serveClient(self, conn, stop_evt):
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat, lon, alt = self.nmea_latitude, self.nmea_longitude, self.nmea_altitude_m
                    heading, speed = self.nmea_heading_deg, self.nmea_speed_ms
                sog_kts = speed * 1.944
                lines = [_make_GGA(lat, lon, alt),
                         _make_RMC(lat, lon, sog_kts, heading),
                         _make_VTG(heading, sog_kts),
                         _make_HDG(heading)]
                conn.sendall(("\r\n".join(lines) + "\r\n").encode("ascii"))
                if speed > 0:
                    self._deadReckon(1.0 / 5)
                time.sleep(1.0 / 5)
        except Exception:
            pass
        finally:
            try: conn.close()
            except Exception: pass

    def _deadReckon(self, dt_sec):
        with self._lock:
            heading_rad = math.radians(self.nmea_heading_deg)
            speed, lat  = self.nmea_speed_ms, self.nmea_latitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) * math.sin(heading_rad)
        with self._lock:
            self.nmea_latitude  += dlat
            self.nmea_longitude += dlon

    def publishNavpose(self):
        if self._navpose_if is None:
            return
        with self._lock:
            lat, lon, alt, heading = (self.nmea_latitude, self.nmea_longitude,
                                      self.nmea_altitude_m, self.nmea_heading_deg)
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location'] = True; d['latitude']    = lat;     d['longitude']   = lon; d['time_location'] = t
        d['has_heading']  = True; d['heading_deg'] = heading;                         d['time_heading']  = t
        d['has_altitude'] = True; d['altitude_m']  = alt;                             d['time_altitude'] = t
        self._navpose_if.publish_navpose(d)

    def publish_status(self):
        self._status_pub.publish(self._buildStatusMsg())

    def _buildStatusMsg(self):
        msg = NepiAppNmeaSimStatus()
        with self._lock:
            msg.nmea_sim_enabled = self.nmea_sim_enabled
            msg.nmea_connected   = self.nmea_connected
            msg.nmea_port        = self.nmea_port
            msg.nmea_latitude    = self.nmea_latitude
            msg.nmea_longitude   = self.nmea_longitude
            msg.nmea_altitude_m  = self.nmea_altitude_m
            msg.nmea_heading_deg = self.nmea_heading_deg
            msg.nmea_speed_ms    = self.nmea_speed_ms
            for f in _NMEA_MOVE_FIELDS:
                setattr(msg, 'enable_move_' + f,  getattr(self, 'enable_move_' + f))
                setattr(msg, 'move_step_' + f,     getattr(self, 'move_step_' + f))
                setattr(msg, 'move_rate_hz_' + f,  getattr(self, 'move_rate_hz_' + f))
        return msg

    def to_dict(self):
        with self._lock:
            d = {
                'nmea_sim_enabled': self.nmea_sim_enabled,
                'nmea_latitude':    self.nmea_latitude,
                'nmea_longitude':   self.nmea_longitude,
                'nmea_altitude_m':  self.nmea_altitude_m,
                'nmea_heading_deg': self.nmea_heading_deg,
                'nmea_speed_ms':    self.nmea_speed_ms,
                'nmea_port':        self.nmea_port,
            }
            for f in _NMEA_MOVE_FIELDS:
                d['enable_move_'  + f] = getattr(self, 'enable_move_'  + f)
                d['move_step_'    + f] = getattr(self, 'move_step_'    + f)
                d['move_rate_hz_' + f] = getattr(self, 'move_rate_hz_' + f)
        return d

    def apply_dict(self, d):
        need_apply = False
        with self._lock:
            for key in ('nmea_latitude', 'nmea_longitude', 'nmea_altitude_m',
                        'nmea_heading_deg', 'nmea_speed_ms'):
                if key in d:
                    setattr(self, key, float(d[key]))
            if 'nmea_port' in d:
                self.nmea_port = int(d['nmea_port'])
            for f in _NMEA_MOVE_FIELDS:
                k = 'enable_move_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'move_step_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'move_rate_hz_' + f
                if k in d: setattr(self, k, float(d[k]))
            if 'nmea_sim_enabled' in d:
                new_enabled = bool(d['nmea_sim_enabled'])
                if new_enabled != self.nmea_sim_enabled:
                    self.nmea_sim_enabled = new_enabled
                    need_apply = True
        if need_apply:
            self._applyState()
        else:
            self.publish_status()

    def cleanup(self):
        self._alive = False
        if self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
        for sub in self._subs:
            sub.unregister()
        if self._navpose_if is not None:
            self._navpose_if.unregister_pubs()
            self._navpose_if = None
        self._status_pub.unregister()


#########################################
# HNav sim instance

class HNavSimInstance:
    """Self-contained HNav nav sim: own ROS subs/pub, TCP server, move+sin+wave thread."""

    def __init__(self, name, base_ns, msg_if):
        self.name    = name
        self._ns     = base_ns + '/hnav_instances/' + name
        self._msg_if = msg_if
        self._lock   = threading.Lock()

        self.hnav_sim_enabled = FACTORY_HNAV_ENABLED
        self.hnav_latitude    = FACTORY_LATITUDE
        self.hnav_longitude   = FACTORY_LONGITUDE
        self.hnav_altitude_m  = FACTORY_ALTITUDE_M
        self.hnav_depth_m     = FACTORY_DEPTH_M
        self.hnav_heading_deg = FACTORY_HEADING_DEG
        self.hnav_roll_deg    = FACTORY_ROLL_DEG
        self.hnav_pitch_deg   = FACTORY_PITCH_DEG
        self.hnav_speed_ms    = FACTORY_SPEED_MS
        self.hnav_port        = FACTORY_HNAV_PORT
        self.hnav_connected   = False

        for f in _HNAV_MOVE_FIELDS:
            setattr(self, 'enable_move_' + f,   False)
            setattr(self, 'move_step_' + f,      0.0)
            setattr(self, 'move_rate_hz_' + f,   1.0)
        for f in _SIN_FIELDS:
            setattr(self, 'enable_sin_' + f,    False)
            setattr(self, 'sin_amplitude_' + f, 5.0)
            setattr(self, 'sin_period_s_' + f,  10.0)
            setattr(self, 'enable_wave_' + f,   False)
            setattr(self, 'sin_spread_' + f,    0.5)

        self._stop_evt        = None
        self._navpose_if      = None
        self._move_last_tick  = {f: 0.0  for f in _HNAV_MOVE_FIELDS}
        self._sin_start_time  = {f: None for f in _SIN_FIELDS}
        self._sin_base        = {f: 0.0  for f in _SIN_FIELDS}
        self._wave_components = {f: None for f in _SIN_FIELDS}
        self._alive           = True

        self._status_pub = nepi_sdk.create_publisher(
            self._ns + '/status', NepiAppHNavSimStatus, queue_size=1, latch=True
        )
        self._subs = []
        self._registerSubs(self._ns)
        threading.Thread(target=self._moveThreadLoop, daemon=True).start()

    def _registerSubs(self, ns):
        S = self._subs.append
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_enabled',   Bool,    self._setHnavEnabledCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_latitude',  Float32, self._setLatitudeCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_longitude', Float32, self._setLongitudeCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_altitude',  Float32, self._setAltitudeCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_depth',     Float32, self._setDepthCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_heading',   Float32, self._setHeadingCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_roll',      Float32, self._setRollCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_pitch',     Float32, self._setPitchCb))
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_speed',     Float32, self._setSpeedCb))
        for field in _HNAV_MOVE_FIELDS:
            S(nepi_sdk.create_subscriber(ns + '/set_enable_move_' + field,
                               Bool,    lambda msg, f=field: self._setEnableMoveCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_move_step_' + field,
                               Float32, lambda msg, f=field: self._setMoveStepCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_move_rate_hz_' + field,
                               Float32, lambda msg, f=field: self._setMoveRateHzCb(msg, f)))
        for field in _SIN_FIELDS:
            S(nepi_sdk.create_subscriber(ns + '/set_enable_sin_' + field,
                               Bool,    lambda msg, f=field: self._setEnableSinCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_sin_amplitude_' + field,
                               Float32, lambda msg, f=field: self._setSinAmplitudeCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_sin_period_s_' + field,
                               Float32, lambda msg, f=field: self._setSinPeriodCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_enable_wave_' + field,
                               Bool,    lambda msg, f=field: self._setEnableWaveCb(msg, f)))
            S(nepi_sdk.create_subscriber(ns + '/set_sin_spread_' + field,
                               Float32, lambda msg, f=field: self._setSinSpreadCb(msg, f)))

    def _setHnavEnabledCb(self, msg):
        with self._lock: self.hnav_sim_enabled = msg.data
        self._applyState()

    def _setLatitudeCb(self, msg):
        with self._lock: self.hnav_latitude = float(msg.data)
        self.publish_status()

    def _setLongitudeCb(self, msg):
        with self._lock: self.hnav_longitude = float(msg.data)
        self.publish_status()

    def _setAltitudeCb(self, msg):
        with self._lock: self.hnav_altitude_m = float(msg.data)
        self.publish_status()

    def _setDepthCb(self, msg):
        with self._lock: self.hnav_depth_m = float(msg.data)
        self.publish_status()

    def _setHeadingCb(self, msg):
        with self._lock: self.hnav_heading_deg = float(msg.data) % 360.0
        self.publish_status()

    def _setRollCb(self, msg):
        with self._lock: self.hnav_roll_deg = float(msg.data)
        self.publish_status()

    def _setPitchCb(self, msg):
        with self._lock: self.hnav_pitch_deg = float(msg.data)
        self.publish_status()

    def _setSpeedCb(self, msg):
        with self._lock: self.hnav_speed_ms = max(0.0, float(msg.data))
        self.publish_status()

    def _setEnableMoveCb(self, msg, field):
        with self._lock: setattr(self, 'enable_move_' + field, msg.data)
        self.publish_status()

    def _setMoveStepCb(self, msg, field):
        with self._lock: setattr(self, 'move_step_' + field, float(msg.data))
        self.publish_status()

    def _setMoveRateHzCb(self, msg, field):
        with self._lock: setattr(self, 'move_rate_hz_' + field, float(msg.data))
        self.publish_status()

    def _setEnableSinCb(self, msg, field):
        with self._lock:
            setattr(self, 'enable_sin_' + field, msg.data)
            if msg.data:
                self._sin_start_time[field] = time.time()
                self._sin_base[field] = getattr(self, field)
                if getattr(self, 'enable_wave_' + field):
                    self._wave_components[field] = self._generateWaveComponents(field)
            else:
                self._sin_start_time[field] = None
        self.publish_status()

    def _setSinAmplitudeCb(self, msg, field):
        with self._lock:
            setattr(self, 'sin_amplitude_' + field, float(msg.data))
            if getattr(self, 'enable_wave_' + field):
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _setSinPeriodCb(self, msg, field):
        with self._lock:
            setattr(self, 'sin_period_s_' + field, float(msg.data))
            if getattr(self, 'enable_wave_' + field):
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _setEnableWaveCb(self, msg, field):
        with self._lock:
            setattr(self, 'enable_wave_' + field, msg.data)
            if msg.data:
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _setSinSpreadCb(self, msg, field):
        with self._lock:
            setattr(self, 'sin_spread_' + field, float(msg.data))
            if getattr(self, 'enable_wave_' + field):
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _generateWaveComponents(self, field):
        period    = getattr(self, 'sin_period_s_' + field)
        amplitude = getattr(self, 'sin_amplitude_' + field)
        spread    = float(getattr(self, 'sin_spread_' + field))
        f0 = 1.0 / period if period > 0.0 else 1.0
        if spread <= 0.0:
            return [(f0, amplitude, 0.0)]
        N  = 7
        bw = spread * f0
        freqs   = [f0 + bw * (i / (N - 1) - 0.5) for i in range(N)]
        sigma   = bw / 3.0
        weights = [math.exp(-0.5 * ((f - f0) / sigma) ** 2) for f in freqs]
        total   = sum(weights) or 1.0
        return [(f, amplitude * w / total, random.uniform(0.0, 2.0 * math.pi))
                for f, w in zip(freqs, weights)]

    def _moveThreadLoop(self):
        while self._alive:
            now     = time.time()
            changed = False
            for field in _HNAV_MOVE_FIELDS:
                if not getattr(self, 'enable_move_' + field):
                    continue
                if field in _SIN_FIELDS and getattr(self, 'enable_sin_' + field):
                    start = self._sin_start_time.get(field)
                    if start is not None:
                        elapsed = now - start
                        if getattr(self, 'enable_wave_' + field):
                            components = self._wave_components.get(field)
                            if components:
                                new_val = self._sin_base[field] + sum(
                                    a * math.sin(2.0 * math.pi * f * elapsed + ph)
                                    for f, a, ph in components
                                )
                                with self._lock: setattr(self, field, new_val)
                                changed = True
                        else:
                            period = getattr(self, 'sin_period_s_' + field)
                            if period > 0.0:
                                new_val = (self._sin_base[field] +
                                           getattr(self, 'sin_amplitude_' + field) *
                                           math.sin(2.0 * math.pi * elapsed / period))
                                with self._lock: setattr(self, field, new_val)
                                changed = True
                    continue
                rate = getattr(self, 'move_rate_hz_' + field)
                if rate <= 0.0:
                    continue
                if (now - self._move_last_tick[field]) >= 1.0 / rate:
                    with self._lock:
                        setattr(self, field,
                                getattr(self, field) + getattr(self, 'move_step_' + field))
                    self._move_last_tick[field] = now
                    changed = True
            if changed:
                self.publish_status()
            time.sleep(1.0 / 20)

    def _applyState(self):
        with self._lock:
            enabled = self.hnav_sim_enabled
            port    = self.hnav_port
        if enabled and self._stop_evt is None:
            self._navpose_if = NavPoseIF(
                namespace=self._ns + '/navpose',
                data_source_description=f'nav_sim_{self.name}',
                data_ref_description='WGS84',
                pub_navpose=True, pub_location=True,
                pub_heading=True, pub_orientation=True,
                pub_altitude=True, pub_depth=True,
                msg_if=self._msg_if,
            )
            stop_evt = threading.Event()
            self._stop_evt = stop_evt
            threading.Thread(target=self._serverLoop,
                             args=('127.0.0.1', port, stop_evt), daemon=True).start()
        elif not enabled and self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
            with self._lock: self.hnav_connected = False
            self._navpose_if = None
        self.publish_status()

    def _serverLoop(self, host, port, stop_evt):
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
                with self._lock: self.hnav_connected = True
                self.publish_status()
                self._serveClient(conn, stop_evt)
                with self._lock: self.hnav_connected = False
                self.publish_status()
        except Exception as e:
            self._msg_if.pub_warn(f"HNav server error on {self.name}: {e}")
        finally:
            try: srv.close()
            except Exception: pass

    def _serveClient(self, conn, stop_evt):
        rate_hz = 10
        period  = 1.0 / rate_hz
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat, lon   = self.hnav_latitude, self.hnav_longitude
                    alt, depth = self.hnav_altitude_m, self.hnav_depth_m
                    heading    = self.hnav_heading_deg
                    roll, pitch, speed = self.hnav_roll_deg, self.hnav_pitch_deg, self.hnav_speed_ms
                packet = _build_hnav_packet(
                    lat_deg=lat, lon_deg=lon, depth_m=depth, alt_m=alt,
                    roll_deg=roll, pitch_deg=pitch, heading_deg=heading, vel_fwd_ms=speed,
                )
                conn.sendall(packet)
                if speed > 0:
                    self._deadReckon(period)
                time.sleep(period)
        except Exception:
            pass
        finally:
            try: conn.close()
            except Exception: pass

    def _deadReckon(self, dt_sec):
        with self._lock:
            heading_rad = math.radians(self.hnav_heading_deg)
            speed, lat  = self.hnav_speed_ms, self.hnav_latitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) * math.sin(heading_rad)
        with self._lock:
            self.hnav_latitude  += dlat
            self.hnav_longitude += dlon

    def publishNavpose(self):
        if self._navpose_if is None:
            return
        with self._lock:
            lat, lon  = self.hnav_latitude, self.hnav_longitude
            alt, depth = self.hnav_altitude_m, self.hnav_depth_m
            heading, roll, pitch = self.hnav_heading_deg, self.hnav_roll_deg, self.hnav_pitch_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']    = True; d['latitude']    = lat;    d['longitude'] = lon;   d['time_location']    = t
        d['has_heading']     = True; d['heading_deg'] = heading;                        d['time_heading']     = t
        d['has_orientation'] = True; d['roll_deg']    = roll;   d['pitch_deg'] = pitch; d['yaw_deg'] = heading; d['time_orientation'] = t
        d['has_altitude']    = True; d['altitude_m']  = alt;                            d['time_altitude']    = t
        d['has_depth']       = True; d['depth_m']     = depth;                          d['time_depth']       = t
        self._navpose_if.publish_navpose(d)

    def publish_status(self):
        self._status_pub.publish(self._buildStatusMsg())

    def _buildStatusMsg(self):
        msg = NepiAppHNavSimStatus()
        with self._lock:
            msg.hnav_sim_enabled = self.hnav_sim_enabled
            msg.hnav_connected   = self.hnav_connected
            msg.hnav_port        = self.hnav_port
            msg.hnav_latitude    = self.hnav_latitude
            msg.hnav_longitude   = self.hnav_longitude
            msg.hnav_altitude_m  = self.hnav_altitude_m
            msg.hnav_depth_m     = self.hnav_depth_m
            msg.hnav_heading_deg = self.hnav_heading_deg
            msg.hnav_roll_deg    = self.hnav_roll_deg
            msg.hnav_pitch_deg   = self.hnav_pitch_deg
            msg.hnav_speed_ms    = self.hnav_speed_ms
            for f in _HNAV_MOVE_FIELDS:
                setattr(msg, 'enable_move_' + f,  getattr(self, 'enable_move_' + f))
                setattr(msg, 'move_step_' + f,     getattr(self, 'move_step_' + f))
                setattr(msg, 'move_rate_hz_' + f,  getattr(self, 'move_rate_hz_' + f))
            for f in _SIN_FIELDS:
                setattr(msg, 'enable_sin_' + f,    getattr(self, 'enable_sin_' + f))
                setattr(msg, 'sin_amplitude_' + f, getattr(self, 'sin_amplitude_' + f))
                setattr(msg, 'sin_period_s_' + f,  getattr(self, 'sin_period_s_' + f))
                setattr(msg, 'enable_wave_' + f,   getattr(self, 'enable_wave_' + f))
                setattr(msg, 'sin_spread_' + f,    getattr(self, 'sin_spread_' + f))
        return msg

    def to_dict(self):
        with self._lock:
            d = {
                'hnav_sim_enabled': self.hnav_sim_enabled,
                'hnav_latitude':    self.hnav_latitude,
                'hnav_longitude':   self.hnav_longitude,
                'hnav_altitude_m':  self.hnav_altitude_m,
                'hnav_depth_m':     self.hnav_depth_m,
                'hnav_heading_deg': self.hnav_heading_deg,
                'hnav_roll_deg':    self.hnav_roll_deg,
                'hnav_pitch_deg':   self.hnav_pitch_deg,
                'hnav_speed_ms':    self.hnav_speed_ms,
                'hnav_port':        self.hnav_port,
            }
            for f in _HNAV_MOVE_FIELDS:
                d['enable_move_'  + f] = getattr(self, 'enable_move_'  + f)
                d['move_step_'    + f] = getattr(self, 'move_step_'    + f)
                d['move_rate_hz_' + f] = getattr(self, 'move_rate_hz_' + f)
            for f in _SIN_FIELDS:
                d['enable_sin_'    + f] = getattr(self, 'enable_sin_'    + f)
                d['sin_amplitude_' + f] = getattr(self, 'sin_amplitude_' + f)
                d['sin_period_s_'  + f] = getattr(self, 'sin_period_s_'  + f)
                d['enable_wave_'   + f] = getattr(self, 'enable_wave_'   + f)
                d['sin_spread_'    + f] = getattr(self, 'sin_spread_'    + f)
        return d

    def apply_dict(self, d):
        need_apply = False
        with self._lock:
            for key in ('hnav_latitude', 'hnav_longitude', 'hnav_altitude_m', 'hnav_depth_m',
                        'hnav_heading_deg', 'hnav_roll_deg', 'hnav_pitch_deg', 'hnav_speed_ms'):
                if key in d:
                    setattr(self, key, float(d[key]))
            if 'hnav_port' in d:
                self.hnav_port = int(d['hnav_port'])
            for f in _HNAV_MOVE_FIELDS:
                k = 'enable_move_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'move_step_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'move_rate_hz_' + f
                if k in d: setattr(self, k, float(d[k]))
            for f in _SIN_FIELDS:
                k = 'enable_sin_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'sin_amplitude_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'sin_period_s_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'enable_wave_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'sin_spread_' + f
                if k in d: setattr(self, k, float(d[k]))
            if 'hnav_sim_enabled' in d:
                new_enabled = bool(d['hnav_sim_enabled'])
                if new_enabled != self.hnav_sim_enabled:
                    self.hnav_sim_enabled = new_enabled
                    need_apply = True
        if need_apply:
            self._applyState()
        else:
            self.publish_status()

    def cleanup(self):
        self._alive = False
        if self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
        for sub in self._subs:
            sub.unregister()
        if self._navpose_if is not None:
            self._navpose_if.unregister_pubs()
            self._navpose_if = None
        self._status_pub.unregister()


#########################################
# Main coordinator node

class NepiNavSimApp:

    DEFAULT_NODE_NAME = "app_nav_sim"
    _CFG_FILE = '/mnt/nepi_storage/user_cfg/app_nav_sim_instances.yaml'

    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name     = type(self).__name__
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Nav Sim App starting")

        self._inst_lock      = threading.Lock()
        self._nmea_instances = {}
        self._hnav_instances = {}

        ns = self.node_namespace

        self._master_pub = nepi_sdk.create_publisher(
            ns + '/status', NepiAppNavSimMasterStatus, queue_size=1, latch=True
        )

        nepi_sdk.create_subscriber(ns + '/add_nmea_instance',    String,       self._addNmeaInstanceCb)
        nepi_sdk.create_subscriber(ns + '/remove_nmea_instance', String,       self._removeNmeaInstanceCb)
        nepi_sdk.create_subscriber(ns + '/rename_nmea_instance', UpdateString, self._renameNmeaInstanceCb)
        nepi_sdk.create_subscriber(ns + '/add_hnav_instance',    String,       self._addHnavInstanceCb)
        nepi_sdk.create_subscriber(ns + '/remove_hnav_instance', String,       self._removeHnavInstanceCb)
        nepi_sdk.create_subscriber(ns + '/rename_hnav_instance', UpdateString, self._renameHnavInstanceCb)

        nepi_sdk.create_subscriber(ns + '/save_config',          Empty, self._saveConfigCb)
        nepi_sdk.create_subscriber(ns + '/reset_config',         Empty, self._resetConfigCb)
        nepi_sdk.create_subscriber(ns + '/factory_reset_config', Empty, self._factoryResetConfigCb)

        saved_state = self._readConfigFile()
        nmea_cfg = saved_state.get('nmea', {}) if saved_state else {}
        hnav_cfg = saved_state.get('hnav', {}) if saved_state else {}
        if nmea_cfg:
            for name, inst_state in nmea_cfg.items():
                port = int(inst_state.get('nmea_port', FACTORY_NMEA_PORT)) if inst_state else FACTORY_NMEA_PORT
                self._createNmeaInstance(name, port)
        else:
            self._createNmeaInstance('nmea_0', FACTORY_NMEA_PORT)
        if hnav_cfg:
            for name, inst_state in hnav_cfg.items():
                port = int(inst_state.get('hnav_port', FACTORY_HNAV_PORT)) if inst_state else FACTORY_HNAV_PORT
                self._createHnavInstance(name, port)
        else:
            self._createHnavInstance('hnav_0', FACTORY_HNAV_PORT)
        if saved_state:
            self._applyConfigState(saved_state)
        self._publishMasterStatus()

        nepi_sdk.start_timer_process(1.0 / STATUS_RATE_HZ, self._timerCb)
        self.msg_if.pub_info("Nav Sim App ready")
        nepi_sdk.on_shutdown(self._cleanupAll)
        nepi_sdk.spin()

    def _timerCb(self, timer):
        self._publishMasterStatus()
        with self._inst_lock:
            nmea_insts = list(self._nmea_instances.values())
            hnav_insts = list(self._hnav_instances.values())
        for inst in nmea_insts + hnav_insts:
            inst.publish_status()
            inst.publishNavpose()

    def _createNmeaInstance(self, name, port):
        inst = NmeaSimInstance(name, self.node_namespace, self.msg_if)
        inst.nmea_port = port
        with self._inst_lock:
            self._nmea_instances[name] = inst
        self.msg_if.pub_info(f"NMEA sim instance '{name}' created (port:{port})")

    def _createHnavInstance(self, name, port):
        inst = HNavSimInstance(name, self.node_namespace, self.msg_if)
        inst.hnav_port = port
        with self._inst_lock:
            self._hnav_instances[name] = inst
        self.msg_if.pub_info(f"HNav sim instance '{name}' created (port:{port})")

    def _addNmeaInstanceCb(self, msg):
        name = msg.data.strip() or self._autoNmeaName()
        with self._inst_lock:
            if name in self._nmea_instances:
                return
        port = self._nextNmeaPort()
        self._createNmeaInstance(name, port)
        self._publishMasterStatus()

    def _removeNmeaInstanceCb(self, msg):
        name = msg.data.strip()
        with self._inst_lock:
            if name not in self._nmea_instances or len(self._nmea_instances) <= 1:
                return
            inst = self._nmea_instances.pop(name)
        inst.cleanup()
        self._publishMasterStatus()

    def _addHnavInstanceCb(self, msg):
        name = msg.data.strip() or self._autoHnavName()
        with self._inst_lock:
            if name in self._hnav_instances:
                return
        port = self._nextHnavPort()
        self._createHnavInstance(name, port)
        self._publishMasterStatus()

    def _removeHnavInstanceCb(self, msg):
        name = msg.data.strip()
        with self._inst_lock:
            if name not in self._hnav_instances or len(self._hnav_instances) <= 1:
                return
            inst = self._hnav_instances.pop(name)
        inst.cleanup()
        self._publishMasterStatus()

    def _renameNmeaInstanceCb(self, msg):
        old_name = msg.name.strip()
        new_name = msg.value.strip()
        if not new_name or old_name == new_name:
            return
        with self._inst_lock:
            if old_name not in self._nmea_instances or new_name in self._nmea_instances:
                return
            inst = self._nmea_instances.pop(old_name)
            port = inst.nmea_port
        old_state = inst.to_dict()
        inst.cleanup()
        self._createNmeaInstance(new_name, port)
        with self._inst_lock:
            new_inst = self._nmea_instances[new_name]
        new_inst.apply_dict(old_state)
        self._publishMasterStatus()

    def _renameHnavInstanceCb(self, msg):
        old_name = msg.name.strip()
        new_name = msg.value.strip()
        if not new_name or old_name == new_name:
            return
        with self._inst_lock:
            if old_name not in self._hnav_instances or new_name in self._hnav_instances:
                return
            inst = self._hnav_instances.pop(old_name)
            port = inst.hnav_port
        old_state = inst.to_dict()
        inst.cleanup()
        self._createHnavInstance(new_name, port)
        with self._inst_lock:
            new_inst = self._hnav_instances[new_name]
        new_inst.apply_dict(old_state)
        self._publishMasterStatus()

    def _publishMasterStatus(self):
        msg = NepiAppNavSimMasterStatus()
        with self._inst_lock:
            msg.nmea_instance_names = list(self._nmea_instances.keys())
            msg.hnav_instance_names = list(self._hnav_instances.keys())
        self._master_pub.publish(msg)

    def _autoNmeaName(self):
        with self._inst_lock:
            i = 0
            while f'nmea_{i}' in self._nmea_instances:
                i += 1
        return f'nmea_{i}'

    def _autoHnavName(self):
        with self._inst_lock:
            i = 0
            while f'hnav_{i}' in self._hnav_instances:
                i += 1
        return f'hnav_{i}'

    def _nextNmeaPort(self):
        with self._inst_lock:
            used = {inst.nmea_port for inst in self._nmea_instances.values()}
        p = FACTORY_NMEA_PORT
        while p in used:
            p += 1
        return p

    def _nextHnavPort(self):
        with self._inst_lock:
            used = {inst.hnav_port for inst in self._hnav_instances.values()}
        p = FACTORY_HNAV_PORT
        while p in used:
            p += 1
        return p

    def _readConfigFile(self):
        if not os.path.isfile(self._CFG_FILE):
            return None
        try:
            with open(self._CFG_FILE, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.msg_if.pub_warn(f"Nav Sim: failed to read config: {e}")
            return None

    def _applyConfigState(self, state):
        with self._inst_lock:
            nmea_pairs = list(self._nmea_instances.items())
            hnav_pairs = list(self._hnav_instances.items())
        for name, inst in nmea_pairs:
            if name in state.get('nmea', {}):
                inst.apply_dict(state['nmea'][name])
        for name, inst in hnav_pairs:
            if name in state.get('hnav', {}):
                inst.apply_dict(state['hnav'][name])

    def _loadConfig(self):
        state = self._readConfigFile()
        if state:
            self._applyConfigState(state)
            self.msg_if.pub_info("Nav Sim: config loaded")

    def _saveConfig(self):
        state = {'nmea': {}, 'hnav': {}}
        with self._inst_lock:
            for name, inst in self._nmea_instances.items():
                state['nmea'][name] = inst.to_dict()
            for name, inst in self._hnav_instances.items():
                state['hnav'][name] = inst.to_dict()
        try:
            os.makedirs(os.path.dirname(self._CFG_FILE), exist_ok=True)
            with open(self._CFG_FILE, 'w') as f:
                yaml.safe_dump(state, f, default_flow_style=False)
            self.msg_if.pub_info("Nav Sim: config saved")
        except Exception as e:
            self.msg_if.pub_warn(f"Nav Sim: failed to save config: {e}")

    def _saveConfigCb(self, msg):
        self._saveConfig()

    def _resetConfigCb(self, msg):
        self._loadConfig()

    def _factoryResetConfigCb(self, msg):
        try:
            os.remove(self._CFG_FILE)
        except OSError:
            pass
        with self._inst_lock:
            all_nmea = list(self._nmea_instances.values())
            all_hnav = list(self._hnav_instances.values())
            self._nmea_instances.clear()
            self._hnav_instances.clear()
        for inst in all_nmea + all_hnav:
            inst.cleanup()
        self._createNmeaInstance('nmea_0', FACTORY_NMEA_PORT)
        self._createHnavInstance('hnav_0', FACTORY_HNAV_PORT)
        self._publishMasterStatus()
        self.msg_if.pub_info("Nav Sim: config reset to factory defaults")

    def _cleanupAll(self):
        with self._inst_lock:
            all_insts = (list(self._nmea_instances.values()) +
                         list(self._hnav_instances.values()))
        for inst in all_insts:
            inst.cleanup()
        self.msg_if.pub_info("Nav Sim App shutdown")


#########################################
if __name__ == '__main__':
    NepiNavSimApp()
