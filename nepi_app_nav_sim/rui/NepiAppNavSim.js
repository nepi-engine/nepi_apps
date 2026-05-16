/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
 */

import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Button from "./Button"
import Toggle from "react-toggle"
import Styles from "./Styles"

import NepiIFConfig from "./Nepi_IF_Config"
import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

@inject("ros")
@observer

class NepiAppNavSim extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_nav_sim",
      appNamespace: null,

      nmea_sim_enabled: false,
      nmea_connected:   false,
      nmea_port:        50000,
      nmea_latitude:    47.6205,
      nmea_longitude:   -122.3493,
      nmea_altitude_m:  10.0,
      nmea_heading_deg: 0.0,
      nmea_speed_ms:    0.0,

      hnav_sim_enabled: false,
      hnav_connected:   false,
      hnav_port:        16718,
      hnav_latitude:    47.6205,
      hnav_longitude:   -122.3493,
      hnav_altitude_m:  10.0,
      hnav_depth_m:     0.0,
      hnav_heading_deg: 0.0,
      hnav_roll_deg:    0.0,
      hnav_pitch_deg:   0.0,
      hnav_speed_ms:    0.0,

      // NMEA input buffers
      nmeaLatInput:     "47.6205",
      nmeaLonInput:     "-122.3493",
      nmeaAltInput:     "10.0",
      nmeaHeadingInput: "0.0",
      nmeaSpeedInput:   "0.0",

      // HNav input buffers
      hnavLatInput:     "47.6205",
      hnavLonInput:     "-122.3493",
      hnavAltInput:     "10.0",
      hnavDepthInput:   "0.0",
      hnavHeadingInput: "0.0",
      hnavRollInput:    "0.0",
      hnavPitchInput:   "0.0",
      hnavSpeedInput:   "0.0",

      // Move enable booleans
      enableMoveNmeaLat:     false,
      enableMoveNmeaLon:     false,
      enableMoveNmeaAlt:     false,
      enableMoveNmeaHeading: false,
      enableMoveNmeaSpeed:   false,
      enableMoveHnavLat:     false,
      enableMoveHnavLon:     false,
      enableMoveHnavAlt:     false,
      enableMoveHnavDepth:   false,
      enableMoveHnavHeading: false,
      enableMoveHnavRoll:    false,
      enableMoveHnavPitch:   false,
      enableMoveHnavSpeed:   false,

      // Move step buffers
      nmeaLatStep:     "0.0",
      nmeaLonStep:     "0.0",
      nmeaAltStep:     "0.0",
      nmeaHeadingStep: "0.0",
      nmeaSpeedStep:   "0.0",
      hnavLatStep:     "0.0",
      hnavLonStep:     "0.0",
      hnavAltStep:     "0.0",
      hnavDepthStep:   "0.0",
      hnavHeadingStep: "0.0",
      hnavRollStep:    "0.0",
      hnavPitchStep:   "0.0",
      hnavSpeedStep:   "0.0",

      // Move rate buffers
      nmeaLatRateHz:     "1.0",
      nmeaLonRateHz:     "1.0",
      nmeaAltRateHz:     "1.0",
      nmeaHeadingRateHz: "1.0",
      nmeaSpeedRateHz:   "1.0",
      hnavLatRateHz:     "1.0",
      hnavLonRateHz:     "1.0",
      hnavAltRateHz:     "1.0",
      hnavDepthRateHz:   "1.0",
      hnavHeadingRateHz: "1.0",
      hnavRollRateHz:    "1.0",
      hnavPitchRateHz:   "1.0",
      hnavSpeedRateHz:   "1.0",

      statusListener: null,
      connected: false,
    }

    this.dirtyFields = new Set()

    this.getAppNamespace      = this.getAppNamespace.bind(this)
    this.statusListener       = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.onToggleNmea         = this.onToggleNmea.bind(this)
    this.onToggleHnav         = this.onToggleHnav.bind(this)
    this.onPublishFloat       = this.onPublishFloat.bind(this)
    this.renderInputRow       = this.renderInputRow.bind(this)
    this.renderNmeaSide       = this.renderNmeaSide.bind(this)
    this.renderHnavSide       = this.renderHnavSide.bind(this)
    this.renderConfig         = this.renderConfig.bind(this)
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return null
  }

  statusListener(message) {
    const s = {}

    // Always sync enable-move flags from backend
    s.enableMoveNmeaLat     = message.enable_move_nmea_latitude
    s.enableMoveNmeaLon     = message.enable_move_nmea_longitude
    s.enableMoveNmeaAlt     = message.enable_move_nmea_altitude_m
    s.enableMoveNmeaHeading = message.enable_move_nmea_heading_deg
    s.enableMoveNmeaSpeed   = message.enable_move_nmea_speed_ms
    s.enableMoveHnavLat     = message.enable_move_hnav_latitude
    s.enableMoveHnavLon     = message.enable_move_hnav_longitude
    s.enableMoveHnavAlt     = message.enable_move_hnav_altitude_m
    s.enableMoveHnavDepth   = message.enable_move_hnav_depth_m
    s.enableMoveHnavHeading = message.enable_move_hnav_heading_deg
    s.enableMoveHnavRoll    = message.enable_move_hnav_roll_deg
    s.enableMoveHnavPitch   = message.enable_move_hnav_pitch_deg
    s.enableMoveHnavSpeed   = message.enable_move_hnav_speed_ms

    // Sync step/rate buffers only when move is currently off (don't interrupt active entry)
    if (!this.state.enableMoveNmeaLat) {
      s.nmeaLatStep   = String(message.move_step_nmea_latitude)
      s.nmeaLatRateHz = String(message.move_rate_hz_nmea_latitude)
    }
    if (!this.state.enableMoveNmeaLon) {
      s.nmeaLonStep   = String(message.move_step_nmea_longitude)
      s.nmeaLonRateHz = String(message.move_rate_hz_nmea_longitude)
    }
    if (!this.state.enableMoveNmeaAlt) {
      s.nmeaAltStep   = String(message.move_step_nmea_altitude_m)
      s.nmeaAltRateHz = String(message.move_rate_hz_nmea_altitude_m)
    }
    if (!this.state.enableMoveNmeaHeading) {
      s.nmeaHeadingStep   = String(message.move_step_nmea_heading_deg)
      s.nmeaHeadingRateHz = String(message.move_rate_hz_nmea_heading_deg)
    }
    if (!this.state.enableMoveNmeaSpeed) {
      s.nmeaSpeedStep   = String(message.move_step_nmea_speed_ms)
      s.nmeaSpeedRateHz = String(message.move_rate_hz_nmea_speed_ms)
    }
    if (!this.state.enableMoveHnavLat) {
      s.hnavLatStep   = String(message.move_step_hnav_latitude)
      s.hnavLatRateHz = String(message.move_rate_hz_hnav_latitude)
    }
    if (!this.state.enableMoveHnavLon) {
      s.hnavLonStep   = String(message.move_step_hnav_longitude)
      s.hnavLonRateHz = String(message.move_rate_hz_hnav_longitude)
    }
    if (!this.state.enableMoveHnavAlt) {
      s.hnavAltStep   = String(message.move_step_hnav_altitude_m)
      s.hnavAltRateHz = String(message.move_rate_hz_hnav_altitude_m)
    }
    if (!this.state.enableMoveHnavDepth) {
      s.hnavDepthStep   = String(message.move_step_hnav_depth_m)
      s.hnavDepthRateHz = String(message.move_rate_hz_hnav_depth_m)
    }
    if (!this.state.enableMoveHnavHeading) {
      s.hnavHeadingStep   = String(message.move_step_hnav_heading_deg)
      s.hnavHeadingRateHz = String(message.move_rate_hz_hnav_heading_deg)
    }
    if (!this.state.enableMoveHnavRoll) {
      s.hnavRollStep   = String(message.move_step_hnav_roll_deg)
      s.hnavRollRateHz = String(message.move_rate_hz_hnav_roll_deg)
    }
    if (!this.state.enableMoveHnavPitch) {
      s.hnavPitchStep   = String(message.move_step_hnav_pitch_deg)
      s.hnavPitchRateHz = String(message.move_rate_hz_hnav_pitch_deg)
    }
    if (!this.state.enableMoveHnavSpeed) {
      s.hnavSpeedStep   = String(message.move_step_hnav_speed_ms)
      s.hnavSpeedRateHz = String(message.move_rate_hz_hnav_speed_ms)
    }

    // Sync main input buffers from server; skip if user is actively editing (dirty),
    // but always sync when Move is on (box is read-only, server is driving the value).
    const syncBuf = (key, val, moveEnabled) => {
      if (moveEnabled || !this.dirtyFields.has(key)) s[key] = val
    }
    syncBuf('nmeaLatInput',     message.nmea_latitude.toFixed(6),   s.enableMoveNmeaLat)
    syncBuf('nmeaLonInput',     message.nmea_longitude.toFixed(6),  s.enableMoveNmeaLon)
    syncBuf('nmeaAltInput',     message.nmea_altitude_m.toFixed(1), s.enableMoveNmeaAlt)
    syncBuf('nmeaHeadingInput', message.nmea_heading_deg.toFixed(1),s.enableMoveNmeaHeading)
    syncBuf('nmeaSpeedInput',   message.nmea_speed_ms.toFixed(2),   s.enableMoveNmeaSpeed)
    syncBuf('hnavLatInput',     message.hnav_latitude.toFixed(6),   s.enableMoveHnavLat)
    syncBuf('hnavLonInput',     message.hnav_longitude.toFixed(6),  s.enableMoveHnavLon)
    syncBuf('hnavAltInput',     message.hnav_altitude_m.toFixed(1), s.enableMoveHnavAlt)
    syncBuf('hnavDepthInput',   message.hnav_depth_m.toFixed(1),    s.enableMoveHnavDepth)
    syncBuf('hnavHeadingInput', message.hnav_heading_deg.toFixed(1),s.enableMoveHnavHeading)
    syncBuf('hnavRollInput',    message.hnav_roll_deg.toFixed(1),   s.enableMoveHnavRoll)
    syncBuf('hnavPitchInput',   message.hnav_pitch_deg.toFixed(1),  s.enableMoveHnavPitch)
    syncBuf('hnavSpeedInput',   message.hnav_speed_ms.toFixed(2),   s.enableMoveHnavSpeed)

    this.setState({
      nmea_sim_enabled: message.nmea_sim_enabled,
      nmea_connected:   message.nmea_connected,
      nmea_port:        message.nmea_port,
      hnav_sim_enabled: message.hnav_sim_enabled,
      hnav_connected:   message.hnav_connected,
      hnav_port:        message.hnav_port,
      connected: true,
      ...s,
    })
  }

  updateStatusListener(namespace) {
    const statusNamespace = namespace + '/status'
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    const statusListener = this.props.ros.setupStatusListener(
      statusNamespace,
      "nepi_app_nav_sim/NepiAppNavSimStatus",
      this.statusListener
    )
    this.setState({ appNamespace: namespace, statusListener })
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.updateStatusListener(namespace)
    }
  }

  componentDidUpdate() {
    const namespace = this.getAppNamespace()
    const updated = this.state.appNamespace !== namespace && namespace !== null
    if (updated && namespace.indexOf('null') === -1) {
      this.updateStatusListener(namespace)
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }

  onToggleNmea() {
    const { sendBoolMsg } = this.props.ros
    sendBoolMsg(this.getAppNamespace() + '/set_nmea_enabled', !this.state.nmea_sim_enabled)
  }

  onToggleHnav() {
    const { sendBoolMsg } = this.props.ros
    sendBoolMsg(this.getAppNamespace() + '/set_hnav_enabled', !this.state.hnav_sim_enabled)
  }

  onPublishFloat(topicSuffix, value) {
    const { sendFloatMsg } = this.props.ros
    sendFloatMsg(this.getAppNamespace() + '/' + topicSuffix, value)
  }

  renderInputRow(label, bufKey, topicSuffix, disabled,
                 moveStateKey, stepBufKey, rateBufKey, moveTopicBase) {
    const { sendFloatMsg, sendBoolMsg } = this.props.ros
    const enableMove = this.state[moveStateKey]
    const ns = this.getAppNamespace()

    return (
      <div key={bufKey}>
        <Columns>
          <Column>
            <Label title={label} />
          </Column>
          <Column>
            <Input
              id={bufKey}
              value={this.state[bufKey]}
              onChange={enableMove ? undefined : (e) => {
                this.dirtyFields.add(bufKey)
                const el = document.getElementById(bufKey)
                setElementStyleModified(el)
                this.setState({ [bufKey]: e.target.value })
              }}
              onKeyDown={enableMove ? undefined : (e) => {
                if (e.key === 'Enter') {
                  this.dirtyFields.delete(bufKey)
                  const el = document.getElementById(bufKey)
                  clearElementStyleModified(el)
                  this.onPublishFloat(topicSuffix, this.state[bufKey])
                }
              }}
              readOnly={enableMove}
              disabled={disabled}
              backgroundOverride={enableMove ? Styles.vars.colors.orange : null}
              style={{ width: 100 }}
            />
          </Column>
          <Column>
            <Toggle
              checked={enableMove}
              onChange={() => {
                const newVal = !enableMove
                sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, newVal)
                this.setState({ [moveStateKey]: newVal })
              }}
              disabled={disabled}
            />
            <span style={{ fontSize: 11, color: '#aaa', marginLeft: 4 }}>Move</span>
          </Column>
        </Columns>
        {enableMove && (
          <div style={{ paddingLeft: 16 }}>
            <Columns>
              <Column>
                <Label title={"Step"} />
              </Column>
              <Column>
                <Input
                  id={stepBufKey}
                  value={this.state[stepBufKey]}
                  onChange={(e) => {
                    const el = document.getElementById(stepBufKey)
                    setElementStyleModified(el)
                    this.setState({ [stepBufKey]: e.target.value })
                  }}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter') {
                      const el = document.getElementById(stepBufKey)
                      clearElementStyleModified(el)
                      sendFloatMsg(ns + '/set_move_step_' + moveTopicBase,
                                   parseFloat(this.state[stepBufKey]))
                    }
                  }}
                  disabled={disabled}
                  style={{ width: 80 }}
                />
              </Column>
            </Columns>
            <Columns>
              <Column>
                <Label title={"Rate Hz"} />
              </Column>
              <Column>
                <Input
                  id={rateBufKey}
                  value={this.state[rateBufKey]}
                  onChange={(e) => {
                    const el = document.getElementById(rateBufKey)
                    setElementStyleModified(el)
                    this.setState({ [rateBufKey]: e.target.value })
                  }}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter') {
                      const el = document.getElementById(rateBufKey)
                      clearElementStyleModified(el)
                      sendFloatMsg(ns + '/set_move_rate_hz_' + moveTopicBase,
                                   parseFloat(this.state[rateBufKey]))
                    }
                  }}
                  disabled={disabled}
                  style={{ width: 80 }}
                />
              </Column>
            </Columns>
          </div>
        )}
      </div>
    )
  }

  renderNmeaSide() {
    const { connected, nmea_sim_enabled, nmea_connected, nmea_port } = this.state
    const dis = !connected

    const dotColor = nmea_connected ? '#00cc44' : (nmea_sim_enabled ? '#ff9900' : '#555')
    const dotTitle = nmea_connected ? "Client connected"
                   : (nmea_sim_enabled ? "Server running, no client" : "Sim disabled")
    const dotStyle = {
      display: 'inline-block', width: 10, height: 10,
      borderRadius: '50%', backgroundColor: dotColor, marginLeft: 8,
    }

    const divider = (
      <div style={{ borderTop: "1px solid #555",
                    marginTop: Styles.vars.spacing.small,
                    marginBottom: Styles.vars.spacing.xs }} />
    )

    return (
      <div>
        <Columns>
          <Column>
            <Label title={"NMEA Sim"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>port {nmea_port}</span>
            <span style={dotStyle} title={dotTitle} />
          </Column>
          <Column>
            <Toggle checked={nmea_sim_enabled} onClick={this.onToggleNmea} disabled={dis} />
          </Column>
        </Columns>

        {divider}
        <Label title={"Position"} />
        {this.renderInputRow("Latitude (°)",  "nmeaLatInput",     "set_nmea_latitude",  dis, "enableMoveNmeaLat",     "nmeaLatStep",     "nmeaLatRateHz",     "nmea_latitude")}
        {this.renderInputRow("Longitude (°)", "nmeaLonInput",     "set_nmea_longitude", dis, "enableMoveNmeaLon",     "nmeaLonStep",     "nmeaLonRateHz",     "nmea_longitude")}
        {this.renderInputRow("Altitude (m)",  "nmeaAltInput",     "set_nmea_altitude",  dis, "enableMoveNmeaAlt",     "nmeaAltStep",     "nmeaAltRateHz",     "nmea_altitude_m")}

        {divider}
        <Label title={"Orientation"} />
        {this.renderInputRow("Heading (°)", "nmeaHeadingInput", "set_nmea_heading", dis, "enableMoveNmeaHeading", "nmeaHeadingStep", "nmeaHeadingRateHz", "nmea_heading_deg")}

        {divider}
        <Label title={"Dead-Reckoning"} />
        {this.renderInputRow("Speed (m/s)", "nmeaSpeedInput", "set_nmea_speed", dis, "enableMoveNmeaSpeed", "nmeaSpeedStep", "nmeaSpeedRateHz", "nmea_speed_ms")}
        <Columns>
          <Column>
            <Button
              style={{}}
              onClick={() => { this.setState({ nmeaSpeedInput: "0.0" }); this.onPublishFloat('set_nmea_speed', 0.0) }}
              disabled={dis}
            >
              Stop
            </Button>
          </Column>
        </Columns>
      </div>
    )
  }

  renderHnavSide() {
    const { connected, hnav_sim_enabled, hnav_connected, hnav_port } = this.state
    const dis = !connected

    const dotColor = hnav_connected ? '#00cc44' : (hnav_sim_enabled ? '#ff9900' : '#555')
    const dotTitle = hnav_connected ? "Client connected"
                   : (hnav_sim_enabled ? "Server running, no client" : "Sim disabled")
    const dotStyle = {
      display: 'inline-block', width: 10, height: 10,
      borderRadius: '50%', backgroundColor: dotColor, marginLeft: 8,
    }

    const divider = (
      <div style={{ borderTop: "1px solid #555",
                    marginTop: Styles.vars.spacing.small,
                    marginBottom: Styles.vars.spacing.xs }} />
    )

    return (
      <div>
        <Columns>
          <Column>
            <Label title={"HNav Sim"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>port {hnav_port}</span>
            <span style={dotStyle} title={dotTitle} />
          </Column>
          <Column>
            <Toggle checked={hnav_sim_enabled} onClick={this.onToggleHnav} disabled={dis} />
          </Column>
        </Columns>

        {divider}
        <Label title={"Position"} />
        {this.renderInputRow("Latitude (°)",  "hnavLatInput",   "set_hnav_latitude",  dis, "enableMoveHnavLat",     "hnavLatStep",     "hnavLatRateHz",     "hnav_latitude")}
        {this.renderInputRow("Longitude (°)", "hnavLonInput",   "set_hnav_longitude", dis, "enableMoveHnavLon",     "hnavLonStep",     "hnavLonRateHz",     "hnav_longitude")}
        {this.renderInputRow("Altitude (m)",  "hnavAltInput",   "set_hnav_altitude",  dis, "enableMoveHnavAlt",     "hnavAltStep",     "hnavAltRateHz",     "hnav_altitude_m")}
        {this.renderInputRow("Depth (m)",     "hnavDepthInput", "set_hnav_depth",     dis, "enableMoveHnavDepth",   "hnavDepthStep",   "hnavDepthRateHz",   "hnav_depth_m")}

        {divider}
        <Label title={"Orientation"} />
        {this.renderInputRow("Heading (°)", "hnavHeadingInput", "set_hnav_heading", dis, "enableMoveHnavHeading", "hnavHeadingStep", "hnavHeadingRateHz", "hnav_heading_deg")}
        {this.renderInputRow("Roll (°)",    "hnavRollInput",    "set_hnav_roll",    dis, "enableMoveHnavRoll",    "hnavRollStep",    "hnavRollRateHz",    "hnav_roll_deg")}
        {this.renderInputRow("Pitch (°)",   "hnavPitchInput",   "set_hnav_pitch",   dis, "enableMoveHnavPitch",   "hnavPitchStep",   "hnavPitchRateHz",   "hnav_pitch_deg")}

        {divider}
        <Label title={"Dead-Reckoning"} />
        {this.renderInputRow("Speed (m/s)", "hnavSpeedInput", "set_hnav_speed", dis, "enableMoveHnavSpeed", "hnavSpeedStep", "hnavSpeedRateHz", "hnav_speed_ms")}
        <Columns>
          <Column>
            <Button
              style={{}}
              onClick={() => { this.setState({ hnavSpeedInput: "0.0" }); this.onPublishFloat('set_hnav_speed', 0.0) }}
              disabled={dis}
            >
              Stop
            </Button>
          </Column>
        </Columns>
      </div>
    )
  }

  renderConfig() {
    return (
      <React.Fragment>
        <NepiIFConfig namespace={this.getAppNamespace()} title={"Nepi_IF_Config"} />
      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    const content = (
      <React.Fragment>
        <div style={{ display: 'flex', gap: 0 }}>
          <div style={{ flex: 1, paddingRight: 16, borderRight: '1px solid #444' }}>
            {this.renderNmeaSide()}
          </div>
          <div style={{ flex: 1, paddingLeft: 16 }}>
            {this.renderHnavSide()}
          </div>
        </div>
        {this.renderConfig()}
      </React.Fragment>
    )

    if (make_section === false) {
      return <Columns><Column>{content}</Column></Columns>
    }
    return <Section>{content}</Section>
  }
}

export default NepiAppNavSim
