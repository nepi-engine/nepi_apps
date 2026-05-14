/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
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

      statusListener: null,
      connected: false,
    }

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
    this.setState({
      nmea_sim_enabled: message.nmea_sim_enabled,
      nmea_connected:   message.nmea_connected,
      nmea_port:        message.nmea_port,
      nmea_latitude:    message.nmea_latitude,
      nmea_longitude:   message.nmea_longitude,
      nmea_altitude_m:  message.nmea_altitude_m,
      nmea_heading_deg: message.nmea_heading_deg,
      nmea_speed_ms:    message.nmea_speed_ms,
      hnav_sim_enabled: message.hnav_sim_enabled,
      hnav_connected:   message.hnav_connected,
      hnav_port:        message.hnav_port,
      hnav_latitude:    message.hnav_latitude,
      hnav_longitude:   message.hnav_longitude,
      hnav_altitude_m:  message.hnav_altitude_m,
      hnav_depth_m:     message.hnav_depth_m,
      hnav_heading_deg: message.hnav_heading_deg,
      hnav_roll_deg:    message.hnav_roll_deg,
      hnav_pitch_deg:   message.hnav_pitch_deg,
      hnav_speed_ms:    message.hnav_speed_ms,
      connected: true,
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

  componentDidUpdate(prevProps, prevState) {
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

  renderInputRow(label, bufKey, topicSuffix, currentDisplay, disabled) {
    return (
      <Columns key={bufKey}>
        <Column>
          <Label title={label} />
          <span style={{ fontSize: 11, color: '#aaa' }}>{currentDisplay}</span>
        </Column>
        <Column>
          <Input
            value={this.state[bufKey]}
            onChange={(e) => this.setState({ [bufKey]: e.target.value })}
            disabled={disabled}
            style={{ width: 100 }}
          />
        </Column>
        <Column>
          <Button
            style={{}}
            onClick={() => this.onPublishFloat(topicSuffix, this.state[bufKey])}
            disabled={disabled}
          >
            Set
          </Button>
        </Column>
      </Columns>
    )
  }

  renderNmeaSide() {
    const { connected, nmea_sim_enabled, nmea_connected, nmea_port,
            nmea_latitude, nmea_longitude, nmea_altitude_m,
            nmea_heading_deg, nmea_speed_ms } = this.state
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
        {this.renderInputRow("Latitude (°)",  "nmeaLatInput", "set_nmea_latitude",  nmea_latitude.toFixed(6),   dis)}
        {this.renderInputRow("Longitude (°)", "nmeaLonInput", "set_nmea_longitude", nmea_longitude.toFixed(6),  dis)}
        {this.renderInputRow("Altitude (m)",  "nmeaAltInput", "set_nmea_altitude",  nmea_altitude_m.toFixed(1), dis)}

        {divider}
        <Label title={"Orientation"} />
        {this.renderInputRow("Heading (°)", "nmeaHeadingInput", "set_nmea_heading", nmea_heading_deg.toFixed(1), dis)}

        {divider}
        <Label title={"Dead-Reckoning"} />
        {this.renderInputRow("Speed (m/s)", "nmeaSpeedInput", "set_nmea_speed", nmea_speed_ms.toFixed(2), dis)}
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
    const { connected, hnav_sim_enabled, hnav_connected, hnav_port,
            hnav_latitude, hnav_longitude, hnav_altitude_m, hnav_depth_m,
            hnav_heading_deg, hnav_roll_deg, hnav_pitch_deg, hnav_speed_ms } = this.state
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
        {this.renderInputRow("Latitude (°)",  "hnavLatInput",   "set_hnav_latitude",  hnav_latitude.toFixed(6),   dis)}
        {this.renderInputRow("Longitude (°)", "hnavLonInput",   "set_hnav_longitude", hnav_longitude.toFixed(6),  dis)}
        {this.renderInputRow("Altitude (m)",  "hnavAltInput",   "set_hnav_altitude",  hnav_altitude_m.toFixed(1), dis)}
        {this.renderInputRow("Depth (m)",     "hnavDepthInput", "set_hnav_depth",     hnav_depth_m.toFixed(1),    dis)}

        {divider}
        <Label title={"Orientation"} />
        {this.renderInputRow("Heading (°)", "hnavHeadingInput", "set_hnav_heading", hnav_heading_deg.toFixed(1), dis)}
        {this.renderInputRow("Roll (°)",    "hnavRollInput",    "set_hnav_roll",    hnav_roll_deg.toFixed(1),    dis)}
        {this.renderInputRow("Pitch (°)",   "hnavPitchInput",   "set_hnav_pitch",   hnav_pitch_deg.toFixed(1),   dis)}

        {divider}
        <Label title={"Dead-Reckoning"} />
        {this.renderInputRow("Speed (m/s)", "hnavSpeedInput", "set_hnav_speed", hnav_speed_ms.toFixed(2), dis)}
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
