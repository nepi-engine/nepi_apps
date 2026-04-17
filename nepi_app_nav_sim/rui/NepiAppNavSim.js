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

@inject("ros")
@observer

class NepiAppNavSim extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_nav_sim",
      appNamespace: null,

      // Status fields — mirror NepiAppNavSimStatus.msg
      nmea_sim_enabled: false,
      hnav_sim_enabled: false,
      nmea_connected:   false,
      hnav_connected:   false,
      latitude:         47.6205,
      longitude:        -122.3493,
      altitude_m:       10.0,
      depth_m:          0.0,
      heading_deg:      0.0,
      roll_deg:         0.0,
      pitch_deg:        0.0,
      speed_ms:         0.0,
      nmea_port:        50000,
      hnav_port:        16718,

      // Local edit buffers for text inputs
      latInput:     "47.6205",
      lonInput:     "-122.3493",
      altInput:     "10.0",
      depthInput:   "0.0",
      headingInput: "0.0",
      rollInput:    "0.0",
      pitchInput:   "0.0",
      speedInput:   "0.0",

      statusListener: null,
      connected: false,
    }

    this.getAppNamespace       = this.getAppNamespace.bind(this)
    this.statusListener        = this.statusListener.bind(this)
    this.updateStatusListener  = this.updateStatusListener.bind(this)
    this.onToggleNmea          = this.onToggleNmea.bind(this)
    this.onToggleHnav          = this.onToggleHnav.bind(this)
    this.onPublishFloat        = this.onPublishFloat.bind(this)
    this.renderSimToggles      = this.renderSimToggles.bind(this)
    this.renderPosition        = this.renderPosition.bind(this)
    this.renderOrientation     = this.renderOrientation.bind(this)
    this.renderMotion          = this.renderMotion.bind(this)
    this.renderConfig          = this.renderConfig.bind(this)
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
      hnav_sim_enabled: message.hnav_sim_enabled,
      nmea_connected:   message.nmea_connected,
      hnav_connected:   message.hnav_connected,
      latitude:         message.latitude,
      longitude:        message.longitude,
      altitude_m:       message.altitude_m,
      depth_m:          message.depth_m,
      heading_deg:      message.heading_deg,
      roll_deg:         message.roll_deg,
      pitch_deg:        message.pitch_deg,
      speed_ms:         message.speed_ms,
      nmea_port:        message.nmea_port,
      hnav_port:        message.hnav_port,
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
    const topic = this.getAppNamespace() + '/' + topicSuffix
    sendFloatMsg(topic, value)
  }

  renderSimToggles() {
    const { connected, nmea_sim_enabled, hnav_sim_enabled,
            nmea_connected, hnav_connected, nmea_port, hnav_port } = this.state

    const dot = (active) => ({
      display: 'inline-block',
      width: 10, height: 10,
      borderRadius: '50%',
      backgroundColor: active ? '#00cc44' : '#555',
      marginLeft: 8,
    })

    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Columns>
          <Column>
            <Label title={"NMEA Sim"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>port {nmea_port}</span>
            <span style={dot(nmea_connected)} title={nmea_connected ? "Client connected" : "No client"} />
          </Column>
          <Column>
            <Toggle checked={nmea_sim_enabled} onClick={this.onToggleNmea} disabled={!connected} />
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"HNav Sim"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>port {hnav_port}</span>
            <span style={dot(hnav_connected)} title={hnav_connected ? "Client connected" : "No client"} />
          </Column>
          <Column>
            <Toggle checked={hnav_sim_enabled} onClick={this.onToggleHnav} disabled={!connected} />
          </Column>
        </Columns>
      </React.Fragment>
    )
  }

  renderPosition() {
    const { connected, latitude, longitude, altitude_m, depth_m } = this.state
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Position"} />

        {[
          { label: "Latitude (°)",    key: "latInput",   topic: "set_latitude",  current: latitude.toFixed(6)  },
          { label: "Longitude (°)",   key: "lonInput",   topic: "set_longitude", current: longitude.toFixed(6) },
          { label: "Altitude (m)",    key: "altInput",   topic: "set_altitude",  current: altitude_m.toFixed(1) },
          { label: "Depth (m)",       key: "depthInput", topic: "set_depth",     current: depth_m.toFixed(1)   },
        ].map(({ label, key, topic, current }) => (
          <Columns key={key}>
            <Column>
              <Label title={label} />
              <span style={{ fontSize: 11, color: '#aaa' }}>{current}</span>
            </Column>
            <Column>
              <Input
                value={this.state[key]}
                onChange={(e) => this.setState({ [key]: e.target.value })}
                disabled={!connected}
                style={{ width: 110 }}
              />
            </Column>
            <Column>
              <Button
                style={{}}
                onClick={() => this.onPublishFloat(topic, this.state[key])}
                disabled={!connected}
              >
                Set
              </Button>
            </Column>
          </Columns>
        ))}
      </React.Fragment>
    )
  }

  renderOrientation() {
    const { connected, heading_deg, roll_deg, pitch_deg } = this.state
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Orientation"} />

        {[
          { label: "Heading (°)",  key: "headingInput", topic: "set_heading", current: heading_deg.toFixed(1) },
          { label: "Roll (°)",     key: "rollInput",    topic: "set_roll",    current: roll_deg.toFixed(1)    },
          { label: "Pitch (°)",    key: "pitchInput",   topic: "set_pitch",   current: pitch_deg.toFixed(1)   },
        ].map(({ label, key, topic, current }) => (
          <Columns key={key}>
            <Column>
              <Label title={label} />
              <span style={{ fontSize: 11, color: '#aaa' }}>{current}</span>
            </Column>
            <Column>
              <Input
                value={this.state[key]}
                onChange={(e) => this.setState({ [key]: e.target.value })}
                disabled={!connected}
                style={{ width: 110 }}
              />
            </Column>
            <Column>
              <Button
                style={{}}
                onClick={() => this.onPublishFloat(topic, this.state[key])}
                disabled={!connected}
              >
                Set
              </Button>
            </Column>
          </Columns>
        ))}
      </React.Fragment>
    )
  }

  renderMotion() {
    const { connected, speed_ms } = this.state
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Dead-Reckoning"} />

        <Columns>
          <Column>
            <Label title={"Speed (m/s)"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>{speed_ms.toFixed(2)}</span>
          </Column>
          <Column>
            <Input
              value={this.state.speedInput}
              onChange={(e) => this.setState({ speedInput: e.target.value })}
              disabled={!connected}
              style={{ width: 110 }}
            />
          </Column>
          <Column>
            <Button
              style={{}}
              onClick={() => this.onPublishFloat('set_speed', this.state.speedInput)}
              disabled={!connected}
            >
              Set
            </Button>
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Button
              style={{}}
              onClick={() => { this.setState({ speedInput: "0.0" }); this.onPublishFloat('set_speed', 0.0) }}
              disabled={!connected}
            >
              Stop
            </Button>
          </Column>
        </Columns>
      </React.Fragment>
    )
  }

  renderConfig() {
    const appNamespace = this.getAppNamespace()
    return (
      <React.Fragment>
        <NepiIFConfig namespace={appNamespace} title={"Nepi_IF_Config"} />
      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    const content = (
      <React.Fragment>
        {this.renderSimToggles()}
        {this.renderPosition()}
        {this.renderOrientation()}
        {this.renderMotion()}
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
