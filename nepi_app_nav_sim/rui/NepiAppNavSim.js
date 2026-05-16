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

      // Step sub-toggle state for sin-capable orientation fields
      enableStepHnavHeading: false,
      enableStepHnavRoll:    false,
      enableStepHnavPitch:   false,

      // Sin enable booleans (orientation fields only)
      enableSinHnavHeading: false,
      enableSinHnavRoll:    false,
      enableSinHnavPitch:   false,

      // Sin amplitude buffers
      hnavHeadingSinAmp: "5.0",
      hnavRollSinAmp:    "5.0",
      hnavPitchSinAmp:   "5.0",

      // Sin period buffers (seconds)
      hnavHeadingSinPeriodS: "10.0",
      hnavRollSinPeriodS:    "10.0",
      hnavPitchSinPeriodS:   "10.0",

      // Wave sub-mode enable booleans
      enableWaveHnavHeading: false,
      enableWaveHnavRoll:    false,
      enableWaveHnavPitch:   false,

      // Sin spread buffers (0.0 – 1.0)
      hnavHeadingSinSpread: "0.5",
      hnavRollSinSpread:    "0.5",
      hnavPitchSinSpread:   "0.5",

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
    // Move gate is local for sin-capable rows — preserve it if user has opened it
    s.enableMoveHnavHeading = message.enable_move_hnav_heading_deg || message.enable_sin_hnav_heading_deg || this.state.enableMoveHnavHeading
    s.enableMoveHnavRoll    = message.enable_move_hnav_roll_deg    || message.enable_sin_hnav_roll_deg    || this.state.enableMoveHnavRoll
    s.enableMoveHnavPitch   = message.enable_move_hnav_pitch_deg   || message.enable_sin_hnav_pitch_deg   || this.state.enableMoveHnavPitch
    s.enableMoveHnavSpeed   = message.enable_move_hnav_speed_ms

    // Step is active only when move is on AND sin is not (move also serves as sin's gate)
    s.enableStepHnavHeading = message.enable_move_hnav_heading_deg && !message.enable_sin_hnav_heading_deg
    s.enableStepHnavRoll    = message.enable_move_hnav_roll_deg    && !message.enable_sin_hnav_roll_deg
    s.enableStepHnavPitch   = message.enable_move_hnav_pitch_deg   && !message.enable_sin_hnav_pitch_deg

    // Always sync sin enable flags
    s.enableSinHnavHeading  = message.enable_sin_hnav_heading_deg
    s.enableSinHnavRoll     = message.enable_sin_hnav_roll_deg
    s.enableSinHnavPitch    = message.enable_sin_hnav_pitch_deg
    // Wave gate: preserve local state until backend confirms (same race as Move gate)
    s.enableWaveHnavHeading = message.enable_wave_hnav_heading_deg || this.state.enableWaveHnavHeading
    s.enableWaveHnavRoll    = message.enable_wave_hnav_roll_deg    || this.state.enableWaveHnavRoll
    s.enableWaveHnavPitch   = message.enable_wave_hnav_pitch_deg   || this.state.enableWaveHnavPitch

    // Sync secondary parameter buffers — skip any box the user is actively editing
    const d = this.dirtyFields
    if (!d.has('nmeaLatStep'))           s.nmeaLatStep           = String(message.move_step_nmea_latitude)
    if (!d.has('nmeaLatRateHz'))         s.nmeaLatRateHz         = String(message.move_rate_hz_nmea_latitude)
    if (!d.has('nmeaLonStep'))           s.nmeaLonStep           = String(message.move_step_nmea_longitude)
    if (!d.has('nmeaLonRateHz'))         s.nmeaLonRateHz         = String(message.move_rate_hz_nmea_longitude)
    if (!d.has('nmeaAltStep'))           s.nmeaAltStep           = String(message.move_step_nmea_altitude_m)
    if (!d.has('nmeaAltRateHz'))         s.nmeaAltRateHz         = String(message.move_rate_hz_nmea_altitude_m)
    if (!d.has('nmeaHeadingStep'))       s.nmeaHeadingStep       = String(message.move_step_nmea_heading_deg)
    if (!d.has('nmeaHeadingRateHz'))     s.nmeaHeadingRateHz     = String(message.move_rate_hz_nmea_heading_deg)
    if (!d.has('nmeaSpeedStep'))         s.nmeaSpeedStep         = String(message.move_step_nmea_speed_ms)
    if (!d.has('nmeaSpeedRateHz'))       s.nmeaSpeedRateHz       = String(message.move_rate_hz_nmea_speed_ms)
    if (!d.has('hnavLatStep'))           s.hnavLatStep           = String(message.move_step_hnav_latitude)
    if (!d.has('hnavLatRateHz'))         s.hnavLatRateHz         = String(message.move_rate_hz_hnav_latitude)
    if (!d.has('hnavLonStep'))           s.hnavLonStep           = String(message.move_step_hnav_longitude)
    if (!d.has('hnavLonRateHz'))         s.hnavLonRateHz         = String(message.move_rate_hz_hnav_longitude)
    if (!d.has('hnavAltStep'))           s.hnavAltStep           = String(message.move_step_hnav_altitude_m)
    if (!d.has('hnavAltRateHz'))         s.hnavAltRateHz         = String(message.move_rate_hz_hnav_altitude_m)
    if (!d.has('hnavDepthStep'))         s.hnavDepthStep         = String(message.move_step_hnav_depth_m)
    if (!d.has('hnavDepthRateHz'))       s.hnavDepthRateHz       = String(message.move_rate_hz_hnav_depth_m)
    if (!d.has('hnavHeadingStep'))       s.hnavHeadingStep       = String(message.move_step_hnav_heading_deg)
    if (!d.has('hnavHeadingRateHz'))     s.hnavHeadingRateHz     = String(message.move_rate_hz_hnav_heading_deg)
    if (!d.has('hnavRollStep'))          s.hnavRollStep          = String(message.move_step_hnav_roll_deg)
    if (!d.has('hnavRollRateHz'))        s.hnavRollRateHz        = String(message.move_rate_hz_hnav_roll_deg)
    if (!d.has('hnavPitchStep'))         s.hnavPitchStep         = String(message.move_step_hnav_pitch_deg)
    if (!d.has('hnavPitchRateHz'))       s.hnavPitchRateHz       = String(message.move_rate_hz_hnav_pitch_deg)
    if (!d.has('hnavSpeedStep'))         s.hnavSpeedStep         = String(message.move_step_hnav_speed_ms)
    if (!d.has('hnavSpeedRateHz'))       s.hnavSpeedRateHz       = String(message.move_rate_hz_hnav_speed_ms)
    if (!d.has('hnavHeadingSinAmp'))      s.hnavHeadingSinAmp      = String(message.sin_amplitude_hnav_heading_deg)
    if (!d.has('hnavHeadingSinPeriodS'))  s.hnavHeadingSinPeriodS  = String(message.sin_period_s_hnav_heading_deg)
    if (!d.has('hnavHeadingSinSpread'))   s.hnavHeadingSinSpread   = String(message.sin_spread_hnav_heading_deg)
    if (!d.has('hnavRollSinAmp'))         s.hnavRollSinAmp         = String(message.sin_amplitude_hnav_roll_deg)
    if (!d.has('hnavRollSinPeriodS'))     s.hnavRollSinPeriodS     = String(message.sin_period_s_hnav_roll_deg)
    if (!d.has('hnavRollSinSpread'))      s.hnavRollSinSpread      = String(message.sin_spread_hnav_roll_deg)
    if (!d.has('hnavPitchSinAmp'))        s.hnavPitchSinAmp        = String(message.sin_amplitude_hnav_pitch_deg)
    if (!d.has('hnavPitchSinPeriodS'))    s.hnavPitchSinPeriodS    = String(message.sin_period_s_hnav_pitch_deg)
    if (!d.has('hnavPitchSinSpread'))     s.hnavPitchSinSpread     = String(message.sin_spread_hnav_pitch_deg)

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
    syncBuf('hnavHeadingInput', message.hnav_heading_deg.toFixed(1), message.enable_move_hnav_heading_deg || message.enable_sin_hnav_heading_deg)
    syncBuf('hnavRollInput',    message.hnav_roll_deg.toFixed(1),    message.enable_move_hnav_roll_deg    || message.enable_sin_hnav_roll_deg)
    syncBuf('hnavPitchInput',   message.hnav_pitch_deg.toFixed(1),   message.enable_move_hnav_pitch_deg   || message.enable_sin_hnav_pitch_deg)
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
                 moveStateKey, stepBufKey, rateBufKey, moveTopicBase,
                 sinStateKey, sinAmpBufKey, sinPeriodBufKey, sinTopicBase,
                 stepStateKey,
                 waveStateKey, sinSpreadBufKey) {
    const { sendFloatMsg, sendBoolMsg } = this.props.ros
    const enableMove = this.state[moveStateKey]
    const hasSin     = sinStateKey != null
    const hasWave    = hasSin && waveStateKey != null
    // For sin-capable rows, Step is a sub-toggle; for non-sin rows, Step == Move
    const enableStep = hasSin
      ? (stepStateKey != null && !!this.state[stepStateKey])
      : enableMove
    const enableSin  = hasSin && !!this.state[sinStateKey]
    const enableWave = hasWave && !!this.state[waveStateKey]
    // Box is locked (orange, read-only) when the field is actively being driven
    const isLocked = hasSin ? (enableStep || enableSin) : enableMove
    const ns = this.getAppNamespace()

    const labelStyle = { fontSize: 11, color: '#aaa', display: 'block', marginBottom: 2 }

    return (
      <div key={bufKey} style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: Styles.vars.spacing.xs }}>
        <div style={{ minWidth: 100 }}>
          <Label title={label} />
        </div>
        <Input
          id={bufKey}
          value={this.state[bufKey]}
          onChange={isLocked ? undefined : (e) => {
            this.dirtyFields.add(bufKey)
            const el = document.getElementById(bufKey)
            setElementStyleModified(el)
            this.setState({ [bufKey]: e.target.value })
          }}
          onKeyDown={isLocked ? undefined : (e) => {
            if (e.key === 'Enter') {
              this.dirtyFields.delete(bufKey)
              const el = document.getElementById(bufKey)
              clearElementStyleModified(el)
              this.onPublishFloat(topicSuffix, this.state[bufKey])
            }
          }}
          readOnly={isLocked}
          disabled={disabled}
          backgroundOverride={isLocked ? Styles.vars.colors.orange : null}
          style={{ width: 100 }}
        />
        <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
          <Toggle
            checked={enableMove}
            onChange={() => {
              if (!enableMove) {
                // Opening: non-sin rows send backend command immediately; sin rows are local-only
                if (!hasSin) sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, true)
                this.setState({ [moveStateKey]: true })
              } else {
                // Closing: stop whatever is running on the backend
                if (enableStep || enableSin) sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, false)
                if (enableSin)  sendBoolMsg(ns + '/set_enable_sin_'  + sinTopicBase,  false)
                const update = { [moveStateKey]: false }
                if (stepStateKey) update[stepStateKey] = false
                if (sinStateKey)  update[sinStateKey]  = false
                this.setState(update)
              }
            }}
            disabled={disabled}
          />
          <span style={{ fontSize: 11, color: '#aaa' }}>Auto</span>
        </div>
        {/* Step group: toggle + boxes inline; hidden when Sin is active */}
        {(hasSin ? (enableMove && !enableSin) : enableMove) && (
          <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
            {hasSin && (
              <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
                <Toggle
                  checked={enableStep}
                  onChange={() => {
                    if (!enableStep) {
                      sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, true)
                      this.setState({ [stepStateKey]: true })
                    } else {
                      sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, false)
                      this.setState({ [stepStateKey]: false })
                    }
                  }}
                  disabled={disabled}
                />
                <span style={{ fontSize: 11, color: '#aaa' }}>Step</span>
              </div>
            )}
            <div>
              <span style={labelStyle}>Step</span>
              <Input
                id={stepBufKey}
                value={this.state[stepBufKey]}
                onChange={(e) => {
                  this.dirtyFields.add(stepBufKey)
                  const el = document.getElementById(stepBufKey)
                  setElementStyleModified(el)
                  this.setState({ [stepBufKey]: e.target.value })
                }}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    this.dirtyFields.delete(stepBufKey)
                    const el = document.getElementById(stepBufKey)
                    clearElementStyleModified(el)
                    sendFloatMsg(ns + '/set_move_step_' + moveTopicBase,
                                 parseFloat(this.state[stepBufKey]))
                  }
                }}
                disabled={disabled}
                style={{ width: 70 }}
              />
            </div>
            <div>
              <span style={labelStyle}>Rate Hz</span>
              <Input
                id={rateBufKey}
                value={this.state[rateBufKey]}
                onChange={(e) => {
                  this.dirtyFields.add(rateBufKey)
                  const el = document.getElementById(rateBufKey)
                  setElementStyleModified(el)
                  this.setState({ [rateBufKey]: e.target.value })
                }}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    this.dirtyFields.delete(rateBufKey)
                    const el = document.getElementById(rateBufKey)
                    clearElementStyleModified(el)
                    sendFloatMsg(ns + '/set_move_rate_hz_' + moveTopicBase,
                                 parseFloat(this.state[rateBufKey]))
                  }
                }}
                disabled={disabled}
                style={{ width: 70 }}
              />
            </div>
          </div>
        )}
        {/* Sin group: toggle + sub-mode + boxes; hidden when Step is active */}
        {enableMove && hasSin && !enableStep && (
          <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
              <Toggle
                checked={enableSin}
                onChange={() => {
                  if (!enableSin) {
                    sendBoolMsg(ns + '/set_enable_sin_'  + sinTopicBase,  true)
                    sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, true)
                    this.setState({ [sinStateKey]: true })
                  } else {
                    sendBoolMsg(ns + '/set_enable_sin_' + sinTopicBase, false)
                    sendBoolMsg(ns + '/set_enable_move_' + moveTopicBase, false)
                    if (enableWave) sendBoolMsg(ns + '/set_enable_wave_' + sinTopicBase, false)
                    const update = { [sinStateKey]: false }
                    if (waveStateKey) update[waveStateKey] = false
                    this.setState(update)
                  }
                }}
                disabled={disabled}
              />
              <span style={{ fontSize: 11, color: '#aaa' }}>Sin</span>
            </div>
            {/* Sub-mode: Sine | Wave */}
            {hasWave && (
              <React.Fragment>
                <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
                  <Toggle
                    checked={!enableWave}
                    onChange={() => {
                      if (enableWave) {
                        sendBoolMsg(ns + '/set_enable_wave_' + sinTopicBase, false)
                        this.setState({ [waveStateKey]: false })
                      }
                    }}
                    disabled={disabled}
                  />
                  <span style={{ fontSize: 11, color: '#aaa' }}>Sine</span>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
                  <Toggle
                    checked={enableWave}
                    onChange={() => {
                      if (!enableWave) {
                        sendBoolMsg(ns + '/set_enable_wave_' + sinTopicBase, true)
                        this.setState({ [waveStateKey]: true })
                      } else {
                        sendBoolMsg(ns + '/set_enable_wave_' + sinTopicBase, false)
                        this.setState({ [waveStateKey]: false })
                      }
                    }}
                    disabled={disabled}
                  />
                  <span style={{ fontSize: 11, color: '#aaa' }}>Wave</span>
                </div>
              </React.Fragment>
            )}
            {/* Shared sin params */}
            <div>
              <span style={labelStyle}>Amp (°)</span>
              <Input
                id={sinAmpBufKey}
                value={this.state[sinAmpBufKey]}
                onChange={(e) => {
                  this.dirtyFields.add(sinAmpBufKey)
                  const el = document.getElementById(sinAmpBufKey)
                  setElementStyleModified(el)
                  this.setState({ [sinAmpBufKey]: e.target.value })
                }}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    this.dirtyFields.delete(sinAmpBufKey)
                    const el = document.getElementById(sinAmpBufKey)
                    clearElementStyleModified(el)
                    sendFloatMsg(ns + '/set_sin_amplitude_' + sinTopicBase,
                                 parseFloat(this.state[sinAmpBufKey]))
                  }
                }}
                disabled={disabled}
                style={{ width: 70 }}
              />
            </div>
            <div>
              <span style={labelStyle}>Period (s)</span>
              <Input
                id={sinPeriodBufKey}
                value={this.state[sinPeriodBufKey]}
                onChange={(e) => {
                  this.dirtyFields.add(sinPeriodBufKey)
                  const el = document.getElementById(sinPeriodBufKey)
                  setElementStyleModified(el)
                  this.setState({ [sinPeriodBufKey]: e.target.value })
                }}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    this.dirtyFields.delete(sinPeriodBufKey)
                    const el = document.getElementById(sinPeriodBufKey)
                    clearElementStyleModified(el)
                    sendFloatMsg(ns + '/set_sin_period_s_' + sinTopicBase,
                                 parseFloat(this.state[sinPeriodBufKey]))
                  }
                }}
                disabled={disabled}
                style={{ width: 70 }}
              />
            </div>
            {/* Wave-only: Spread */}
            {enableWave && (
              <div>
                <span style={labelStyle}>Spread</span>
                <Input
                  id={sinSpreadBufKey}
                  value={this.state[sinSpreadBufKey]}
                  onChange={(e) => {
                    this.dirtyFields.add(sinSpreadBufKey)
                    const el = document.getElementById(sinSpreadBufKey)
                    setElementStyleModified(el)
                    this.setState({ [sinSpreadBufKey]: e.target.value })
                  }}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter') {
                      this.dirtyFields.delete(sinSpreadBufKey)
                      const el = document.getElementById(sinSpreadBufKey)
                      clearElementStyleModified(el)
                      sendFloatMsg(ns + '/set_sin_spread_' + sinTopicBase,
                                   parseFloat(this.state[sinSpreadBufKey]))
                    }
                  }}
                  disabled={disabled}
                  style={{ width: 60 }}
                />
              </div>
            )}
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
        {this.renderInputRow("Heading (°)", "hnavHeadingInput", "set_hnav_heading", dis, "enableMoveHnavHeading", "hnavHeadingStep", "hnavHeadingRateHz", "hnav_heading_deg", "enableSinHnavHeading", "hnavHeadingSinAmp", "hnavHeadingSinPeriodS", "hnav_heading_deg", "enableStepHnavHeading", "enableWaveHnavHeading", "hnavHeadingSinSpread")}
        {this.renderInputRow("Roll (°)",    "hnavRollInput",    "set_hnav_roll",    dis, "enableMoveHnavRoll",    "hnavRollStep",    "hnavRollRateHz",    "hnav_roll_deg",    "enableSinHnavRoll",    "hnavRollSinAmp",    "hnavRollSinPeriodS",    "hnav_roll_deg",    "enableStepHnavRoll",    "enableWaveHnavRoll",    "hnavRollSinSpread")}
        {this.renderInputRow("Pitch (°)",   "hnavPitchInput",   "set_hnav_pitch",   dis, "enableMoveHnavPitch",   "hnavPitchStep",   "hnavPitchRateHz",   "hnav_pitch_deg",   "enableSinHnavPitch",   "hnavPitchSinAmp",   "hnavPitchSinPeriodS",   "hnav_pitch_deg",   "enableStepHnavPitch",   "enableWaveHnavPitch",   "hnavPitchSinSpread")}

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
