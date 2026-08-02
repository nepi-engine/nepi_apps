/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Toggle from "react-toggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import { SliderAdjustment } from "./AdjustmentWidgets"
import RangeAdjustment from "./RangeAdjustment"

import { setElementStyleModified, clearElementStyleModified } from "./Utilities"


@inject("ros")
@observer

// Renders one widget per control from a nepi_interfaces/ControlsStatus message.
class NepiAppControlsSandboxControls extends Component {
  constructor(props) {
    super(props)

    this.state = {
      controlsNamespace: null,
      status_msg: null,

      // name -> in-progress edit string for editable text/number inputs
      editValues: {},

      statusListener: null,
      needs_update: false
    }

    this.getNamespace = this.getNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.renderControl = this.renderControl.bind(this)
    this.onInputChange = this.onInputChange.bind(this)
    this.onInputKey = this.onInputKey.bind(this)
  }

  getNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null) {
      if (this.props.namespace !== undefined) {
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  statusListener(message) {
    this.setState({ status_msg: message })
  }

  updateStatusListener(namespace) {
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1) {
      const statusNamespace = namespace + '/status'
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_interfaces/ControlsStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ controlsNamespace: namespace, needs_update: false })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getNamespace()
    if ((namespace != null && namespace !== this.state.controlsNamespace) || this.state.needs_update === true) {
      this.updateStatusListener(namespace)
    }
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null })
    }
  }

  // Editable text/number input helpers (PTX controls pattern)
  onInputChange(name, e) {
    const el = document.getElementById('csbx_' + name)
    if (el) { setElementStyleModified(el) }
    const editValues = { ...this.state.editValues }
    editValues[name] = e.target.value
    this.setState({ editValues: editValues })
  }

  onInputKey(name, type, e) {
    if (e.key !== 'Enter') { return }
    const namespace = this.getNamespace()
    const { sendUpdateStringMsg, sendUpdateIntMsg, sendUpdateFloatMsg } = this.props.ros
    const el = document.getElementById('csbx_' + name)
    if (el) { clearElementStyleModified(el) }
    const raw = e.target.value
    if (type === "String") {
      sendUpdateStringMsg(namespace + "/set_string_control_value", name, raw)
    } else if (type === "Int") {
      const val = parseInt(raw, 10)
      if (!Number.isNaN(val)) { sendUpdateIntMsg(namespace + "/set_int_control_value", name, val) }
    } else if (type === "Float") {
      const val = parseFloat(raw)
      if (!Number.isNaN(val)) { sendUpdateFloatMsg(namespace + "/set_float_control_value", name, val) }
    }
    const editValues = { ...this.state.editValues }
    delete editValues[name]
    this.setState({ editValues: editValues })
  }

  // Render a single control given its type and Control message
  renderControl(name, type, control_msg, index) {
    const namespace = this.getNamespace()
    const { sendUpdateIntMsg, sendUpdateStringMsg, sendUpdateBoolMsg } = this.props.ros
    const display_name = (control_msg.display_name && control_msg.display_name !== '') ? control_msg.display_name : name

    // Value inputs whose value tracks either the in-progress edit or the message
    const editing = (name in this.state.editValues)

    if (type === "Menu") {
      const options = control_msg.string_options
      const set_index = control_msg.set_index
      return (
        <Label title={display_name} key={name}>
          <Select
            id={'csbx_' + name}
            value={set_index}
            onChange={(e) => sendUpdateIntMsg(namespace + "/set_menu_control_value", name, parseInt(e.target.value, 10))}
          >
            {options.map((opt, i) => <Option key={name + '_' + i} value={i}>{opt}</Option>)}
          </Select>
        </Label>
      )
    }

    if (type === "Selection") {
      const options = control_msg.string_options
      const set_string = control_msg.set_string
      return (
        <Label title={display_name} key={name}>
          <Select
            id={'csbx_' + name}
            value={set_string}
            onChange={(e) => sendUpdateStringMsg(namespace + "/set_selection_control_value", name, e.target.value)}
          >
            {options.map((opt, i) => <Option key={name + '_' + i} value={opt}>{opt}</Option>)}
          </Select>
        </Label>
      )
    }

    if (type === "Selections") {
      const options = control_msg.string_options
      const set_strings = control_msg.set_strings || []
      return (
        <Label title={display_name} key={name}>
          <div>
            {options.map((opt, i) => (
              <div key={name + '_' + i} style={{ display: "inline-block", marginRight: Styles.vars.spacing.small, textAlign: "center" }}>
                <div style={{ fontSize: "0.8em" }}>{opt}</div>
                <Toggle
                  checked={set_strings.indexOf(opt) !== -1}
                  onClick={() => sendUpdateStringMsg(namespace + "/set_selections_control_value", name, opt)}
                />
              </div>
            ))}
          </div>
        </Label>
      )
    }

    if (type === "Trigger") {
      return (
        <Label title={display_name} key={name}>
          <ButtonMenu>
            <Button onClick={() => sendUpdateStringMsg(namespace + "/set_trigger_control_value", name, "")}>{"Trigger"}</Button>
          </ButtonMenu>
        </Label>
      )
    }

    if (type === "Bool") {
      const checked = (control_msg.set_bool === true)
      return (
        <Label title={display_name} key={name}>
          <Toggle
            checked={checked}
            onClick={() => sendUpdateBoolMsg(namespace + "/set_bool_control_value", name, !checked)}
          />
        </Label>
      )
    }

    if (type === "String" || type === "Int" || type === "Float") {
      var msgValue = ''
      if (type === "String") { msgValue = control_msg.set_string }
      else if (type === "Int") { msgValue = control_msg.set_int }
      else { msgValue = control_msg.set_float }
      const value = editing ? this.state.editValues[name] : msgValue
      return (
        <Label title={display_name} key={name}>
          <Input
            id={'csbx_' + name}
            style={{ width: "60%", float: "left" }}
            value={value}
            onChange={(e) => this.onInputChange(name, e)}
            onKeyDown={(e) => this.onInputKey(name, type, e)}
          />
        </Label>
      )
    }

    if (type === "FloatSlider") {
      const bounds = control_msg.float_bounds || []
      const min = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 100
      return (
        <div key={name}>
          <SliderAdjustment
            title={display_name}
            comp_name={name}
            topic={namespace + "/set_floatslider_control_value"}
            msgType={"std_msgs/Float32"}
            adjustment={control_msg.set_float}
            min={min}
            max={max}
            scaled={1}
            tooltip={control_msg.description}
            unit={""}
          />
        </div>
      )
    }

    if (type === "FloatSliders") {
      const set_floats = control_msg.set_floats || [0, 1]
      const bounds = control_msg.float_bounds || []
      const min_limit = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max_limit = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 1
      return (
        <div key={name}>
          <RangeAdjustment
            title={display_name}
            comp_name={name}
            topic={namespace + "/set_floatsliders_control_value"}
            min={set_floats[0]}
            max={set_floats[1]}
            min_limit_m={min_limit}
            max_limit_m={max_limit}
            tooltip={control_msg.description}
            unit={""}
          />
        </div>
      )
    }

    return null
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = this.state.status_msg

    if (status_msg == null) {
      return (
        <Columns>
          <Column>
          </Column>
        </Columns>
      )
    }

    const names = status_msg.controls_name_list || []
    const types = status_msg.controls_type_list || []
    const msgs = status_msg.controls_msg_list || []

    const body = (
      <React.Fragment>
        {names.map((name, i) => {
          const control_msg = msgs[i]
          if (control_msg == null) { return null }
          // Hidden controls are not shown in the Controls box (they remain
          // manageable from the Controls Settings box).
          if (control_msg.hidden === true) { return null }
          return this.renderControl(name, types[i], control_msg, i)
        })}
      </React.Fragment>
    )

    if (make_section === false) {
      return body
    }
    return (
      <Section title={(this.props.title !== undefined) ? this.props.title : "CONTROLS"}>
        {body}
      </Section>
    )
  }
}

export default NepiAppControlsSandboxControls
