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


// ---------------------------------------------------------------------------
// Learning aid: a one-line, plain-language explanation of every control type
// exposed by nepi_controls / ControlsIF. When a control does not carry its own
// `description` field, the matching line here is shown instead so that someone
// new to the framework can tell at a glance what each widget does and which
// nepi_controls service it drives.
// ---------------------------------------------------------------------------
const TYPE_HELP = {
  Menu:         "Pick one option from a drop-down. Sends the chosen option's index (set_menu_control_value).",
  Selection:    "Pick one option from a drop-down. Sends the chosen option's text (set_selection_control_value).",
  Selections:   "Turn any number of options on or off. Sends the full list of selected options (set_selections_control_value).",
  Trigger:      "A momentary action button. Sends a one-shot trigger with no value (set_trigger_control_value).",
  Bool:         "An on/off switch. Sends true or false (set_bool_control_value).",
  String:       "Free-form text. Type a value and press Enter to send it (set_string_control_value).",
  Int:          "A whole number. Type a value and press Enter to send it (set_int_control_value).",
  Float:        "A decimal number. Type a value and press Enter to send it (set_float_control_value).",
  FloatSlider:  "A single decimal value dragged between a min and max (set_floatslider_control_value).",
  FloatSliders: "A min/max decimal range dragged between two limits (set_floatsliders_control_value)."
}

// Accent colour used down the left edge of each control card, keyed loosely by
// interaction style so related controls read as a group. Purely cosmetic.
const TYPE_ACCENT = {
  Menu:         Styles.vars.colors.blue,
  Selection:    Styles.vars.colors.blue,
  Selections:   Styles.vars.colors.blue,
  Trigger:      Styles.vars.colors.orange,
  Bool:         Styles.vars.colors.green,
  String:       Styles.vars.colors.grey1,
  Int:          Styles.vars.colors.grey1,
  Float:        Styles.vars.colors.grey1,
  FloatSlider:  Styles.vars.colors.blue,
  FloatSliders: Styles.vars.colors.blue
}


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
    this.controlCard = this.controlCard.bind(this)
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

  // -------------------------------------------------------------------------
  // Presentational wrapper. Every control is rendered inside one of these
  // cards so that each control is clearly separated from its neighbours and
  // carries its own title, a type badge, and a short explanation. Behaviour
  // lives entirely in `children`; this method only handles layout.
  // -------------------------------------------------------------------------
  controlCard(name, type, display_name, description, children) {
    const accent = TYPE_ACCENT[type] || Styles.vars.colors.grey1
    // Prefer the control's own description; fall back to the built-in help
    // line so learners always see what the control does.
    const help = (description && description !== '') ? description : (TYPE_HELP[type] || '')

    return (
      <div
        key={name}
        style={{
          marginBottom: Styles.vars.spacing.regular,
          padding: Styles.vars.spacing.regular,
          border: `1px solid ${Styles.vars.colors.grey1}`,
          borderLeft: `4px solid ${accent}`,
          borderRadius: "4px"
        }}
      >
        {/* Header: control name on the left, its type as a badge on the right */}
        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between" }}>
          <span style={{ fontSize: Styles.vars.fontSize.medium, fontWeight: "bold" }}>
            {display_name}
          </span>
          <span
            style={{
              fontSize: Styles.vars.fontSize.small,
              fontWeight: "bold",
              color: Styles.vars.colors.white,
              backgroundColor: accent,
              borderRadius: "10px",
              padding: "2px 10px",
              whiteSpace: "nowrap"
            }}
          >
            {type}
          </span>
        </div>

        {/* Learner-friendly explanation of what this control does */}
        {(help !== '') ? (
          <div
            style={{
              fontSize: Styles.vars.fontSize.small,
              fontStyle: "italic",
              color: Styles.vars.colors.grey1,
              marginTop: Styles.vars.spacing.xs
            }}
          >
            {help}
          </div>
        ) : null}

        {/* The actual interactive widget */}
        <div style={{ marginTop: Styles.vars.spacing.small }}>
          {children}
        </div>
      </div>
    )
  }

  // Render a single control given its type and Control message.
  // Each block below maps one nepi_controls control type to its RUI widget and
  // the nepi_controls "set_*_control_value" topic it publishes to on change.
  renderControl(name, type, control_msg, index) {
    const namespace = this.getNamespace()
    const { sendUpdateIntMsg, sendUpdateStringMsg, sendUpdateBoolMsg } = this.props.ros
    const display_name = (control_msg.display_name && control_msg.display_name !== '') ? control_msg.display_name : name
    const description = control_msg.description

    // Value inputs whose value tracks either the in-progress edit or the message
    const editing = (name in this.state.editValues)

    // MENU -- drop-down of string options; the control's value is the *index*
    // of the selected option. Sends the new index as an Int.
    if (type === "Menu") {
      const options = control_msg.string_options
      const set_index = control_msg.set_index
      return this.controlCard(name, type, display_name, description,
        <Select
          id={'csbx_' + name}
          value={set_index}
          onChange={(e) => sendUpdateIntMsg(namespace + "/set_menu_control_value", name, parseInt(e.target.value, 10))}
        >
          {options.map((opt, i) => <Option key={name + '_' + i} value={i}>{opt}</Option>)}
        </Select>
      )
    }

    // SELECTION -- drop-down of string options; the control's value is the
    // selected option *text* (not its index). Sends the new text as a String.
    if (type === "Selection") {
      const options = control_msg.string_options
      const set_string = control_msg.set_string
      return this.controlCard(name, type, display_name, description,
        <Select
          id={'csbx_' + name}
          value={set_string}
          onChange={(e) => sendUpdateStringMsg(namespace + "/set_selection_control_value", name, e.target.value)}
        >
          {options.map((opt, i) => <Option key={name + '_' + i} value={opt}>{opt}</Option>)}
        </Select>
      )
    }

    // SELECTIONS -- a multi-select: each option gets its own toggle. The value
    // is the full array of currently-selected option strings. On every toggle
    // we send the complete desired selection (declarative), not a single delta.
    if (type === "Selections") {
      const options = control_msg.string_options
      const set_strings = control_msg.set_strings || []
      const { sendUpdateStringArrayMsg } = this.props.ros
      return this.controlCard(name, type, display_name, description,
        <div>
          {options.map((opt, i) => (
            <div key={name + '_' + i} style={{ display: "inline-block", marginRight: Styles.vars.spacing.regular, textAlign: "center" }}>
              <div style={{ fontSize: Styles.vars.fontSize.small, marginBottom: Styles.vars.spacing.xs }}>{opt}</div>
              <Toggle
                checked={set_strings.indexOf(opt) !== -1}
                onClick={() => {
                  // Send the complete desired selection (declarative), not a toggle.
                  const next = set_strings.indexOf(opt) !== -1
                    ? set_strings.filter((s) => s !== opt)
                    : [...set_strings, opt]
                  sendUpdateStringArrayMsg(namespace + "/set_selections_control_value", name, next)
                }}
              />
            </div>
          ))}
        </div>
      )
    }

    // TRIGGER -- a momentary action. There is no persistent value; pressing the
    // button fires a one-shot trigger (an empty String payload).
    if (type === "Trigger") {
      return this.controlCard(name, type, display_name, description,
        <ButtonMenu>
          <Button onClick={() => sendUpdateStringMsg(namespace + "/set_trigger_control_value", name, "")}>{"Trigger"}</Button>
        </ButtonMenu>
      )
    }

    // BOOL -- a single on/off switch. Sends the *opposite* of the current
    // value as a Bool each time it is clicked.
    if (type === "Bool") {
      const checked = (control_msg.set_bool === true)
      return this.controlCard(name, type, display_name, description,
        <Toggle
          checked={checked}
          onClick={() => sendUpdateBoolMsg(namespace + "/set_bool_control_value", name, !checked)}
        />
      )
    }

    // STRING / INT / FLOAT -- free-form typed values. These follow the PTX
    // editable-input pattern: the box shows an in-progress edit string while
    // the user types, and the value is sent (parsed to the right type) only on
    // Enter. See onInputChange / onInputKey above.
    if (type === "String" || type === "Int" || type === "Float") {
      var msgValue = ''
      if (type === "String") { msgValue = control_msg.set_string }
      else if (type === "Int") { msgValue = control_msg.set_int }
      else { msgValue = control_msg.set_float }
      const value = editing ? this.state.editValues[name] : msgValue
      return this.controlCard(name, type, display_name, description,
        <Input
          id={'csbx_' + name}
          style={{ width: "100%" }}
          value={value}
          onChange={(e) => this.onInputChange(name, e)}
          onKeyDown={(e) => this.onInputKey(name, type, e)}
        />
      )
    }

    // FLOATSLIDER -- a single decimal value dragged between a min and max.
    // float_bounds carries [min, max]; -999 in either slot means "no limit",
    // in which case we fall back to a sensible default (0 / 100).
    if (type === "FloatSlider") {
      const bounds = control_msg.float_bounds || []
      const min = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 100
      return this.controlCard(name, type, display_name, description,
        <SliderAdjustment
          title={""} /* name is shown in the card header, so keep the slider row clean */
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
      )
    }

    // FLOATSLIDERS -- a min/max *range* dragged between two limits. set_floats
    // holds the current [min, max] handles; float_bounds holds the outer
    // [min_limit, max_limit] the handles may move within.
    if (type === "FloatSliders") {
      const set_floats = control_msg.set_floats || [0, 1]
      const bounds = control_msg.float_bounds || []
      const min_limit = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max_limit = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 1
      return this.controlCard(name, type, display_name, description,
        <RangeAdjustment
          title={""} /* name is shown in the card header, so keep the slider row clean */
          comp_name={name}
          topic={namespace + "/set_floatsliders_control_value"}
          min={set_floats[0]}
          max={set_floats[1]}
          min_limit_m={min_limit}
          max_limit_m={max_limit}
          tooltip={control_msg.description}
          unit={""}
        />
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
