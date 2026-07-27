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

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import BooleanIndicator from "./BooleanIndicator"

import Nepi_IF_ImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import { round } from "./Utilities"

@inject("ros")
@observer

// Stereo Depth Application page
class StereoDepthApp extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_stereo_depth",
      appNamespace: null,
      statusListener: null,
      connected: false,
      needs_update: false,

      available_image_topics: [],
      selected_left_topic: "None",
      selected_right_topic: "None",
      left_connected: false,
      right_connected: false,

      sync_tolerance_s: 0.05,
      last_pair_dt_s: 0.0,

      calibrated: false,
      calibration_file: "",
      baseline_m: 0.06,
      image_width: 0,
      image_height: 0,
      last_rms_reproj_error: 0.0,

      calib_state: "idle",
      capture_count: 0,
      capture_target: 15,
      checkerboard_cols: 9,
      checkerboard_rows: 6,
      checkerboard_square_m: 0.025,

      num_disparities: 128,
      block_size: 7,
      min_range_m: 0.2,
      max_range_m: 20.0,

      running: false,
      status_message: "",

      // Local edit buffer for numeric inputs
      edit: {}
    }

    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getAllSaveNamespace = this.getAllSaveNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

    this.createTopicOptions = this.createTopicOptions.bind(this)
    this.onLeftSelected = this.onLeftSelected.bind(this)
    this.onRightSelected = this.onRightSelected.bind(this)
    this.onEditChange = this.onEditChange.bind(this)
    this.onEditKey = this.onEditKey.bind(this)
    this.editValue = this.editValue.bind(this)

    this.renderControls = this.renderControls.bind(this)
    this.renderCalibration = this.renderCalibration.bind(this)
    this.renderImages = this.renderImages.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null) {
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  getAllSaveNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null) {
      allNamespace = "/" + namespacePrefix + "/" + deviceId + "/save_data"
    }
    return allNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      available_image_topics: message.available_image_topics,
      selected_left_topic: message.selected_left_topic,
      selected_right_topic: message.selected_right_topic,
      left_connected: message.left_connected,
      right_connected: message.right_connected,
      sync_tolerance_s: message.sync_tolerance_s,
      last_pair_dt_s: message.last_pair_dt_s,
      calibrated: message.calibrated,
      calibration_file: message.calibration_file,
      baseline_m: message.baseline_m,
      image_width: message.image_width,
      image_height: message.image_height,
      last_rms_reproj_error: message.last_rms_reproj_error,
      calib_state: message.calib_state,
      capture_count: message.capture_count,
      capture_target: message.capture_target,
      checkerboard_cols: message.checkerboard_cols,
      checkerboard_rows: message.checkerboard_rows,
      checkerboard_square_m: message.checkerboard_square_m,
      num_disparities: message.num_disparities,
      block_size: message.block_size,
      min_range_m: message.min_range_m,
      max_range_m: message.max_range_m,
      running: message.running,
      status_message: message.status_message,
      connected: true
    })
  }

  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    const statusNamespace = namespace + "/status"
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStatusListener(
      statusNamespace,
      "nepi_app_stereo_depth/StereoDepthAppStatus",
      this.statusListener
    )
    this.setState({
      appNamespace: namespace,
      statusListener: statusListener,
      needs_update: false
    })
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    if ((namespace !== null && namespace !== this.state.appNamespace) || this.state.needs_update === true) {
      if (namespace !== null && namespace.indexOf("null") === -1) {
        this.updateStatusListener(namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }

  // Build Select options from the available image topics
  createTopicOptions(selected) {
    const topics = this.state.available_image_topics
    var items = []
    items.push(<Option value={"None"}>{"None"}</Option>)
    var i
    for (i = 0; i < topics.length; i++) {
      items.push(<Option value={topics[i]}>{topics[i]}</Option>)
    }
    return items
  }

  onLeftSelected(event) {
    const namespace = this.getAppNamespace()
    const item = event.target.value
    this.setState({ selected_left_topic: item })
    this.props.ros.sendStringMsg(namespace + "/set_left_topic", item)
  }

  onRightSelected(event) {
    const namespace = this.getAppNamespace()
    const item = event.target.value
    this.setState({ selected_right_topic: item })
    this.props.ros.sendStringMsg(namespace + "/set_right_topic", item)
  }

  // Generic numeric-input change handler: stash the edited value by field id
  onEditChange(event) {
    const field = event.target.id
    const value = event.target.value
    const edit = Object.assign({}, this.state.edit)
    edit[field] = value
    this.setState({ edit: edit })
  }

  // Generic numeric-input Enter handler: send the value on the given topic
  onEditKey(event, topic_suffix, msg_type) {
    if (event.key !== "Enter") {
      return
    }
    const namespace = this.getAppNamespace()
    const field = event.target.id
    const value = event.target.value
    if (msg_type === "int") {
      this.props.ros.sendIntMsg(namespace + topic_suffix, value)
    } else {
      this.props.ros.sendFloatMsg(namespace + topic_suffix, value)
    }
    const edit = Object.assign({}, this.state.edit)
    delete edit[field]
    this.setState({ edit: edit })
  }

  // Current input value: the local edit if present, otherwise the status value
  editValue(field, status_value) {
    if (this.state.edit[field] !== undefined) {
      return this.state.edit[field]
    }
    return status_value
  }

  renderControls() {
    const namespace = this.getAppNamespace()
    return (
      <Section title={"CAMERA SELECTION"}>

        <Label title={"Left Camera"}>
          <Select onChange={this.onLeftSelected} value={this.state.selected_left_topic}>
            {this.createTopicOptions(this.state.selected_left_topic)}
          </Select>
        </Label>

        <Label title={"Right Camera"}>
          <Select onChange={this.onRightSelected} value={this.state.selected_right_topic}>
            {this.createTopicOptions(this.state.selected_right_topic)}
          </Select>
        </Label>

        <Label title={"Left Connected"}>
          <BooleanIndicator value={this.state.left_connected} />
        </Label>
        <Label title={"Right Connected"}>
          <BooleanIndicator value={this.state.right_connected} />
        </Label>

        <Label title={"Frame Sync Tolerance (s)"}>
          <Input
            id={"sync_tolerance_s"}
            value={this.editValue("sync_tolerance_s", round(this.state.sync_tolerance_s, 3))}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_sync_tolerance", "float")}
          />
        </Label>
        <Label title={"Last Pair dt (s)"}>
          <Input disabled value={round(this.state.last_pair_dt_s, 3)} />
        </Label>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Num Disparities (x16)"}>
          <Input
            id={"num_disparities"}
            value={this.editValue("num_disparities", this.state.num_disparities)}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_num_disparities", "int")}
          />
        </Label>
        <Label title={"Block Size (odd)"}>
          <Input
            id={"block_size"}
            value={this.editValue("block_size", this.state.block_size)}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_block_size", "int")}
          />
        </Label>
        <Label title={"Min Range (m)"}>
          <Input
            id={"min_range_m"}
            value={this.editValue("min_range_m", round(this.state.min_range_m, 2))}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_min_range", "float")}
          />
        </Label>
        <Label title={"Max Range (m)"}>
          <Input
            id={"max_range_m"}
            value={this.editValue("max_range_m", round(this.state.max_range_m, 2))}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_max_range", "float")}
          />
        </Label>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Calibrated"}>
          <BooleanIndicator value={this.state.calibrated} />
        </Label>
        <Label title={"Running"}>
          <BooleanIndicator value={this.state.running} />
        </Label>

        <ButtonMenu>
          <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/start_pub")}>{"RUN"}</Button>
          <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/stop_pub")}>{"STOP"}</Button>
        </ButtonMenu>

        {this.renderConfig()}

      </Section>
    )
  }

  renderCalibration() {
    const namespace = this.getAppNamespace()
    return (
      <Section title={"STEREO CALIBRATION"}>

        <Label title={"State"}>
          <Input disabled value={this.state.calib_state} />
        </Label>
        <Label title={"Captures"}>
          <Input disabled value={this.state.capture_count + " / " + this.state.capture_target} />
        </Label>
        <Label title={"RMS Reproj Error"}>
          <Input disabled value={round(this.state.last_rms_reproj_error, 4)} />
        </Label>
        <Label title={"Image Size"}>
          <Input disabled value={this.state.image_width + " x " + this.state.image_height} />
        </Label>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Measured Baseline (m)"}>
          <Input
            id={"baseline_m"}
            value={this.editValue("baseline_m", round(this.state.baseline_m, 4))}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_baseline", "float")}
          />
        </Label>
        <Label title={"Checkerboard Cols"}>
          <Input
            id={"checkerboard_cols"}
            value={this.editValue("checkerboard_cols", this.state.checkerboard_cols)}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_checkerboard_cols", "int")}
          />
        </Label>
        <Label title={"Checkerboard Rows"}>
          <Input
            id={"checkerboard_rows"}
            value={this.editValue("checkerboard_rows", this.state.checkerboard_rows)}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_checkerboard_rows", "int")}
          />
        </Label>
        <Label title={"Square Size (m)"}>
          <Input
            id={"checkerboard_square_m"}
            value={this.editValue("checkerboard_square_m", round(this.state.checkerboard_square_m, 4))}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_square_size", "float")}
          />
        </Label>
        <Label title={"Capture Target"}>
          <Input
            id={"capture_target"}
            value={this.editValue("capture_target", this.state.capture_target)}
            onChange={this.onEditChange}
            onKeyDown={(e) => this.onEditKey(e, "/set_capture_target", "int")}
          />
        </Label>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <ButtonMenu>
          <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/start_calibration")}>{"START"}</Button>
          <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/capture_calibration")}>{"CAPTURE"}</Button>
        </ButtonMenu>
        <ButtonMenu>
          <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/compute_calibration")}>{"COMPUTE & SAVE"}</Button>
          <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/cancel_calibration")}>{"CANCEL"}</Button>
        </ButtonMenu>

        <Label title={"Status"}>
          <Input disabled value={this.state.status_message} />
        </Label>

      </Section>
    )
  }

  renderImages() {
    const namespace = this.getAppNamespace()
    const depth_image_topic = (namespace !== null) ? namespace + "/depth_map/depth_map_image" : "None"
    const pointcloud_image_topic = (namespace !== null) ? namespace + "/pointcloud/image" : "None"
    return (
      <React.Fragment>
        <div id="stereo_depth_image">
          <Nepi_IF_ImageViewer
            id="stereo_depth_image"
            title={"Depth Map"}
            image_topic={depth_image_topic}
          />
        </div>
        <div id="stereo_pointcloud_image">
          <Nepi_IF_ImageViewer
            id="stereo_pointcloud_image"
            title={"Point Cloud"}
            image_topic={pointcloud_image_topic}
          />
        </div>
      </React.Fragment>
    )
  }

  renderConfig() {
    const namespace = this.getAppNamespace()
    return (
      <NepiIFConfig
        namespace={namespace}
        title={"Nepi_IF_Config"}
        make_section={false}
      />
    )
  }

  renderSaveData() {
    const allSaveNamespace = this.getAllSaveNamespace()
    return (
      <NepiIFSaveData
        saveNamespace={allSaveNamespace}
        make_section={true}
        show_all_options={true}
        show_topic_selector={true}
      />
    )
  }

  render() {
    return (
      <React.Fragment>
        <div style={{ display: "flex" }}>

          <div style={{ width: "60%" }}>
            {this.renderImages()}
            {this.renderSaveData()}
          </div>

          <div style={{ width: "2%" }} />

          <div style={{ width: "38%" }}>
            {this.renderControls()}
            {this.renderCalibration()}
          </div>

        </div>
      </React.Fragment>
    )
  }
}

export default StereoDepthApp
