/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
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
import Toggle from "react-toggle"

import Section from "./Section"
import { Columns, Column } from "./Columns"
//import Select, { Option } from "./Select"
//import { SliderAdjustment } from "./AdjustmentWidgets"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
//import Button, { ButtonMenu } from "./Button"
import {onUpdateSetStateValue} from "./Utilities"

//import {onChangeSwitchStateValue } from "./Utilities"




function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

// Component that contains the PTX controls
class NepiAppPTAutoControls extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appName: 'app_pan_tilt_auto',
      appNamespace: null,  
      status_msg: null,  
   

      selected_topic: 'None',


      scanPanMin: -60,
      scanPanMax: 60,
      scanTiltMin: -60,
      scanTiltMax: 60,


      autoPanEnabled: false,
      autoTiltEnabled: false,

      track_source_namespaces: null,
      track_source_namespace: null,
      track_source_connected: null,

      click_pan_enabled: false,
      click_tilt_enabled: false,

      hide_click_controls: true,

      /*
      sinPanEnabled: false,
      #sinTiltEnabled: false,
      */

      speed_pan_dps: 0,
      speed_tilt_dps: 0,


      statusListener: null,         
      needs_update: false

    }


    this.onEnterSendPanScanRangeWindowValue = this.onEnterSendPanScanRangeWindowValue.bind(this)
    this.onEnterSendTiltScanRangeWindowValue = this.onEnterSendTiltScanRangeWindowValue.bind(this)

    this.renderControlPanel = this.renderControlPanel.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

  }

  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null){
      if (this.props.namespace !== undefined){
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  // Callback for handling ROS Status3DX messages
  statusListener(message) {
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message,
      selected_topic: message.selected_topic,

      autoPanEnabled: message.auto_pan_enabled,
      autoTiltEnabled: message.auto_tilt_enabled,

      track_source_namespaces: message.track_source_namespaces,
      track_source_namespace: message.track_source_namespace,
      track_source_connected: message.track_source_connected,

      trackPanEnabled: message.track_pan_enabled,
      trackTiltEnabled: message.track_tilt_enabled,
      
      sinPanEnabled: message.sin_pan_enabled,
      sinTiltEnabled: message.sin_tilt_enabled,

      click_pan_enabled: message.click_pan_enabled,
      click_tilt_enabled: message.click_tilt_enabled,

      
    })
  

    const pan_min_ss = message.scan_pan_min_deg
    const pan_max_ss = message.scan_pan_max_deg
    const tilt_min_ss = message.scan_tilt_min_deg
    const tilt_max_ss = message.scan_tilt_max_deg

    const scan_limits_changed = (last_status_msg == null) ? true : (pan_min_ss !== last_status_msg.scan_pan_min_deg || pan_max_ss !== last_status_msg.scan_pan_max_deg ||
                              tilt_min_ss !== last_status_msg.scan_tilt_min_deg || tilt_max_ss !== last_status_msg.scan_tilt_max_deg)
    if (scan_limits_changed === true){
      this.setState({scanPanMin: pan_min_ss,
                     scanPanMax: pan_max_ss
      })
    }
    if (scan_limits_changed === true){
      this.setState({scanTiltMin: tilt_min_ss,
                     scanTiltMax: tilt_max_ss
      })
    }
    
  }

  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener: null, status_msg: null})
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1){
        const statusNamespace = namespace + '/status'
        var statusListener = this.props.ros.setupStatusListener(
              statusNamespace,
              "nepi_app_pan_tilt_auto/PanTiltAutoAppStatus",
              this.statusListener
            )
    this.setState({ 
      statusListener: statusListener,
    })
    }
    this.setState({ 
      appNamespace: namespace,
      needs_update: false
    })
  }
  
// Lifecycle method called when compnent updates.
// Used to track changes in the topic
componentDidUpdate(prevProps, prevState, snapshot) {
  const namespace = this.getAppNamespace()
  if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true){
      this.updateStatusListener(namespace)
  }
}

  componentDidMount() {
    this.setState({ needs_update: true })
    }
  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message


componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
  }



  onClickToggleShowSettings(){
    const currentVal = this.state.showSettings 
    this.setState({showSettings: !currentVal})
    this.render()
  }




onEnterSendPanScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getAppNamespace()
  const topic_namespace = appnamespace + '/' + topicName
  var min = 0
  var max = 0
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (entryName === "max"){
      if (value < this.state.scanPanMin && !isNaN(value)){
        console.log("invaled range")

        const cur_max = this.state.status_msg.scan_pan_max_deg
        this.setState({scanPanMax: cur_max })
      }
      else{
        min = other_val
        max = value
        publishRangeWindow(topic_namespace,min,max,false)
      }
    }
    else if (entryName === "min") {
      if (value > this.state.scanPanMax && !isNaN(value)){
        console.log("invaled range")

        const cur_min = this.state.status_msg.scan_pan_min_deg
        this.setState({scanPanMin: cur_min })
      }
      else {
        min = value
        max = other_val
        publishRangeWindow(topic_namespace,min,max,false)
    }
    }
  
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }

}

onEnterSendTiltScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getAppNamespace()
  const topic_namespace = appnamespace + '/' + topicName
  var min = 0
  var max = 0
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (entryName === "max"){
      if (value < this.state.scanTiltMin && !isNaN(value)){
        console.log("invaled range")

        const cur_max = this.state.status_msg.scan_tilt_max_deg
        this.setState({scanTiltMax: cur_max })
      }
      else{
        min = other_val
        max = value
        publishRangeWindow(topic_namespace,min,max,false)
      }
    }
    else if (entryName === "min") {
      if (value > this.state.scanTiltMax && !isNaN(value)){
        console.log("invaled range")

        const cur_min = this.state.status_msg.scan_tilt_min_deg
        this.setState({scanTiltMin: cur_min })
      }
      else {
        min = value
        max = other_val
        publishRangeWindow(topic_namespace,min,max,false)
    }
    }
  
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }

}


  renderControlPanel() {
    const { ptxDevices, sendBoolMsg, onPTXGoHome, onPTXSetHomeHere } = this.props.ros
    
    const { reversePanEnabled, reverseTiltEnabled, autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled, 
            track_source_connected,
            speed_pan_dps, speed_tilt_dps, click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const ptx_caps = ptxDevices[selected_topic]
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning)
    //Unused const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    //Unused const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt_control)
    const has_auto_pan = ptx_caps && (ptx_caps.has_auto_pan)
    const has_auto_tilt = ptx_caps && (ptx_caps.has_auto_tilt)
    const has_speed_control = ptx_caps && (ptx_caps.has_adjustable_speed)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    //Unused const has_set_home = ptx_caps && (ptx_caps.has_set_home)

    const hide_auto_pan = ((has_auto_pan === false ))
    const hide_auto_tilt = ((has_auto_tilt === false ))

    const hide_track_pan = ((track_source_connected === false || hide_auto_pan === true))
    const hide_track_tilt = ((track_source_connected === false || hide_auto_tilt === true))

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getAppNamespace()

    const status_msg = this.state.status_msg
    const selected_topic = this.state.selected_topic
    const topics = Object.keys(ptxDevices)
    const connected_topic = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_topic)){
      connected_topic.push(topics[i])
    }
  }
    
    const pt_connected = (connected_topic.indexOf(selected_topic) !== -1)
    //console.log('pt_connected: ' + pt_connected)
    if (status_msg == null || pt_connected === false || namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
  

        return (
          <React.Fragment>


            <Label title={"PT Auto CONTROLS"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
            </Label>

            <Label title={"Enable Auto Scan"}>
            <div hidden={(hide_auto_pan === true)}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} checked={autoPanEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_auto_pan_enable",!autoPanEnabled)} />
              </div>
              </div>

              <div hidden={(hide_auto_tilt === true)}>
              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} checked={autoTiltEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_auto_tilt_enable",!autoTiltEnabled)} />
              </div>
              </div>
            </Label>

            <div hidden={(this.state.hide_click_controls === true)}>
            <Label title={"Select Pixel"}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} checked={click_pan_enabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_pan_click_enable",!click_pan_enabled)} />
              </div>


              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} checked={click_tilt_enabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_tilt_click_enable",!click_tilt_enabled)} />
              </div>
            </Label>
            </div>

            <Label title={"Enable Tracking"}>
              <div hidden={(hide_track_pan === true)}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} checked={trackPanEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_pan_enable",!trackPanEnabled)} />
              </div>
              </div>

              <div hidden={(hide_track_tilt === true)}>
              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} checked={trackTiltEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_tilt_enable",!trackTiltEnabled)} />
              </div>
              </div>
            </Label>


            <Label title={"Min Scan Limits"}>

              <Input id="scan_pan_min" 
                  value={this.state.scanPanMin} 
                  style={{ width: "45%", float: "left" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanPanMin")} 
                  onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_auto_pan_window","min",Number(this.state.scanPanMax))} />

              <Input id="scan_tilt_min" 
                  value={this.state.scanTiltMin} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanTiltMin")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_auto_tilt_window","min",Number(this.state.scanTiltMax))} />

              
            </Label>

            <Label title={"Max Scan Limits"}>

              <Input id="scan_pan_max" 
                value={this.state.scanPanMax} 
                style={{ width: "45%", float: "left" }}
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanPanMax")} 
                onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_auto_pan_window","max",Number(this.state.scanPanMin))} />     


              <Input id="scan_tilt_max" 
                  value={this.state.scanTiltMax} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanTiltMax")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_auto_tilt_window","max",Number(this.state.scanTiltMin))} />                      
            </Label>

            </React.Fragment>
        )
  }
}


  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true

    const status_msg = this.state.status_msg
    if (status_msg == null){
      return (
        <Columns>
        <Column>
       
        </Column>
        </Columns>
      )


    }
    else if (make_section === false){

      return (

          <Columns>
            <Column >

              { this.renderControlPanel()}


            </Column>
          </Columns>
      )
    }
    else {
      return (

          <Section title={(this.props.title != undefined) ? this.props.title : ""}>

              {this.renderControlPanel()}


        </Section>
     )
   }

  }


}

export default NepiAppPTAutoControls
