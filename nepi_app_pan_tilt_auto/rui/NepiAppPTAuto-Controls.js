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
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import {setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue, onChangeChangeStateValue, onUpdateSetStateValue, round} from "./Utilities"

import Nepi_IF_Tracking from "./Nepi_IF_Tracking"

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
   
      show_control: 'None',

      selected_pan_tilt: 'None',
      linkSpeeds: true,

      scanPanMin: -50,
      scanPanMax: 50,
      scanTiltMin: -50,
      scanTiltMax: 50,

      trackPanMin: -50,
      trackPanMax: 50,
      trackTiltMin: -50,
      trackTiltMax: 50,
      trackResetTime: null,

      lockPanMin: -50,
      lockPanMax: 50,
      lockTiltMin: -50,
      lockTiltMax: 50,



      autoPanEnabled: false,
      autoTiltEnabled: false,

      track_available_sources: null,
      track_selected_source: null,
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
    this.renderPTControls = this.renderPTControls.bind(this)
    this.renderTrackControls = this.renderTrackControls.bind(this)

    this.onPTUpdateText = this.onPTUpdateText.bind(this)
    this.onPTKeyText = this.onPTKeyText.bind(this)

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
      selected_pan_tilt: message.selected_pan_tilt,

      autoPanEnabled: message.scan_pan_enabled,
      autoTiltEnabled: message.scan_tilt_enabled,

      track_available_sources: message.track_available_sources,
      track_selected_source: message.track_selected_source,
      track_source_connected: message.track_source_connected,

      trackPanEnabled: message.track_pan_enabled,
      trackTiltEnabled: message.track_tilt_enabled,
      
      sinPanEnabled: message.sin_pan_enabled,
      sinTiltEnabled: message.sin_tilt_enabled,

      click_pan_enabled: message.click_pan_enabled,
      click_tilt_enabled: message.click_tilt_enabled,

      
    })
  
    const pan_min_scan = round(message.scan_pan_min_deg, 0)
    const pan_max_scan = round(message.scan_pan_max_deg, 0)
    const tilt_min_scan = round(message.scan_tilt_min_deg, 0)
    const tilt_max_scan = round(message.scan_tilt_max_deg, 0)

    var scan_limits_changed = true
    if (last_status_msg != null) {


      const last_pan_min_scan = round(message.scan_pan_min_deg, 0)
      const last_pan_max_scan = round(message.scan_pan_max_deg, 0)
      const last_tilt_min_scan = round(message.scan_tilt_min_deg, 0)
      const last_tilt_max_scan = round(message.scan_tilt_max_deg, 0)

      scan_limits_changed = (pan_min_scan !== last_pan_min_scan || pan_max_scan !== last_pan_max_scan ||
                                tilt_min_scan !== last_tilt_min_scan || tilt_max_scan !== last_tilt_max_scan)
      }
      
      if (scan_limits_changed === true){
        this.setState({scanPanMin: pan_min_scan,
                      scanPanMax: pan_max_scan
        })
      }
      if (scan_limits_changed === true){
        this.setState({scanTiltMin: tilt_min_scan,
                      scanTiltMax: tilt_max_scan
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




  onPTUpdateText(e) {
    var panElement = null
    var tiltElement = null
    if (e.target.id === "PTXPanGoto") 
      {
        panElement = document.getElementById("PTXPanGoto")
        setElementStyleModified(panElement)
        this.setState({panGoto: e.target.value})
             
      }
        
    else if  (e.target.id === "PTXTiltGoto")
        {
          tiltElement = document.getElementById("PTXTiltGoto")
          setElementStyleModified(tiltElement)
          this.setState({tiltGoto: e.target.value})         
          
        }

  }

  onPTKeyText(e) {
    const {ptxDevices, onSetPTXGotoPos, onSetPTXGotoPanPos, onSetPTXGotoTiltPos, onSetPTXHomePos, onSetPTXSoftStopPos} = this.props.ros

    const selected_pan_tilt = this.state.selected_pan_tilt
    const ptx_caps = ptxDevices[selected_pan_tilt]
    const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt_control)
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    const has_speed_control = ptx_caps && (ptx_caps.has_adjustable_speed)
    const has_sep_speed = ptx_caps && (ptx_caps.has_seperate_pan_tilt_speed === true)
    //Unused const has_set_home = ptx_caps && (ptx_caps.has_set_home)

    var panElement = null
    var tiltElement = null
    if(e.key === 'Enter'){
      if (e.target.id === "PTXPanGoto") 
        {
          panElement = document.getElementById("PTXPanGoto")
          tiltElement = document.getElementById("PTXTiltGoto")                    
          if (has_sep_pan_tilt === true){
            onSetPTXGotoPanPos(selected_pan_tilt, Number(panElement.value))
          }
          else {
            onSetPTXGotoPos(selected_pan_tilt, Number(panElement.value),Number(tiltElement.value))
          }            
          clearElementStyleModified(panElement)   
          this.setState({panGoto: null})    
          
        }
        else if  (e.target.id === "PTXTiltGoto")
          {
            
            panElement = document.getElementById("PTXPanGoto")
            tiltElement = document.getElementById("PTXTiltGoto")

            if (has_sep_pan_tilt === true){
              onSetPTXGotoTiltPos(selected_pan_tilt, Number(tiltElement.value))
            }
            else {
              onSetPTXGotoPos(selected_pan_tilt, Number(panElement.value),Number(tiltElement.value))
            }              
            clearElementStyleModified(tiltElement)
            this.setState({tiltGoto: null})      
          
          }
    }
  }




  renderControlPanel() {
    const { ptxDevices, sendBoolMsg } = this.props.ros

    const { autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getAppNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(ptxDevices)
    const pt_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_pan_tilt)){
      pt_connected_topics.push(topics[i])
    }
  }
    
    const pt_connected = (pt_connected_topics.indexOf(selected_pan_tilt) !== -1)
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

    const ptx_caps = ptxDevices[selected_pan_tilt]
    //Unused const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    //Unused const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt_control)
    const has_scan_pan = ptx_caps && (ptx_caps.has_scan_pan)
    const has_scan_tilt = ptx_caps && (ptx_caps.has_scan_tilt)
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    const has_speed_control = ptx_caps && (ptx_caps.has_adjustable_speed)
    const has_sep_speed = ptx_caps && (ptx_caps.has_seperate_pan_tilt_speed === true)
    //Unused const has_set_home = ptx_caps && (ptx_caps.has_set_home)

    const hide_scan_pan = ((has_scan_pan === false ))
    const hide_scan_tilt = ((has_scan_tilt === false ))

    const hide_track_pan = ((track_source_connected === false || hide_scan_pan === true))
    const hide_track_tilt = ((track_source_connected === false || hide_scan_tilt === true))

  
      const pt_namespace = status_msg.pt_connected_topic
      const panPosition = status_msg.pt_status_msg.pan_now_deg
      const tiltPosition = status_msg.pt_status_msg.tilt_now_deg

      const panPositionClean = panPosition + .001
      const tiltPositionClean = tiltPosition + .001

      const panMove = status_msg.pt_status_msg.pan_goal_deg
      const tiltMove = status_msg.pt_status_msg.tilt_goal_deg

      const panMoveClean = panMove + .001
      const tiltMoveClean = tiltMove + .001


      const speedRatio = status_msg.pt_status_msg.speed_ratio
      const speedPanRatio = status_msg.pt_status_msg.speed_pan_ratio
      const speedTiltRatio = status_msg.pt_status_msg.speed_tilt_ratio

      const show_control = this.state.show_control
        return (
          <React.Fragment>



          { (has_homing === false) ?


          <ButtonMenu>
            <Button onClick={() => this.props.ros.onPTXStop(selected_pan_tilt)}>{"STOP"}</Button>
          </ButtonMenu>

          :

          <ButtonMenu>
            <Button onClick={() => this.props.ros.onPTXStop(selected_pan_tilt)}>{"STOP"}</Button>
            <Button disabled={!has_homing} onClick={() => this.props.ros.onPTXGoHome(selected_pan_tilt)}>{"GO HOME"}</Button>
          </ButtonMenu>

          }

          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>

            <Label title={"Enable Auto Scan"}>
            <div hidden={(hide_scan_pan === true)}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} checked={autoPanEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_pan_enable",!autoPanEnabled)} />
              </div>
              </div>

              <div hidden={(hide_scan_tilt === true)}>
              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} checked={autoTiltEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_tilt_enable",!autoTiltEnabled)} />
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


            <div hidden={track_source_connected == false}>
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
            </div>


          <div hidden={(has_abs_pos === false)}>

              <Label title={"GoTo Position "}>
                <Input
                  disabled={!has_abs_pos}
                  id={"PTXPanGoto"}
                  style={{ width: "45%", float: "left" }}
                  value={this.state.panGoto}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
                <Input
                  disabled={!has_abs_pos}
                  id={"PTXTiltGoto"}
                  style={{ width: "45%" }}
                  value={this.state.tiltGoto}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
              </Label>


              <Label title={"Current Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={round(panPositionClean, 2)}
                />
                <Input
                  disabled
                  style={{ width: "45%" }}
                  value={round(tiltPositionClean, 2)}
                />
              </Label>

              <Label title={"Move Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={round(panMoveClean, 2)}
                />
                <Input
                  disabled
                  style={{ width: "45%" }}
                  value={round(tiltMoveClean, 2)}
                />
              </Label>



          </div>

   

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <div style={{ display: 'flex' }} hidden={(show_control !== 'None' && show_control !== 'pantilt')}>
                <div style={{ width: '60%' }} >

                        <Label title="Show PanTilt Controls">
                            <Toggle
                              checked={(show_control === 'pantilt')}
                              onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'pantilt') ? 'None' : 'pantilt' )}>
                            </Toggle>
                        </Label>

                </div>

                <div style={{ width: '40%' }}>
                </div>

          </div>
          <div hidden={(show_control !== 'pantilt')}>
                {this.renderPTControls()}
          </div>

            <div style={{ display: 'flex' }} hidden={(show_control !== 'None' && show_control !== 'track')}>
                <div style={{ width: '60%' }} >

                        <Label title="Show Track Controls">
                            <Toggle
                              checked={(show_control === 'track')}
                              onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'track') ? 'None' : 'track' )}>
                            </Toggle>
                        </Label>

                </div>

                <div style={{ width: '40%' }}>
                </div>

          </div>
          <div hidden={(show_control !== 'track')}>
                {this.renderTrackControls()}
          </div>

            </React.Fragment>
        )
  }
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

  renderPTControls() {


    const namespace = this.getAppNamespace()

    const status_msg = this.state.status_msg

    const selected_pan_tilt = this.state.selected_pan_tilt
    const ptx_caps = this.props.ros.ptxDevices[selected_pan_tilt]
    //Unused const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    //Unused const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt_control)
    const has_scan_pan = ptx_caps && (ptx_caps.has_scan_pan)
    const has_scan_tilt = ptx_caps && (ptx_caps.has_scan_tilt)
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    const has_speed_control = ptx_caps && (ptx_caps.has_adjustable_speed)
    const has_sep_speed = ptx_caps && (ptx_caps.has_seperate_pan_tilt_speed === true)
    //Unused const has_set_home = ptx_caps && (ptx_caps.has_set_home)

    const speedRatio = status_msg.pt_status_msg.speed_ratio
    const speedPanRatio = status_msg.pt_status_msg.speed_pan_ratio
    const speedTiltRatio = status_msg.pt_status_msg.speed_tilt_ratio

   

        return (
          <React.Fragment>


          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                {"PT Frame - Angles in ENU frame (Tilt+:Down , Pan+:Left)"}
              </label>


          <div hidden={(has_speed_control === false)}>



            {(has_sep_speed === true && this.state.linkSpeeds === false) ? (
              <React.Fragment>
                <SliderAdjustment
                  disabled={!has_speed_control}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedPanRatio}
                  topic={selected_pan_tilt + "/set_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
                <SliderAdjustment
                  disabled={!has_speed_control}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedTiltRatio}
                  topic={selected_pan_tilt + "/set_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
              </React.Fragment>
            ) : (
              <SliderAdjustment
                disabled={!has_speed_control}
                title={"Speed"}
                msgType={"std_msgs/Float32"}
                adjustment={speedRatio}
                topic={selected_pan_tilt + "/set_speed_ratio"}
                scaled={0.01}
                min={0}
                max={100}
                tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                unit={"%"}
              />
            )}


                {(has_sep_speed === true) ?
                <Columns>

                  <Column>
                  </Column>

                  <Column>
                      <Label title="Link Speeds">
                      <Toggle
                        checked={this.state.linkSpeeds === true}
                        onClick={() => onChangeSwitchStateValue.bind(this)("linkSpeeds", this.state.linkSpeeds)}>
                      </Toggle>
                    </Label>
                  </Column>
                </Columns>
                : null }


          </div>



            <Label title={""}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
            </Label>

            <Label title={"Min Scan Limit"}>

              <Input id="scan_pan_min" 
                  value={this.state.scanPanMin} 
                  style={{ width: "45%", float: "left" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanPanMin")} 
                  onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_scan_pan_window","min",Number(this.state.scanPanMax))} />

              <Input id="scan_tilt_min" 
                  value={this.state.scanTiltMin} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanTiltMin")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_scan_tilt_window","min",Number(this.state.scanTiltMax))} />

              
            </Label>


            <Label title={"Max Scan Limit"}>

              <Input id="scan_pan_max" 
                value={this.state.scanPanMax} 
                style={{ width: "45%", float: "left" }}
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanPanMax")} 
                onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_scan_pan_window","max",Number(this.state.scanPanMin))} />     


              <Input id="scan_tilt_max" 
                  value={this.state.scanTiltMax} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanTiltMax")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_scan_tilt_window","max",Number(this.state.scanTiltMin))} />                      
            </Label>





            </React.Fragment>
        )
  
}



  renderTrackControls() {


    const namespace = this.getAppNamespace()

    const status_msg = this.state.status_msg

    const selected_pan_tilt = this.state.selected_pan_tilt

   

        return (
          <React.Fragment>


          <Nepi_IF_Tracking                        
            namespace={namespace + '/tracking'}
            make_section={false}
          />


        <Columns>
            <Column>
                      <ButtonMenu>
                        <Button onClick={() => window.open("/ai_detectors_mgr", "_blank")}>{"Open Detectors"}</Button>
                      </ButtonMenu>
            </Column>

            <Column>
                    <ButtonMenu>
                      <Button onClick={() => window.open("/ai_models_mgr", "_blank")}>{"Open Models"}</Button>
                    </ButtonMenu>          
            </Column>
        </Columns>




            </React.Fragment>
        )
  
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

          <React.Fragment>

              { this.renderControlPanel()}


          </React.Fragment>
      )
    }
    else {
      return (

          <Section title={(this.props.title !== undefined) ? this.props.title : ""}>


              {this.renderControlPanel()}


        </Section>
     )
   }

  }


}

export default NepiAppPTAutoControls
