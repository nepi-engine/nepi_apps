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

import Toggle from "react-toggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import BooleanIndicator from "./BooleanIndicator"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import RangeAdjustment from "./RangeAdjustment"



import {setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue, onChangeChangeStateValue, onUpdateSetStateValue, round} from "./Utilities"
import {createMenuBaseNames, createMenuFirstLastNames, createMenuListFromStrLists, removeStringFromMenuNames} from "./Utilities"



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


      track_source_connected: false,
      track_reset_time: null,
      track_goal_deg: null,
      track_move_deg: null,
      tracking_topic: 'track',
      trackPanMin: -50,
      trackPanMax: 50,
      trackTiltMin: -50,
      trackTiltMax: 50,
      trackResetTime: null,

      stab_update_rate: null,
      stab_num_avg: null,
      stab_control_names: [],
      stab_control_values: [],
      stab_reset_time_sec: null,

      lockPanMin: -50,
      lockPanMax: 50,
      lockTiltMin: -50,
      lockTiltMax: 50,



      autoPanEnabled: false,
      autoTiltEnabled: false,

      stabPanEnabled: false,
      stabTiltEnabled: false,

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
    this.renderScanControls = this.renderScanControls.bind(this)


    this.onUpdateInputControlNameValue = this.onUpdateInputControlNameValue.bind(this)
    this.onKeySaveInputControlNameValue = this.onKeySaveInputControlNameValue.bind(this)
    this.renderStabControlValues = this.renderStabControlValues.bind(this)
    this.renderStabControls = this.renderStabControls.bind(this)
    this.onStabUpdateText = this.onStabUpdateText.bind(this)
    this.onStabKeyText = this.onStabKeyText.bind(this)

    this.onMenuSelection = this.onMenuSelection.bind(this)
    this.renderTrackControls = this.renderTrackControls.bind(this)

    this.onPTUpdateText = this.onPTUpdateText.bind(this)
    this.onPTKeyText = this.onPTKeyText.bind(this)
    this.renderPTControlPanel = this.renderPTControlPanel.bind(this)

    this.onTrackUpdateText = this.onTrackUpdateText.bind(this)
    this.onTrackKeyText = this.onTrackKeyText.bind(this)


    this.getNamespace = this.getNamespace.bind(this)
    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

  }

  getNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null){
      if (this.props.namespace !== undefined){
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }


  // Callback for handling ROS Status3DX messages
  statusListener(message) {
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message,
      selected_pan_tilt: message.selected_pan_tilt,

      autoPanEnabled: message.scan_pan_enabled,
      autoTiltEnabled: message.scan_tilt_enabled,

      track_source_connected: message.track_source_connected,

      trackPanEnabled: message.track_pan_enabled,
      trackTiltEnabled: message.track_tilt_enabled,
      
      stabPanEnabled: message.stab_pan_enabled,
      stabTiltEnabled: message.stab_tilt_enabled,

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

    var stab_controls_changed = true
   if (last_status_msg != null) {
      stab_controls_changed = JSON.stringify(message.stab_control_values) !== JSON.stringify(last_status_msg.stab_control_values)
   }
    
    if (stab_controls_changed === true){
      this.setState({stab_control_names: message.stab_control_names,
                    stab_control_values: message.stab_control_values
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
  const namespace = this.getNamespace()
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




  renderControlPanel() {
    const { ptxDevices, sendBoolMsg } = this.props.ros

    const { autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg


    if (status_msg == null || namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
 


      const show_control = this.state.show_control
        return (
          <React.Fragment>
   

          { this.renderPTControlPanel() }
        

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <div style={{ display: 'flex' }} >
                <div style={{ width: '60%' }} hidden={(show_control !== 'None' && show_control !== 'scan')} >

                        <Label title="Show Scanning Controls">
                            <Toggle
                              checked={(show_control === 'scan')}
                              onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'scan') ? 'None' : 'scan' )}>
                            </Toggle>
                        </Label>

                </div>

                <div style={{ width: '40%' }}>
                </div>

          </div>
          <div hidden={(show_control !== 'scan')}>
                {this.renderScanControls()}
          </div>

            <div style={{ display: 'flex' }} >
                <div style={{ width: '60%' }} hidden={(show_control !== 'None' && show_control !== 'track')}>

                        <Label title="Show Tracking Controls">
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

            <div style={{ display: 'flex' }} >
                <div style={{ width: '60%' }} hidden={(show_control !== 'None' && show_control !== 'stab')}>

                        <Label title="Show Stabilize Controls">
                            <Toggle
                              checked={(show_control === 'stab')}
                              onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'stab') ? 'None' : 'stab' )}>
                            </Toggle>
                        </Label>

                </div>

                <div style={{ width: '40%' }}>
                </div>

          </div>
          <div hidden={(show_control !== 'stab')}>
                {this.renderStabControls()}
          </div>

            </React.Fragment>
        )
  }
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




  renderPTControlPanel() {
    const { ptxDevices, sendBoolMsg } = this.props.ros

    const { autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

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


    if (status_msg == null || pt_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.pt_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.pt_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.pt_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.pt_status_msg.has_homing)
    const has_speed_control = (status_msg.pt_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.pt_status_msg.has_seperate_pan_tilt_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false

      const panPosition = status_msg.pt_status_msg.pan_now_deg
      const tiltPosition = status_msg.pt_status_msg.tilt_now_deg

      const panPositionClean = panPosition + .001
      const tiltPositionClean = tiltPosition + .001

      const panMove = status_msg.pt_status_msg.pan_goal_deg
      const tiltMove = status_msg.pt_status_msg.tilt_goal_deg

      const panMoveClean = panMove + .001
      const tiltMoveClean = tiltMove + .001

      const disable_pan_speed = (status_msg.stab_pan_enabled === true)
      const disable_tilt_speed = (status_msg.stab_tilt_enabled === true)
      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.pan_speed_ratio
      const speedTiltRatio = status_msg.tilt_speed_ratio


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
            <Button disabled={!has_homing} onClick={() => this.props.ros.onPTXGoHome(namespace)}>{"GO HOME"}</Button>
          </ButtonMenu>

          }

          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>

            <Label title={"Enable Scanning"}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} 
                  checked={autoPanEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_pan_enable",!autoPanEnabled)} />
              </div>


              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} 
                  checked={autoTiltEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_tilt_enable",!autoTiltEnabled)} />
              </div>

            </Label>


              <Label title={"Enable Tracking"}>

                <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                  <Toggle style={{justifyContent: "flex-left"}} 
                    disabled={disable_track_enable === true}
                    checked={trackPanEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_pan_enable",!trackPanEnabled)} />
                </div>


                <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                  <Toggle style={{justifyContent: "flex-right"}} 
                    disabled={disable_track_enable === true}
                    checked={trackTiltEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_tilt_enable",!trackTiltEnabled)} />
                </div>

              </Label>


              <Label title={"Enable Stabilize"}>

                <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                  <Toggle style={{justifyContent: "flex-left"}} 
                    disabled={true}
                    checked={stabPanEnabled === true && disable_stab_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_pan_enable",!stabPanEnabled)} />
                </div>


                <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                  <Toggle style={{justifyContent: "flex-right"}} 
                    disabled={disable_stab_enable === true}
                    checked={stabTiltEnabled === true && disable_stab_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_tilt_enable",!stabTiltEnabled)} />
                </div>

              </Label>


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

              <Label title={"Goal Position"}>
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



          <div hidden={(has_speed_control === false)}>

              <React.Fragment>
                <SliderAdjustment
                  disabled={disable_pan_speed}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedPanRatio}
                  topic={namespace + "/set_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
                <SliderAdjustment
                  disabled={disable_tilt_speed}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedTiltRatio}
                  topic={namespace + "/set_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
              </React.Fragment>
  
          </div>


            </React.Fragment>
        )
  }
}





onEnterSendPanScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getNamespace()
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
  const appnamespace = this.getNamespace()
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

  renderScanControls() {


    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg

   

        return (
          <React.Fragment>


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
{/* 
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                {"PT Frame - Angles in ENU frame (Tilt+:Down , Pan+:Left)"}
              </label> */}




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


  onMenuSelection(event){
    const {sendStringMsg} = this.props.ros

    const value = event.target.value
    const topic = event.target.id
    const namespace = this.getNamespace()
    const tracking_topic = this.state.tracking_topic
    const sendNamespace = namespace + '/' + tracking_topic + '/' +  topic
    sendStringMsg(sendNamespace,value)
  }



  onTrackUpdateText(e) {
    var element = null
    if (e.target.id === "ErrorGoalDeg")
      {
        element = document.getElementById("ErrorGoalDeg")
        setElementStyleModified(element)
        this.setState({track_goal_deg: e.target.value})
      }
    else if (e.target.id === "MinMoveDeg")
      {
        element = document.getElementById("MinMoveDeg")
        setElementStyleModified(element)
        this.setState({track_move_deg: e.target.value})
      }
    else if (e.target.id === "LostTargetTime")
      {
        element = document.getElementById("LostTargetTime")
        setElementStyleModified(element)
        this.setState({track_reset_time: e.target.value})
      }
  }

  onTrackKeyText(e) {
    const {sendFloatMsg} = this.props.ros
    const namespace = this.getNamespace()

    var element = null

    if(e.key === 'Enter'){
      if (e.target.id === "ErrorGoalDeg")
        {
          element = document.getElementById("ErrorGoalDeg")
          clearElementStyleModified(element)
          const goal_deg = parseFloat(e.target.value)
          if (!isNaN(goal_deg)){
            sendFloatMsg(namespace + "/set_track_goal_deg", goal_deg)
          }
          this.setState({track_goal_deg: null})
        }
      else if (e.target.id === "MinMoveDeg")
        {
          element = document.getElementById("MinMoveDeg")
          clearElementStyleModified(element)
          const move_deg = parseFloat(e.target.value)
          if (!isNaN(move_deg)){
            sendFloatMsg(namespace + "/set_track_move_deg", move_deg)
          }
          this.setState({track_move_deg: null})
        }
      else if (e.target.id === "LostTargetTime")
        {
          element = document.getElementById("LostTargetTime")
          clearElementStyleModified(element)
          this.setState({track_reset_time: null})
          const lost_time = parseFloat(e.target.value)
          if (!isNaN(lost_time)){
            sendFloatMsg(namespace + "/set_track_reset_time_sec", lost_time)
          }
        }
    }
  }




  onStabUpdateText(e) {
    var element = null
    const id = e.target.id
    const stateKey = {
      StabUpdateRate: 'stab_update_rate',
      StabNumAvg: 'stab_num_avg',
      StabPosDeg: 'stab_pos_deg',
      StabVelDeg: 'stab_vel_deg',
      StabResetTimeSec: 'stab_reset_time_sec'
    }[id]
    if (stateKey) {
      element = document.getElementById(id)
      setElementStyleModified(element)
      this.setState({[stateKey]: e.target.value})
    }
  }

  onStabKeyText(e) {
    const {sendFloatMsg, sendIntMsg} = this.props.ros
    const namespace = this.getNamespace()
    var element = null
    if (e.key === 'Enter') {
      const val = parseFloat(e.target.value)
      if (e.target.id === "StabUpdateRate") {
        element = document.getElementById("StabUpdateRate")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_update_rate", val) }
        this.setState({stab_update_rate: null})
      } else if (e.target.id === "StabNumAvg") {
        element = document.getElementById("StabNumAvg")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendIntMsg(namespace + "/set_stab_num_avg", Math.round(val)) }
        this.setState({stab_num_avg: null})
      } else if (e.target.id === "StabPosDeg") {
        element = document.getElementById("StabPosDeg")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_pos_deg", val) }
        this.setState({stab_pos_deg: null})
      } else if (e.target.id === "StabVelDeg") {
        element = document.getElementById("StabVelDeg")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_vel_deg", val) }
        this.setState({stab_vel_deg: null})
      } else if (e.target.id === "StabResetTimeSec") {
        element = document.getElementById("StabResetTimeSec")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_reset_time_sec", val) }
        this.setState({stab_reset_time_sec: null})
      }
    }
  }

  renderTrackControls() {


    const namespace = this.getNamespace()
    const track_namespace = namespace + '/' + this.state.tracking_topic

    const status_msg = this.state.status_msg
    const track_status_msg = status_msg.tracking_status

    const enabled = track_status_msg.enabled
    const running = track_status_msg.running
    const state = track_status_msg.state

    const available_targets = track_status_msg.available_targets_topics
    const has_targets = available_targets.length > 0
    const targets_names = createMenuBaseNames(available_targets)
    const targets_menu = createMenuListFromStrLists(available_targets,targets_names, ['None'], [],'None Available')
    const selected_targets = track_status_msg.selected_targets
    const targets_connected = track_status_msg.targets_connected

    
    const available_sources = track_status_msg.available_source_topics
    const sources_names = createMenuFirstLastNames(available_sources)
    const sources_menu = createMenuListFromStrLists(available_sources,sources_names, ['None'], [],'None Available')
    const selected_source = track_status_msg.selected_source
    const source_connected = track_status_msg.source_connected

    const available_classes = track_status_msg.available_classes
    const classes_menu = createMenuListFromStrLists(available_classes,available_classes, ['None'], [],'None Available')
    const selected_class = track_status_msg.selected_class
    const class_selected = (selected_class !== 'None')

    const threshold_filter = track_status_msg.threshold_filter

    const available_best = track_status_msg.available_best_filters
    const best_menu = createMenuListFromStrLists(available_best,available_best, [], [], '')
    const best_filter = track_status_msg.selected_best_filter


    const goal_deg = status_msg.track_goal_deg
    var track_goal_deg = this.state.track_goal_deg
    if (track_goal_deg == null){
      track_goal_deg = goal_deg
    }

    const move_deg = status_msg.track_move_deg
    var track_move_deg = this.state.track_move_deg
    if (track_move_deg == null){
      track_move_deg = move_deg
    }


    const move_ratio = status_msg.track_move_ratio
    const reset_time = status_msg.track_reset_time_sec
    var track_reset_time = this.state.track_reset_time
    if (track_reset_time == null){
      track_reset_time = reset_time
    }
    const track_pan_error = status_msg.track_pan_error
    const track_tilt_error = status_msg.track_tilt_error


    const manages_targeting = track_status_msg.manages_targeting

        return (
          <React.Fragment>

    <div hiddent={has_targets === false}>

 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                 <div style={{ display: 'flex' }}>
                      <div style={{ width: '10%' }}>

                      </div>

                      <div style={{ width: '30%' }}>


                          <Label title={"Targeting"}>
                            <BooleanIndicator value={running} />
                          </Label>


                      </div>

                      <div style={{ width: '10%' }}>

                      </div>

                      <div style={{ width: '30%' }} >

                          <Label title={"Tracking"}>
                            <BooleanIndicator value={state} />
                          </Label>

                      </div>

                      <div style={{ width: '10%' }}>

                      </div>
                </div>               

                <Label title={"Target Angles"}>
                    <Input
                      disabled
                      style={{ width: "45%", float: "left" }}
                      value={round(track_pan_error, 2)}
                    />

                    <Input
                      disabled
                      style={{ width: "45%" }}
                      value={round(track_tilt_error, 2)}
                    />
                </Label>

{/* 

                <div style={{ display: 'flex' }}>
                      <div style={{ width: '30%' }}>

                      <Label title={"Targets"}>
                    <BooleanIndicator value={targets_connected} />
                    </Label>

                      </div>

                      <div style={{ width: '30%' }}>

                       <Label title={"Image"}>
                      <BooleanIndicator value={source_connected  && targets_connected} />
                      </Label>

                      </div>

                      <div style={{ width: '30%' }} >


                      <Label title={"Class"}>
                     <BooleanIndicator value={class_selected && targets_connected } />
                     </Label>


                      </div>
          </div> */}

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <Label title={'Select Targets'}>
              <Select
                id="set_targets_topic"
                onChange={this.onMenuSelection}
                value={selected_targets}
              >
                {targets_menu}
              </Select>
            </Label>


        

              <Label title={'Select Image'}>
                <Select
                  id="set_source_topic"
                  onChange={this.onMenuSelection}
                  value={selected_source}
                >
                  {sources_menu}
                </Select>
              </Label>



              <Label title={'Select Class'}>
                <Select
                  id="set_class_filter"
                  onChange={this.onMenuSelection}
                  value={selected_class}
                >
                  {classes_menu}
                </Select>
              </Label>



                    <Label title={'Select Filter'}>
                      <Select
                        id="set_best_filter"
                        onChange={this.onMenuSelection}
                        value={best_filter}
                      >
                        {best_menu}
                      </Select>
                    </Label>

               

            <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  <SliderAdjustment
                            title={"Threshold"}
                            msgType={"std_msgs/Float32"}
                            adjustment={threshold_filter}
                            topic={track_namespace + "/set_threshold_filter"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Sets target confidence threshold filter"}
                            unit={"%"}
                    />


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>



          <Columns>
          <Column>

             <Label title={"Goal Deg"}>
                <Input
                  id={"ErrorGoalDeg"}
                  style={{ width: "45%", float: "left" }}
                  value={track_goal_deg}
                  onChange= {this.onTrackUpdateText}
                  onKeyDown= {this.onTrackKeyText}
                />
              </Label>

          </Column>
          <Column>

             <Label title={"Move Deg"}>
                <Input
                  id={"MinMoveDeg"}
                  style={{ width: "45%", float: "left" }}
                  value={track_move_deg}
                  onChange= {this.onTrackUpdateText}
                  onKeyDown= {this.onTrackKeyText}
                />
              </Label>


          </Column>
          <Column>

             <Label title={"Timeout"}>
                <Input
                  id={"LostTargetTime"}
                  style={{ width: "45%", float: "left" }}
                  value={track_reset_time}
                  onChange= {this.onTrackUpdateText}
                  onKeyDown= {this.onTrackKeyText}
                />
              </Label>



          </Column>
          </Columns>

                  <SliderAdjustment
                            title={"Move Sensitivity"}
                            msgType={"std_msgs/Float32"}
                            adjustment={move_ratio}
                            topic={namespace + "/set_track_move_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Sets target confidence threshold filter"}
                            unit={"%"}
                    />


 </div>


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Columns>
            <Column>

                      <div hiddent={has_targets === false}>
                        <ButtonMenu>
                          <Button onClick={() => window.open("/ai_detectors_mgr", "_blank")}>{"Open Detectors"}</Button>
                        </ButtonMenu>

                      </div>
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



  onUpdateInputControlNameValue(event, name, index) {
    const value = event.target.value
    var stab_control_values = this.state.stab_control_values
    stab_control_values[index] = value
    this.setState({ stab_control_values: stab_control_values })
    document.getElementById(name).style.color = Styles.vars.colors.red
    //this.render()
  }

  onKeySaveInputControlNameValue(event, name, index) {
    const namespace = this.getNamespace() + '/set_stab_control_value'

    if(event.key === 'Enter'){
      const value = event.target.value
      const parsed = parseFloat(value)
      if (!Number.isNaN(parsed)) {
        this.props.ros.sendUpdateFloatMsg(namespace,name,parsed)
      }
      else {
        const stab_control_values = this.state.status_msg.stab_control_values
        this.setState({ stab_control_values: stab_control_values })
      }
      document.getElementById(name).style.color = Styles.vars.colors.black
    }
  }



    
    renderStabControlValues(name, value, index) {
    return (

      <React.Fragment>

        <Label title={name}>
          <Input id={name} 
              style={{ width: "45%", float: "left" }}
              value={value} 
              onChange={(event) => this.onUpdateInputControlNameValue(event,name,index)} 
              onKeyDown= {(event) => this.onKeySaveInputControlNameValue(event,name,index)} />
        </Label>
      </React.Fragment>
    )
  }





  renderStabControls() {
    const namespace = this.getNamespace()
    const baseNamespace = this.getBaseNamespace()
    const status_msg = this.state.status_msg

    const available_sources = status_msg.available_stab_source_namespaces
    var sources_names = available_sources
    sources_names = removeStringFromMenuNames(sources_names,baseNamespace + '/')
    sources_names = removeStringFromMenuNames(sources_names,'navposes/')
    sources_names = removeStringFromMenuNames(sources_names,'navpose/')
    sources_names = removeStringFromMenuNames(sources_names,'npx/')
    const sources_menu = createMenuListFromStrLists(available_sources, sources_names, ['None'], [], 'None Available')
    const selected_source = status_msg.selected_stab_source

    const available_processes = status_msg.available_stab_processes
    const processes_menu = createMenuListFromStrLists(available_processes, available_processes, ['None'], [], 'None Available')
    const selected_process = status_msg.selected_stab_process

    var stab_update_rate = this.state.stab_update_rate
    if (stab_update_rate == null) { stab_update_rate = status_msg.stab_update_rate }

    var stab_num_avg = this.state.stab_num_avg
    if (stab_num_avg == null) { stab_num_avg = status_msg.stab_num_avg }

    var stab_reset_time_sec = this.state.stab_reset_time_sec
    if (stab_reset_time_sec == null) { stab_reset_time_sec = status_msg.stab_reset_time_sec }

    const stab_control_names = this.state.stab_control_names
    const stab_control_values = this.state.stab_control_values

    //const pan_tilt_max_speed_dps = status_msg.pan_tilt_max_speed_dps
    const stab_pt_min_speed_ratio = status_msg.stab_pt_min_speed_ratio
    const stab_pt_max_speed_ratio = status_msg.stab_pt_max_speed_ratio

    const pan_deg = status_msg.pan_deg
    const tilt_deg = status_msg.tilt_deg
    const pan_goal = status_msg.pan_goal
    const tilt_goal = status_msg.tilt_goal
    const pan_speed_dps = status_msg.pan_speed_dps
    const tilt_speed_dps = status_msg.tilt_speed_dps




    const stab_roll_deg = status_msg.stab_roll_deg
    const stab_roll_dps = status_msg.stab_roll_dps
    const stab_pitch_deg = status_msg.stab_pitch_deg
    const stab_pitch_dps = status_msg.stab_pitch_dps
    const stab_heading_deg = status_msg.stab_heading_deg
    const stab_heading_dps = status_msg.stab_heading_dps
    const stab_pan_deg = status_msg.stab_pan_deg
    const stab_pan_adj = status_msg.stab_pan_adj
    const stab_tilt_deg = status_msg.stab_tilt_deg
    const stab_tilt_adj = status_msg.stab_tilt_adj
    const stab_pan_goal = status_msg.stab_pan_goal
    const stab_pan_dps = status_msg.stab_pan_dps
    const stab_tilt_goal = status_msg.stab_tilt_goal
    const stab_tilt_dps = status_msg.stab_tilt_dps

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={'Select Source'}>
          <Select
            id="set_stab_source"
            onChange={(e) => this.props.ros.sendStringMsg(namespace + "/set_stab_source", e.target.value)}
            value={selected_source}
          >
            {sources_menu}
          </Select>
        </Label>

        <Label title={'Select Process'}>
          <Select
            id="set_stab_process"
            onChange={(e) => this.props.ros.sendStringMsg(namespace + "/set_stab_process", e.target.value)}
            value={selected_process}
          >
            {processes_menu}
          </Select>
        </Label>

          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/reload_stab_processes")}>{"Reload Processes"}</Button>
          </ButtonMenu>

        <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Update Rate"}>
          <Input
            id={"StabUpdateRate"}
            style={{ width: "45%", float: "left" }}
            value={stab_update_rate}
            onChange={this.onStabUpdateText}
            onKeyDown={this.onStabKeyText}
          />
        </Label>

        <Label title={"Num Avg"}>
          <Input
            id={"StabNumAvg"}
            style={{ width: "45%", float: "left" }}
            value={stab_num_avg}
            onChange={this.onStabUpdateText}
            onKeyDown={this.onStabKeyText}
          />
        </Label>

{/* 
        <Label title={"Reset Timeout"}>
          <Input
            id={"StabResetTimeSec"}
            style={{ width: "45%", float: "left" }}
            value={stab_reset_time_sec}
            onChange={this.onStabUpdateText}
            onKeyDown={this.onStabKeyText}
          />
        </Label> */}

        {/* <SliderAdjustment
          title={"Max Move Speed"}
          msgType={"std_msgs/Float32"}
          adjustment={stab_pt_max_speed_ratio}
          topic={namespace + "/set_stab_max_speed_ratio"}
          scaled={0.01}
          min={0}
          max={100}
          disabled={false}
          tooltip={"Sets stabilize max move speed"}
          unit={"%"}
        /> */}

          <div>
              {/* Map over the stab control names array */}
              {stab_control_names.map((name, index) => (
                this.renderStabControlValues(name, stab_control_values[index], index)
              ))}
            </div>

          <RangeAdjustment
            title="Min Max Speed"
            min={0}
            max={1}
            min_limit_m={stab_pt_min_speed_ratio}
            max_limit_m={stab_pt_max_speed_ratio}
            topic={namespace + "/set_stab_max_speed_ratios"}
            tooltip={"Sets stabilize min max move ratios"}
            noTextBox={true}
          />        

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


        <Label title={""}>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Deg"}</div>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Speed"}</div>
        </Label>


        <Label title={"Pan"}>
          <Input disabled style={{ width: "45%", float: "left" }} value={round(pan_deg, 2)} />
          <Input disabled style={{ width: "45%" }} value={round(pan_speed_dps, 2)} />
        </Label>

        <Label title={"Tilt"}>
          <Input disabled style={{ width: "45%", float: "left" }} value={round(tilt_deg, 2)} />
          <Input disabled style={{ width: "45%" }} value={round(tilt_speed_dps, 2)} />
        </Label>

        <Label title={""}>
          <div style={{ display: "inline-block", width: "30%", float: "left" }}>{"Roll"}</div>
          <div style={{ display: "inline-block", width: "30%", float: "center" }}>{"Pitch"}</div>
          <div style={{ display: "inline-block", width: "30%", float: "right" }}>{"Yaw"}</div>
        </Label>

        <Label title={"Nav"}>
          <Input disabled style={{ width: "30%", float: "left" }} value={round(stab_roll_deg, 2)} />
          <Input disabled style={{ width: "30%", float: "center" }} value={round(stab_pitch_deg, 2)} />
          <Input disabled style={{ width: "30%", float: "right" }} value={round(stab_heading_deg, 2)} />
        </Label>

        <Label title={""}>
          <div style={{ display: "inline-block", width: "30%", float: "left" }}>{"Adj"}</div>
          <div style={{ display: "inline-block", width: "30%", float: "center" }}>{"Goal"}</div>
          <div style={{ display: "inline-block", width: "30%", float: "right" }}>{"DPS"}</div>
        </Label>

        <Label title={"Pan"}>
          <Input disabled style={{ width: "30%", float: "left" }} value={round(stab_pan_adj, 2)} />
          <Input disabled style={{ width: "30%", float: "center" }} value={round(stab_pan_goal, 2)} />
          <Input disabled style={{ width: "30%", float: "right" }} value={round(stab_pan_dps, 2)} />
        </Label>

        <Label title={"Tilt"}>
           <Input disabled style={{ width: "30%", float: "left" }} value={round(stab_tilt_adj, 2)} />
          <Input disabled style={{ width: "30%", float: "center" }} value={round(stab_tilt_goal, 2)} />
          <Input disabled style={{ width: "30%", float: "right" }} value={round(stab_tilt_dps, 2)} />
        </Label>

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
