/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
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
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import {SliderAdjustment} from "./AdjustmentWidgets"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Input from "./Input"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Toggle from "react-toggle"
import BooleanIndicator from "./BooleanIndicator"


import ImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFSaveData from "./Nepi_IF_SaveData"


import {round, createMenuListFromStrList, onDropdownSelectedSendStr, onUpdateSetStateValue, onEnterSendFloatValue, onEnterSendIntValue, onEnterSetStateFloatValue} from "./Utilities"

@inject("ros")
@observer

// Component that contains the  Pointcloud App Viewer Controls
class AppAiTargeting extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {

      appName: "app_ai_targeting",
      appNamespace: null,

      
      app_enabled: false,
      app_msg: "Connecting",

      image_name: "targeting_image",

      classifier_running: false,

      image_topic: null,
      image_fov_vert_degs: null,
      image_fov_horz_degs: null,

      depth_map_topic: null,
      pointcloud_topic: null,

      available_classes_list: [],
      last_classes_list: [],
      selected_classes_list:[],
      selected_classes_depth_list: [],

      available_targets_list: ['None'],
      selected_target: null,

      target_box_reduction_percent: null,
      default_target_depth_m: null,
      target_min_points: null,
      target_min_px_ratio: null,
      target_min_dist_m: null,
      target_age_filter: null,

      target_box_size_percent: null,
      pc_box_size_percent: null,

     

      viewableTopics: false,

      transforms_topic_list: [],
      transforms_list: [],
      transformTX: 0,
      transformTY: 0,
      transformTZ: 0,
      transformRX: 0,
      transformRY: 0,
      transformRZ: 0,
      transformHO: 0,
      showTransforms: false,
      update_transform: false,

      statusListener: null,
      targetsListener: null,
      connected: false,
      needs_update: true

    }
  
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.targetsListener = this.targetsListener.bind(this)
    this.updateTargetsListener = this.updateTargetsListener.bind(this)
    this.onToggleClassSelection = this.onToggleClassSelection.bind(this)
    this.getClassOptions = this.getClassOptions.bind(this)
    this.toggleViewableTopics = this.toggleViewableTopics.bind(this)
    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    this.onClickToggleShowTransforms = this.onClickToggleShowTransforms.bind(this)
    this.sendClearTransformUpdateMessage = this.sendClearTransformUpdateMessage.bind(this)

  }

  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      if (this.state.connected === false){
        const pub_status_topic = appNamespace + "/publish_status"
        this.props.ros.sendTriggerMsg(pub_status_topic)
      }
    }
    return appNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({


    app_enabled: message.app_enabled,
    app_msg: message.app_msg,

    classifier_running: message.classifier_running,

    image_topic: message.image_topic,
    image_fov_vert_degs: message.image_fov_vert_degs,
    image_fov_horz_degs: message.image_fov_horz_degs,

    depth_map_topic: message.depth_map_topic,
    pointcloud_topic: message.pointcloud_topic,

    target_box_reduction_percent: message.target_box_reduction_percent,
    default_target_depth_m: message.default_target_depth_m,
    target_min_points: message.target_min_points,
    target_min_px_ratio: message.target_min_px_ratio,
    target_min_dist_m: message.target_min_dist_m,
    target_age_filter: message.target_age_filter,
    target_box_size_percent: message.target_box_size_percent,
    pc_box_size_percent: message.pointcloud_box_size_percent,

    available_classes_list: message.available_classes_list,
    selected_classes_list: message.selected_classes_list,
    selected_classes_depth_list: message.selected_classes_depth_list,
        })

    if (this.state.update_transform === true){
        this.setState({
          transformTX: message.frame_3d_transform.translate_vector.x,
          transformTY: message.frame_3d_transform.translate_vector.y,
          transformTZ: message.frame_3d_transform.translate_vector.z,
          transformRX: message.frame_3d_transform.rotate_vector.x,
          transformRY: message.frame_3d_transform.rotate_vector.y,
          transformRZ: message.frame_3d_transform.rotate_vector.z,
          transformHO: message.frame_3d_transform.heading_offset,
      })
    }
    this.setState({
      update_transform: false
    })

    this.setState({
      connected: true
    })

    const last_classes_list = this.state.last_classes_list
    this.setState({
      last_classes_list: this.state.available_targets_list
    })
    if (last_classes_list !== this.state.available_targets_list){
      this.render()
    }
  }


  // Callback for handling ROS Status messages
  targetsListener(message) {
    
    this.setState({
    selected_target: message.selected_target,
    available_targets_list: message.available_targets_list
    })

    const last_targets_list = this.state.last_targets_list
    this.setState({
      last_targets_list: message.available_targets_list
    })
    if (last_targets_list !== message.available_targets_list){
      this.render()
    }
  }


    // Function for configuring and subscribing to Status
    updateStatusListener() {
      const appNamespace = this.getAppNamespace()
      const statusNamespace = appNamespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_ai_targeting/AiTargetingStatus",
            this.statusListener
          )
      this.setState({ 
        statusListener: statusListener,
      })
      this.render()
    }

    // Function for configuring and subscribing to Status
    updateTargetsListener() {
      const appNamespace = this.getAppNamespace()
      const statusNamespace = appNamespace + '/targets'
      if (this.state.targetsListener) {
        this.state.targetsListener.unsubscribe()
      }
      var targetsListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_ai_targeting/AiTargetingTargets",
            this.targetsListener
          )
      this.setState({ 
        targetsListener: targetsListener,
      })
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (prevState.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1){
        this.setState({appNamespace: namespace})
        this.updateStatusListener()
        this.updateTargetsListener()
      } 
    }
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }



  settransform(event){
    const pointcloud = event.target.value
    const pointclouds = this.state.transforms_topic_list
    const transforms = this.state.transforms_list
    const tf_index = pointclouds.indexOf(pointcloud)
    if (tf_index !== -1){
      this.setState({
        transformPointcloud: pointcloud,
        transformInd: tf_index
      })
      const transform = transforms[tf_index]
      this.setState({
        transformTX: round(transform[0]),
        transformTY: round(transform[1]),
        transformTZ: round(transform[2]),
        transformRX: round(transform[3]),
        transformRY: round(transform[4]),
        transformRZ: round(transform[5]),
        transformHO: round(transform[6])
      })
      
    }
  }

  sendTransformUpdateMessage(){
    const {sendFrame3DTransformMsg} = this.props.ros
    const appNamespace = this.getAppNamespace()
    const namespace = appNamespace + "/set_frame_3d_transform"
    const TX = parseFloat(this.state.transformTX)
    const TY = parseFloat(this.state.transformTY)
    const TZ = parseFloat(this.state.transformTZ)
    const RX = parseFloat(this.state.transformRX)
    const RY = parseFloat(this.state.transformRY)
    const RZ = parseFloat(this.state.transformRZ)
    const HO = parseFloat(this.state.transformHO)
    const transformList = [TX,TY,TZ,RX,RY,RZ,HO]
    sendFrame3DTransformMsg(namespace,transformList)
    this.setState({
      update_transform: true
    })
  }

  sendClearTransformUpdateMessage(){
    const {sendClearFrame3DTransformMsg} = this.props.ros
    const appNamespace = this.getAppNamespace()
    const namespace = appNamespace + "/set_frame_3d_transform"
    const transformList = [0,0,0,0,0,0,0]
    sendClearFrame3DTransformMsg(namespace,transformList)
    this.setState({
      update_transform: true
    })
  }






  // Function for creating image topic options.
  getClassOptions() {
  const classesList = this.state.available_classes_list
  var items = []
  items.push(<Option>{"None"}</Option>)
  items.push(<Option>{"All"}</Option>)
  if (classesList.length > 0 ){
    for (var i = 0; i < classesList.length; i++) {
        if (classesList[i] !== 'None'){
          items.push(<Option value={classesList[i]}>{classesList[i]}</Option>)
        }
    }
  }
  return items
  }


  toggleViewableTopics() {
    const set = !this.state.viewableTopics
    this.setState({viewableTopics: set})
  }


  onToggleClassSelection(event){
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const appNamespace = this.getAppNamespace()
    const classSelection = event.target.value
    const selectedClassesList = this.state.selected_classes_list
    const addAllNamespace = appNamespace + "/add_all_target_classes"
    const removeAllNamespace = appNamespace + "/remove_all_target_classes"
    const addNamespace = appNamespace + "/add_target_class"
    const removeNamespace = appNamespace + "/remove_target_class"
    if (appNamespace){
      if (classSelection === "None"){
          sendTriggerMsg(removeAllNamespace)
      }
      else if (classSelection === "All"){
        sendTriggerMsg(addAllNamespace)
    }
      else if (selectedClassesList.indexOf(classSelection) !== -1){
        sendStringMsg(removeNamespace,classSelection)
      }
      else {
        sendStringMsg(addNamespace,classSelection)
      }
    }
  }

  onClickToggleShowTransforms(){
    const newVal = this.state.showTransforms === false
    this.setState({showTransforms: newVal})
    this.render()
  }

  renderApp() {
    const {sendBoolMsg, sendTriggerMsg,} = this.props.ros
    const classOptions = this.getClassOptions()
    const selectedClasses = this.state.selected_classes_list
    const NoneOption = <Option>None</Option>
    const connected = this.state.connected === true
    const appNamespace = this.getAppNamespace()
    const classes_sel = selectedClasses[0] !== "" && selectedClasses[0] !== "None"

    return (
      <Section title={"AI Targeting App"}>

        <Columns>
        <Column>


                  <Columns>
                  <Column>

                  <div hidden={(connected === true)}>

                <pre style={{ height: "40px", overflowY: "auto" ,fontWeight: 'bold' , color: Styles.vars.colors.Green, textAlign: "left" }}>
                    {"Loading or Refresh Page"}
                  </pre>

                </div>

                <div hidden={(connected === false)}>

                  <Label title="Enable App">
                      <Toggle
                      checked={this.state.app_enabled===true}
                      onClick={() => sendBoolMsg(appNamespace + "/enable_app",!this.state.app_enabled)}>
                      </Toggle>
                </Label>

                </div>

                    </Column>
                  <Column>

                  </Column>
                </Columns>


        <div hidden={(connected !== true || this.state.app_enabled !== true)}>



              <Columns>
              <Column>


          <Label title={"AI Detection Running"}>
            <BooleanIndicator value={this.state.classifier_running} />
          </Label>


                </Column>
              <Column>

              <Label title={"Target Classes Selected"}>
            <BooleanIndicator value={classes_sel} />
          </Label>
              </Column>
            </Columns>

          
            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
              {"App Settings"}
            </label>

            <Columns>
            <Column>
            

                <Label title="Select Class Filters"> </Label>

                        <div onClick={this.toggleViewableTopics} style={{backgroundColor: Styles.vars.colors.grey0}}>
                          <Select style={{width: "10px"}}/>
                        </div>
                        <div hidden={this.state.viewableTopics === false}>
                        {classOptions.map((Class) =>
                        <div onClick={this.onToggleClassSelection}
                          style={{
                            textAlign: "center",
                            padding: `${Styles.vars.spacing.xs}`,
                            color: Styles.vars.colors.black,
                            backgroundColor: (selectedClasses.includes(Class.props.value))? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                            cursor: "pointer",
                            }}>
                            <body class_name ={Class} style={{color: Styles.vars.colors.black}}>{Class}</body>
                        </div>
                        )}
                        </div>



                        <Label title="Select Target Filter"> </Label>

                        <Select
                          id="select_target"
                          onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/select_target")}
                          value={this.state.selected_target}
                        >
                          {this.state.available_targets_list
                            ? createMenuListFromStrList(this.state.available_targets_list, false, [],[],[])
                            : NoneOption}
                        </Select>


                  </Column>
                  <Column>




            <Label title={"Sensor Vertical Degrees"}>
              <Input id="image_fov_vert_degs" 
                value={this.state.image_fov_vert_degs} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"image_fov_vert_degs")} 
                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_image_fov_vert")} />
            </Label>
              
            <Label title={"Sensor Horzontal Degrees"}>
              <Input id="image_fov_horz_degs" 
                value={this.state.image_fov_horz_degs} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"image_fov_vert_degs")} 
                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_image_fov_horz")} />
            </Label>

            <Label title={"Default Target Depth (m)"}>
              <Input id="default_target_depth_m" 
                value={this.state.default_target_depth_m} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"default_target_depth_m")} 
                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_default_target_detpth")} />
            </Label>

            <Label title={"Target Min Points"}>
              <Input id="target_min_points" 
                value={this.state.target_min_points} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"target_min_points")} 
                onKeyDown= {(event) => onEnterSendIntValue.bind(this)(event,appNamespace + "/set_target_min_points")} />
            </Label>

            <Label title={"Target Min Dist (m)"}>
              <Input id="target_min_dist_m" 
                value={this.state.target_min_dist_m} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"target_min_dist_m")} 
                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_target_min_dist_meters")} />
            </Label>

            <Label title={"Target Age Filter"}>
              <Input id="target_age_filter" 
                value={this.state.target_age_filter} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"target_age_filter")} 
                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_age_filter")} />
            </Label>

            <Label title={"Set Box Adjust %"}>
              <Input id="target_box_size_percent" 
                value={this.state.target_box_size_percent} 
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"target_box_size_percent")} 
                onKeyDown= {(event) => onEnterSendIntValue.bind(this)(event,appNamespace + "/set_target_box_size_percent")} />
            </Label>


            <SliderAdjustment
              title={"Target Min Pixel Ratio"}
              msgType={"std_msgs/float32"}
              adjustment={this.state.target_min_px_ratio}
              topic={appNamespace + "/set_target_min_px_ratio"}
              scaled={0.01}
              min={0}
              max={100}
              tooltip={""}
              unit={"%"}
          />

            </Column>
            </Columns>




        <Columns>
        <Column>

        <Label title="Show 3D Transforms">
        <Toggle
          checked={this.state.showTransforms}
          onClick={this.onClickToggleShowTransforms}>
        </Toggle>
      </Label>

        </Column>
        <Column>

        </Column>
        </Columns>






        <div hidden={ this.state.showTransforms === false}>

          <Columns>
          <Column>

          <Label title={"X (m)"}>
                <Input
                  value={this.state.transformTX}
                  id="XTranslation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTX")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTX")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Y (m)"}>
                <Input
                  value={this.state.transformTY}
                  id="YTranslation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTY")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTY")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Z (m)"}>
                <Input
                  value={this.state.transformTZ}
                  id="ZTranslation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformTZ")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformTZ")}
                  style={{ width: "80%" }}
                />
              </Label>


              <ButtonMenu>
                <Button onClick={() => this.sendTransformUpdateMessage()}>{"Update Transform"}</Button>
              </ButtonMenu>


            </Column>
            <Column>

              <Label title={"Roll (deg)"}>
                <Input
                  value={this.state.transformRX}
                  id="XRotation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRX")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRX")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Pitch (deg)"}>
                <Input
                  value={this.state.transformRY}
                  id="YRotation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRY")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRY")}
                  style={{ width: "80%" }}
                />
              </Label>

                  <Label title={"Yaw (deg)"}>
                    <Input
                      value={this.state.transformRZ}
                      id="ZRotation"
                      onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"transformRZ")}
                      onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"transformRZ")}
                      style={{ width: "80%" }}
                    />
                  </Label>


                  <ButtonMenu>
                <Button onClick={() => this.sendClearTransformUpdateMessage()}>{"Clear Transform"}</Button>
              </ButtonMenu>



          </Column>
          </Columns>

          </div>




          </div>


      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <Columns>
          <Column>

            <ButtonMenu>
              <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_app")}>{"Reset App"}</Button>
            </ButtonMenu>

            </Column>
          <Column>

              <ButtonMenu>
                <Button onClick={() => sendTriggerMsg(appNamespace + "/save_config")}>{"Save Config"}</Button>
          </ButtonMenu>

          </Column>
          <Column>

          <ButtonMenu>
                <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_config")}>{"Reset Config"}</Button>
          </ButtonMenu>

  
          </Column>
        </Columns>



      </Column>
        </Columns>

      </Section>

    
    )
  }


  render() {
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const connected = this.state.connected === true
    const appNamespace = (connected) ? this.getAppNamespace() : null
    const imageNamespace = appNamespace + '/' + this.state.image_name

    return (

      <Columns>
      <Column equalWidth={true}>

      <div hidden={!connected}>

    <NepiIFSaveData
      saveNamespace={appNamespace}
      title={"Nepi_IF_SaveData"}
    />

    </div>

      <ImageViewer
        imageTopic={imageNamespace}
        title={this.state.image_name}
        hideQualitySelector={false}
      />


      </Column>
      <Column>



      {this.renderApp()}


      </Column>
      </Columns>

      )
    }  

}
export default AppAiTargeting
