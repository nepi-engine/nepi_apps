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


import Section from "./Section"
//import EnableAdjustment from "./EnableAdjustment"
//import Button, { ButtonMenu } from "./Button"
import Label from "./Label"
import { Column, Columns } from "./Columns"
//import Input from "./Input"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Toggle from "react-toggle"
import BooleanIndicator from "./BooleanIndicator"
import {SliderAdjustment} from "./AdjustmentWidgets"


import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
//import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import {filterStrList, createMenuFirstLastNames} from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer


class NepiIFTracking extends Component {
  detector_info = []
  constructor(props) {
    super(props)

    this.state = {

      connectedToNepi: false,
      connectedToAiModelsMgr: false,


      status_msg: null,


      selected_detector: "None",
      last_selected_detector: "None",

      img_list_viewable: false,
      img_filter_str_list: ['detection_image','targeting_image','alert_image','tracking_image'],


      selected_display_topic: "None",
      selected_display_text: "None",

      classes_list_viewable: false,
      availableClassesList: [],
      selectedClassesList:[],

      showSettingsControl: this.props.showSettingsControl ? this.props.showSettingsControl : false,      
      showSettings: false,

      statusListener: null,
      connected: false,

      needs_update: false


    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.checkConnection = this.checkConnection.bind(this)

    this.updateStatusListeners = this.updateStatusListeners.bind(this)
    this.statusListener = this.statusListener.bind(this)

    this.getDetectorOptions = this.getDetectorOptions.bind(this)
    this.onDetectorSelected = this.onDetectorSelected.bind(this)

    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)

    this.toggleImagesListViewable = this.toggleImagesListViewable.bind(this)
    this.onImagesTopicSelected = this.onImagesTopicSelected.bind(this)


    this.onToggleClassSelection = this.onToggleClassSelection.bind(this)
    this.getClassOptions = this.getClassOptions.bind(this)
    this.toggleClassesListViewable = this.toggleClassesListViewable.bind(this)

    this.getDisplayImgOptions = this.getDisplayImgOptions.bind(this)
    this.onDisplayImgSelected = this.onDisplayImgSelected.bind(this)
    this.getSaveNamespace = this.getSaveNamespace.bind(this)

    this.renderAiDetector = this.renderAiDetector.bind(this)
    this.renderDetectorSettings = this.renderDetectorSettings.bind(this)

  }
  
  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }

  async checkConnection() {
    const { connectedToNepi , connectedToAiModelsMgr} = this.props.ros
    if (this.state.connectedToNepi !== connectedToNepi){
      this.setState({connectedToNepi: connectedToNepi,
                    status_msg: null,
                    selected_model: 'None'})
    }
    if (this.state.connectedToAiModelsMgr !== connectedToAiModelsMgr )
    {
      this.setState({connectedToAiModelsMgr: connectedToAiModelsMgr, connected: true})
      this.setState({needs_update: true})
    }

    setTimeout(async () => {
      await this.checkConnection()
    }, 1000)
  }



  componentDidMount(){
    this.checkConnection()
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    const sel_detector = this.state.selected_detector
    const got_detector = message.namespace

    if (sel_detector === got_detector){
      this.setState({
      status_msg: message,
      connected: true
      })
    }

  }

  // Function for configuring and subscribing to Status
  updateStatusListeners() {

    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({
        status_msg: null,
        connected: false,
        statusListener: null,
        selected_display_topic: "None",
        selected_display_text: "None"
    })
    }
    const ai_models_namespaces = this.props.ros.ai_models_running_namespace_list
    const selected_detector = this.state.selected_detector
    const detector_ind = ai_models_namespaces.indexOf(selected_detector)
    if (detector_ind !== -1){

      var statusListener = this.props.ros.setupStatusListener(
        selected_detector + '/status',
        "nepi_interfaces/AiDetectorStatus",
        this.statusListener
      )
      this.setState({ 
        statusListener: statusListener,
      })
    }

  }


  
  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    if (this.state.needs_update === true){
      const selected_detector = this.state.selected_detector
      const last_detector = this.state.last_selected_detector

      if (last_detector !== selected_detector) {
          this.setState({      
            last_selected_detector: selected_detector
          })  
          this.updateStatusListeners()

      }
      this.setState({needs_update: false})
    }
     
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message
  componentWillUnmount() {
      this.state.statusListener.unsubscribe()
      this.setState({
        status_msg: null,
        connected: false,
        statusListener: null,
        selected_display_topic: "None",
        selected_display_text: "None"
    })
  }



  // Function for creating image topic options.
  getDetectorOptions() {
    const ai_models_namespaces = this.props.ros.ai_models_running_namespace_list
    const ai_models_display_names = this.props.ros.ai_models_running_name_list
    const ai_models_types = this.props.ros.ai_models_running_type_list
    const selected_detector = this.state.selected_detector
    var items = []
    var check_type = 'detection'
    var type = 'Unknown'

    if (ai_models_namespaces.length === 0) {
      items.push(<Option value={'None'}>{'None'}</Option>)
    }
    else {
      for (var i = 0; i < ai_models_namespaces.length; i++) {
          type = ai_models_types[i]
          if (type === check_type ){
            //items.push(<Option value={ai_models_namespaces[i]}>{ai_models_names[i]}</Option>)
            items.push(<Option value={ai_models_namespaces[i]}>{ai_models_display_names[i]}</Option>)
          }
      }
    }

    if ( ai_models_namespaces.indexOf(selected_detector) === -1){
      if (ai_models_namespaces.length > 0){
        this.setState({selected_detector: ai_models_namespaces[0]})
      }
      else if (selected_detector !== 'None') {
        this.setState({selected_detector: 'None'})
      }
    }
    return items
  }

  onDetectorSelected(event){
    const detector = event.target.value
    this.setState({selected_detector: detector})
    this.setState({needs_update: true})
  }

  renderAiDetector() {

    const detector_options = this.getDetectorOptions()

    const selected_detector = this.state.selected_detector

    const status_msg = this.state.status_msg
    const detector_namespace = (status_msg == null) ? "None" : status_msg.namespace
    const connected = (detector_namespace === selected_detector)? this.state.connected : false



    const { userRestricted} = this.props.ros
    const detector_controls_restricted = userRestricted.indexOf('MANAGER-AI-DETECTORS-CONTROL') !== -1


      return (


        <Section title={"AI Detection Manager"}>

          <Columns>
          <Column>

             <Label title="Select Detector">
                  <Select id="DetectorSelect" onChange={this.onDetectorSelected} 
                      value={selected_detector}
                      disabled={false}>
                    {detector_options}
                  </Select>
              </Label>
    

              <div hidden={((status_msg != null) || selected_detector === 'None')}>

                  <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {"Loading..."}
                  </pre>

              </div>

          </Column>
          </Columns>

   
              { (status_msg != null && connected === true && detector_controls_restricted === false) ? this.renderDetectorSettings() : null}



  
          </Section>

      )
  

  }



  // Function for creating image topic options.
  createImageTopicsOptions() {
    const filter_str_list = this.state.img_filter_str_list
    const { imageTopics } = this.props.ros
    const img_options = filterStrList(imageTopics,filter_str_list)
    var imageTopicShortnames = createMenuFirstLastNames(img_options)
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    items.push(<Option value={'All'}>{'All'}</Option>)
    const sel_det = this.state.selected_detector
    for (var i = 0; i < img_options.length; i++) {
      if (img_options[i].indexOf(sel_det) === -1){
       items.push(<Option value={img_options[i]}>{imageTopicShortnames[i]}</Option>)
      }
    }
    return items
  }

  toggleImagesListViewable() {
    const set = !this.state.img_list_viewable
    this.setState({img_list_viewable: set})
  }


  onImagesTopicSelected(event){
    const {imageTopics, sendStringMsg, sendStringArrayMsg} = this.props.ros
    const detector_namespace = this.state.selected_detector
    const add_img_namespace = detector_namespace + "/add_img_topic"
    const add_imgs_namespace = detector_namespace + "/add_img_topics"
    const remove_img_namespace = detector_namespace + "/remove_img_topic"
    const remove_imgs_namespace = detector_namespace + "/remove_img_topics"
    const filter_str_list = this.state.img_filter_str_list
    const img_options = filterStrList(imageTopics,filter_str_list)
    const det_img_topics = this.state.status_msg.selected_images
    const img_topic = event.target.value
    //this.setState({selected_display_topic: img_topic})

    if (img_topic === "None"){
        sendStringArrayMsg(remove_imgs_namespace,img_options)
    }
    else if (img_topic === "All"){
        sendStringArrayMsg(add_imgs_namespace,img_options)
    }
    else if (det_img_topics.indexOf(img_topic) === -1){
      sendStringMsg(add_img_namespace,img_topic)
    }
    else {
      sendStringMsg(remove_img_namespace,img_topic)
    }
  }


  // Function for creating image topic options.
  getClassOptions() {

    var items = []
    items.push(<Option>{"None"}</Option>)

    const selected_detector = this.state.selected_detector
    const status_msg = this.state.status_msg
    if (status_msg != null){
      const detector_namespace = status_msg.namespace 
      if (selected_detector === detector_namespace){
        items.push(<Option>{"All"}</Option>)
        if (status_msg.available_classes.length > 0 ){
          for (var i = 0; i < status_msg.available_classes.length; i++) {
              if (status_msg.available_classes[i] !== 'None'){
                items.push(<Option value={status_msg.available_classes[i]}>{status_msg.available_classes[i]}</Option>)
              }
          }
        }
      }
    }
    return items
    }
  
  
    toggleClassesListViewable() {
      const set = !this.state.classes_list_viewable
      this.setState({classes_list_viewable: set})
    }
  
  
    onToggleClassSelection(event){
      const {sendTriggerMsg, sendStringMsg} = this.props.ros

      const selected_detector = this.state.selected_detector
      const status_msg = this.state.status_msg
      if (status_msg != null){
        const detector_namespace = status_msg.namespace 
        if (selected_detector === detector_namespace){
          const classSelection = event.target.value
          const selectedClassesList = status_msg.selected_classes
          const addAllNamespace = detector_namespace + "/add_all_classes"
          const removeAllNamespace = detector_namespace + "/remove_all_classes"
          const addNamespace = detector_namespace + "/add_class"
          const removeNamespace = detector_namespace + "/remove_class"
          if (detector_namespace){
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
      }
    }


renderDetectorSettings() {
  const { sendBoolMsg } = this.props.ros


  const sel_img = 'Unselected' //this.state.selected_display_topic

  const classOptions = this.getClassOptions()

  const selected_detector = this.state.selected_detector
  const status_msg = this.state.status_msg
  if (status_msg != null){
    const detector_namespace = status_msg.namespace
    if (selected_detector === detector_namespace){
      






      const selectedClassesList = status_msg.selected_classes
      const classes_selected = (selectedClassesList.length > 0)

      const detector_enabled = status_msg.enabled
      const detecting = status_msg.state
      const detector_state = status_msg.msg_str
      const det_running = (detector_state === "Detecting")


      const pub_image_enabled = status_msg.pub_image_enabled
      const overlay_labels = status_msg.overlay_labels
      const overlay_range_bearing = status_msg.overlay_range_bearing
      const overlay_detector_name = status_msg.overlay_clf_name
      const overlay_img_name = status_msg.overlay_img_name

      const threshold = status_msg.threshold_filter
      const max_det_rate = status_msg.max_proc_rate_hz 
      const max_img_rate = status_msg.max_img_rate_hz
      const use_last_image = status_msg.use_last_image

      const det_img_topics = status_msg.selected_images

      const img_selected = status_msg.image_selected
      const img_connected = status_msg.image_connected

      const image_receive_rate = round(status_msg.avg_image_receive_rate, 3)

      const image_process_time = round(status_msg.avg_image_process_time, 3)

      const image_process_latency = round(status_msg.avg_image_process_latency, 3)
      const image_process_rate = round(status_msg.avg_image_process_rate, 3)

      const detect_process_time = round(status_msg.avg_detect_process_time, 3)

      const detect_process_latency = round(status_msg.avg_detect_process_latency, 3)
      const detect_process_rate = round(status_msg.avg_detect_process_rate, 3)

      const max_detect_rate = round(status_msg.max_detect_rate, 3)


      const img_options = this.createImageTopicsOptions()

      return (
      <Columns>
      <Column>



          <Columns>
        <Column>

          <Label title={"Select Images"}/>

            </Column>
          <Column>

                <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <div onClick={this.toggleImagesListViewable} style={{backgroundColor: Styles.vars.colors.grey0}}>
          <Select style={{width: "10px"}}/>
        </div>
        <div hidden={this.state.img_list_viewable === false}>
        {img_options.map((image) =>
        <div onClick={this.onImagesTopicSelected}
          style={{
            textAlign: "center",
            padding: `${Styles.vars.spacing.xs}`,
            color: Styles.vars.colors.black,
            backgroundColor: (image.props.value === sel_img) ?
              Styles.vars.colors.green :
              (det_img_topics.indexOf(image.props.value) !== -1 ) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
            cursor: "pointer",
            }}>
            <body image-topic ={image} style={{color: Styles.vars.colors.black}}>{image}</body>
        </div>
        )}
        </div>

          </Column>
          </Columns>



          <Columns>
        <Column>

          <Label title={"Select Classes"}/>

            </Column>
          <Column>

                <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                <div onClick={this.toggleClassesListViewable} style={{backgroundColor: Styles.vars.colors.grey0}}>
                      <Select style={{width: "10px"}}/>
                    </div>
                    <div hidden={this.state.classes_list_viewable === false}>
                    {classOptions.map((Class) =>
                    <div onClick={this.onToggleClassSelection}
                      style={{
                        textAlign: "center",
                        padding: `${Styles.vars.spacing.xs}`,
                        color: Styles.vars.colors.black,
                        backgroundColor: (selectedClassesList.includes(Class.props.value))? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                        cursor: "pointer",
                        }}>
                        <body class_name ={Class} style={{color: Styles.vars.colors.black}}>{Class}</body>
                    </div>
                    )}
                    </div>

          </Column>
          </Columns>





        <Columns>
        <Column>

        <Label title="Enable">
              <Toggle
              checked={detector_enabled===true}
              onClick={() => sendBoolMsg(detector_namespace + "/enable",!detector_enabled)}>
              </Toggle>
        </Label>

        </Column>
        <Column>

        </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      <Label title={"STATUS"}></Label>



              <Columns>
        <Column>

        <Label title={"Detecting"}>
                        <BooleanIndicator value={det_running} />
         </Label>

        </Column>
        <Column>

        </Column>
        </Columns>


        <Label title={"IMAGE"}></Label>

        <div style={{ display: 'flex' }}>
                      <div style={{ width: '40%' }}>

                      <Label title={"Selected"}>
                    <BooleanIndicator value={img_selected} />
                    </Label>


                      </div>

                      <div style={{ width: '20%' }}>
                        {}
                      </div>

                      <div style={{ width: '40%' }}>
                       
                          <Label title={"Connected"}>
                            <BooleanIndicator value={img_connected} />
                          </Label>
                      </div>
              </div>


    

          <Label title={"CLASSES"}></Label>

          <div style={{ display: 'flex' }}>
                      <div style={{ width: '40%' }}>

                      <Label title={"Selected"}>
                        <BooleanIndicator value={classes_selected} />
                      </Label>

                      </div>

                      <div style={{ width: '20%' }}>
                        {}
                      </div>

                      <div style={{ width: '40%' }}>
                       
                      <Label title={"Detected"}>
                        <BooleanIndicator value={detecting} />
                      </Label>

                      </div>
                    </div>



        <pre style={{ height: "100px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
        {"\n Avg Detect Rate: " + detect_process_rate +
        "\n Avg Image Process Latency: " + image_process_latency +
        "\n Avg Detect Publish Latency: " + detect_process_latency +
        "\n" +
        "\n Avg Image Process Time: " + image_process_time +
        "\n Avg Detect Process Time: " + detect_process_time +
        "\n Max Detect Rate Hz: " + max_detect_rate +
        "\n" +
        "\n Avg Image Receive Rate: " + image_receive_rate +
        "\n Avg Image Process Rate: " + image_process_rate}

      
        </pre>



        <NepiIFConfig
                        namespace={detector_namespace}
                        title={"Nepi_IF_Conig"}
        />



        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Columns>
        <Column>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {"Detector Settings"}
          </label>

          </Column>
          </Columns>

          <Columns>
        <Column>



          </Column>
          </Columns>




          

          <SliderAdjustment
                    title={"Threshold"}
                    msgType={"std_msgs/Float32"}
                    adjustment={threshold}
                    topic={detector_namespace + "/set_threshold"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    disabled={false}
                    tooltip={"Sets detection confidence threshold"}
                    unit={"%"}
                  />
                  
          <SliderAdjustment
                  title={"Max Detection Rate"}
                  msgType={"std_msgs/Float32"}
                  adjustment={max_det_rate}
                  topic={detector_namespace + "/set_max_proc_rate"}
                  scaled={1.0}
                  min={1}
                  max={20}
                  disabled={false}
                  tooltip={"Sets detection max rate in hz"}
                  unit={"Hz"}
            />

          <SliderAdjustment
                  title={"Max Image Publish Rate"}
                  msgType={"std_msgs/Float32"}
                  adjustment={max_img_rate}
                  topic={detector_namespace + "/set_max_img_rate"}
                  scaled={1.0}
                  min={1}
                  max={20}
                  disabled={false}
                  tooltip={"Sets detection max rate in hz"}
                  unit={"Hz"}
            />




            <Columns>
            <Column>

                  <Label title="Publish Image">
                  <Toggle
                  checked={pub_image_enabled===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_image_pub", pub_image_enabled===false)}>
                  </Toggle>
                  </Label>


                <div hidden={pub_image_enabled === false}>
                  <Label title="Use Last Image">
                    <Toggle
                    checked={use_last_image===true}
                    onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_use_last_image", use_last_image===false)}>
                    </Toggle>
                    </Label>

                  <Label title="Overlay Labels">
                    <Toggle
                    checked={overlay_labels===true}
                    onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_labels", overlay_labels===false)}>
                    </Toggle>
                  </Label>

                  <Label title="Overlay Range Bearing">
                    <Toggle
                    checked={overlay_range_bearing===true}
                    onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_range_bearing", overlay_range_bearing===false)}>
                    </Toggle>
                  </Label>

                  <Label title="Overlay Image Name">
                    <Toggle
                    checked={overlay_img_name===true}
                    onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_img_name", overlay_img_name===false)}>
                    </Toggle>
                    </Label>


                    <Label title="Overlay Classifier">
                    <Toggle
                    checked={overlay_detector_name===true}
                    onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_overlay_clf_name", overlay_detector_name===false)}>
                    </Toggle>
                    </Label>
                </div>

              </Column>
              <Column>

                 

          {/*
            <div hidden={has_tiling === false}>           
                <Label title="Enable Image Tiling">
                  <Toggle
                  checked={is_tiling===true}
                  onClick={() => this.props.ros.sendBoolMsg(detector_namespace + "/set_img_tiling", is_tiling===false)}>
                  </Toggle>
                  </Label>

                  </div>
          */}


            </Column>
            </Columns>



    </Column>
    </Columns>


        )
      }
    }
  }





    // Function for creating image topic for a selected detector.
    getDisplayImgOptions() {
      const { imageTopics } = this.props.ros
      var items = []
      const status_msg = this.state.status_msg
      const sel_img = this.state.selected_display_topic
    

      var img_topic = "None"
      var shortname = ''
      if (status_msg != null){
              const image_pub_topics = status_msg.image_pub_topics
              const image_names = createMenuFirstLastNames(image_pub_topics)
              if (image_pub_topics.length > 0){
                for (var i = 0; i < image_pub_topics.length; i++) {
                    if (imageTopics.indexOf(image_pub_topics[i]) !== -1) {
                      img_topic = image_pub_topics[i] 
                      shortname =  image_names[i]  
                      items.push(<Option value={img_topic}>{shortname}</Option>)
                   

                      if ((sel_img === "None" || sel_img === '') && i === 0 ){
                        
                        this.setState({selected_display_topic: img_topic, selected_display_text: shortname })
                      }
                    }
                }
              }
              else  {
                items.push(<Option value={"None"}>{"None"}</Option>)
                if (sel_img !== 'None'){
                  this.setState({selected_display_topic: "None", selected_display_text: "None" })
                }
              }
      }
      else  {
        items.push(<Option value={"None"}>{"None"}</Option>)
      }
      return items
    }


  onDisplayImgSelected(event){
    const img_topic = event.target.value
    const status_msg = this.state.status_msg
    var detector_name = 'None'
    var img_name = 'None'
    if (status_msg != null){
      detector_name = status_msg.node_name
      img_name = detector_name + img_topic.split(detector_name)[1]
    }
    this.setState({selected_display_topic: img_topic,
                   selected_display_text: img_name
    })
  }   




    // Function for creating image topic options.
    getSaveNamespace() {
      const detector_namespace = this.state.selected_namespace
      var saveNamespace = "None"
      if (detector_namespace){
        saveNamespace = detector_namespace
      }
      return saveNamespace
    }





  render() {
    const { imageTopics } = this.props.ros
    const img_options = this.getDisplayImgOptions()
    const sel_img_topic = this.state.selected_display_topic
    const img_publishning = imageTopics.indexOf(sel_img_topic) !== -1
    const sel_img = (img_publishning === true && this.state.connected === true) ? sel_img_topic : "None"
    const sel_img_text = (sel_img_topic === 'None') ? 'No Image Selected' : img_publishning?  this.state.selected_display_text : 'Waiting for image to publish'

    const save_data_topic = this.state.selected_detector + '/save_data'


    return (



      <Columns>
      <Column equalWidth={false}>


          <Columns>
          <Column>


              

                  <Label title="Select Image">
                      <Select id="ImgSelect" onChange={this.onDisplayImgSelected} 
                      value={sel_img}
                      disabled={false}>
                        {img_options}
                    </Select>
                    </Label>
          </Column>
          <Column>

          </Column>
          </Columns>


      <NepiIFImageViewer
        image_topic={sel_img}
        title={sel_img_text}
        show_res_orient={false}
        save_data_topic={save_data_topic}
      />

{/*
      { (saveNamespace !== 'None' && connected === true) ?
        <NepiIFSaveData
              saveNamespace={saveNamespace + '/save_data'}
              title={"Nepi_IF_SaveData"}
          />
      : null }
*/}
      </Column>
      <Column>


      {this.renderAiDetector()}
      


      </Column>
      </Columns>



      )
    }

  

}

export default NepiIFTracking
