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
import Styles from "./Styles"

import NepiIFImageViewersSelector from "./Nepi_IF_ImageViewersSelector"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

import {createMenuFirstLastNames} from "./Utilities"

// Leaf of the app's ControlsIF namespace. ControlsIF roots itself at
// create_namespace(node_namespace, controls_name), so the control set sits one
// level BELOW the app node namespace. Appending '/controls' here is what makes
// Nepi_IF_Controls subscribe to .../app_image_viewer/controls/status and publish
// its updates to .../app_image_viewer/controls/update_control.
const CONTROLS_NAME = "controls"

@inject("ros")
@observer

// MultiImageViewer Application page
class ImageViewerApp extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_image_viewer",
      appNamespace: null,

      selected_image_topics: ['None','None','None','None'],
      num_windows: 1,
      statusListener: null,
      connected: false,
      needs_update: false,

      show_selectors: false
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAllSaveNamespace = this.getAllSaveNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)

    this.renderImageViewers = this.renderImageViewers.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
    this.renderConfig = this.renderConfig.bind(this)

    
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

  }


  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }

    getAllSaveNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + '/save_data'
    }
    return allNamespace
  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  getControlsNamespace(){
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null){
      return appNamespace + "/" + CONTROLS_NAME
    }
    return null
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      selected_image_topics: message.selected_image_topics,
      num_windows: message.num_windows
  })
    this.setState({
      connected: true
    })


  }

    // Function for configuring and subscribing to Status
    updateStatusListener(namespace) {
      const statusNamespace = namespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_image_viewer/NepiAppImageViewerStatus",
            this.statusListener
          )
      this.setState({ 
        appNamespace: namespace,
        statusListener: statusListener,
      })
    }

    componentDidMount(){
      this.setState({needs_update: true})
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1){
        this.updateStatusListener(namespace)
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



  renderImageViewers() {
     if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const selected_image_topics = this.state.selected_image_topics

    //Unused const baseNamespace = "/" + namespacePrefix + "/" + deviceId
    const topics_text = createMenuFirstLastNames(selected_image_topics)
    const image_filters = ['/all/']
    const appNamespace = this.getAppNamespace()
    const num_windows_updated_topic = appNamespace + '/set_num_windows'
    const select_updated_topics = [
        appNamespace + '/set_topic_1',
        appNamespace + '/set_topic_2',
        appNamespace + '/set_topic_3',
        appNamespace + '/set_topic_4'
    ]

    // const mouse_event_topics = [
        // appNamespace + '/set_click_pixel',
        // appNamespace + '/set_click_pixel',
        // appNamespace + '/set_click_pixel',
        // appNamespace + '/set_click_pixel'
    // ]

      return (
     

      <React.Fragment>


                          <div id="imageviewers">
                            <NepiIFImageViewersSelector
                              id="imageviewers"
                              image_topics={selected_image_topics}
                              titles={topics_text}
                              show_save_controls={false}
                              show_image_controls={true}
                              num_windows_updated_topic={num_windows_updated_topic}
                              auto_select_image={false}
                              select_updated_topics={select_updated_topics}
                              //mouse_event_topics={mouse_event_topics}
                              image_filters={image_filters}
                              make_section={false}
                            />
                          </div>        
 
      </React.Fragment>

      )
  }



  // The app's control set: window count and the four topic selections.
  //
  // NOT CURRENTLY MOUNTED -- both call sites in render() are commented out, on
  // purpose and kept rather than deleted. Every value in this set is already
  // reachable from Nepi_IF_ImageViewersSelector's own control bar above, so
  // rendering this section too put the same five values on the page twice.
  //
  // The backend set is untouched and still live: the node routes the selector's
  // set_topic_N / set_num_windows topics through this same control set, so the
  // controls are doing their job -- they just have no second section of their
  // own here. Uncomment either call site to bring the section back.
  renderControls(){
    const controlsNamespace = this.getControlsNamespace()
    if (controlsNamespace === null || controlsNamespace.indexOf('null') !== -1){
      return null
    }
    return (

      <React.Fragment>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            <Label title={"Image Viewer Controls"} />

              <NepiIFControls
                key={controlsNamespace}
                namespace={controlsNamespace}
                title={null}
                make_section={false}
                allways_show_controls={true}
              />

      </React.Fragment>

      )
  }


  renderSaveData(){
      const allSaveNamespace = this.getAllSaveNamespace()
      return (
    
      <React.Fragment>

                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  <NepiIFSaveData
                    saveNamespace={allSaveNamespace}
                    make_section={false}
                    show_all_options={true}
                    show_topic_selector={true}
                  />
 
      </React.Fragment>

      )
  }

  renderConfig(){
      const appNamespace = this.getAppNamespace()
      return (
    
      <React.Fragment>

             <NepiIFConfig
                  namespace={appNamespace}
                  title={"Nepi_IF_Config"}
              />
 
      </React.Fragment>

      )
  }



  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    

    if (make_section === false){
      return (
        <Columns>
        <Column>
              {this.renderImageViewers()}
                {/* {this.renderControls()} */}
                {/* {this.renderSaveData()} */}
                  {this.renderConfig()}


        </Column>
        </Columns>
      )
    }
    else {
      return (

      <Section>

              {this.renderImageViewers()}
                {/* {this.renderControls()} */}
                {/* {this.renderSaveData()} */}
                  {this.renderConfig()}

      </Section>
      )

    }
  }


}

export default ImageViewerApp
