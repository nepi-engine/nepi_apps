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

//import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"
import Toggle from "react-toggle"

import ImageViewer from "./Nepi_IF_ImageViewer"
import {createShortImagesFromNamespaces} from "./Utilities"

import NepiIFSaveData from "./Nepi_IF_SaveData"

@inject("ros")
@observer

// MultiImageViewer Application page
class ImageViewerApp extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_image_viewer",
      appNamespace: null,

      selectedImageTopics: ['None','None','None','None'],
      statusListener: null,
      connected: false,
      needs_update: false,
      showFullscreen: false
    }
    this.onClickToggleShowFullscreen = this.onClickToggleShowFullscreen.bind(this)

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)

    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)
    this.onChangeInputImgSelection = this.onChangeInputImgSelection.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

    this.getSelectedImageTopics = this.getSelectedImageTopics.bind(this)
  }


  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      selectedImageTopics: message.entries
  })
    this.setState({
      connected: true
    })


  }

    // Function for configuring and subscribing to Status
    updateStatusListener() {
      const statusNamespace = this.state.appNamespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_interfaces/StringArray",
            this.statusListener
          )
      this.setState({ 
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
    const namespace_updated = (prevState.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1){
        this.setState({appNamespace: namespace})
        this.updateStatusListener()
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




  // Function for creating image topic options.
  createImageTopicsOptions() {
    var items = []
    items.push(<Option>{"None"}</Option>) 
    const { imageTopics } = this.props.ros
    const baseNamespace = this.getBaseNamespace()
    var imageTopicShortnames = createShortImagesFromNamespaces(baseNamespace, imageTopics)
    for (var i = 0; i < imageTopics.length; i++) {
      items.push(<Option value={imageTopics[i]}>{imageTopicShortnames[i]}</Option>)
    }
    return items
  }

  onChangeInputImgSelection(event) {
    const {sendImageSelectionMsg} = this.props.ros
    var imageTopics = this.state.selectedImageTopics
    const namespace = this.getAppNamespace() 
    const selNamespace = namespace + "/set_topic"
    const value = event.target.value
    if (namespace !== null){    
      var selector_img = 0
      if (event.nativeEvent.target.id === "ImageSelector_1") {
        selector_img = 1
      }
      else if (event.nativeEvent.target.id === "ImageSelector_2") {
        selector_img = 2
      }
      else if (event.nativeEvent.target.id === "ImageSelector_3") {
        selector_img = 3
      }

      imageTopics[selector_img] = value
      sendImageSelectionMsg(selNamespace,selector_img,value)
    }
    this.setState({selectedImageTopics: imageTopics})
  }

  getSelectedImageTopics(){
    const imageTopics = this.state.selectedImageTopics
    return imageTopics
  }

  onClickToggleShowFullscreen() {
    this.setState({ showFullscreen: !this.state.showFullscreen });
  }

  render() {
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const {sendTriggerMsg} = this.props.ros
    const imageOptions = this.createImageTopicsOptions()
    const selectedImageTopics = this.getSelectedImageTopics()
    const { namespacePrefix, deviceId} = this.props.ros
    const baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    const selectedImageText = createShortImagesFromNamespaces(baseNamespace, selectedImageTopics)
    const appNamespace = this.getAppNamespace()
    const colCount = ((selectedImageTopics[1] !== 'None') || (selectedImageTopics[2] !== 'None') || (selectedImageTopics[3] !== 'None'))? 2 : 1
    const colFlexSize_1 = (colCount === 1)? "100%" : "50%"
    const colFlexSize_2 = (colCount === 1)? "0%" : "50%"
    const flexSize = this.state.showFullscreen === true ? ['100%','0%','0%'] : ['70%','2%','28%']
    
    
    return (
    <div>
      <div style={{ width: "10%" }}>
        <div style={{ marginBottom: "10px" }}>
          <Label title="fullscreen"> 
            <Toggle
              checked={this.state.showFullscreen}
              onClick={this.onClickToggleShowFullscreen}>
            </Toggle>
          </Label>
        </div>
      </div>

      <div style={{ display: 'flex' }}>

           <div style={{ width: flexSize[0] }}>
     
                <div style={{ display: 'flex' }}>

                  <div style={{ width: colFlexSize_1 }}>

                              <ImageViewer
                                imageTopic={selectedImageTopics[0]}
                                show_controls={false}
                                title={selectedImageText[0]}
                                hideQualitySelector={true}
                                squareCanvas={true}
                              />
                              {(selectedImageTopics[2] !== 'None')?
                                <ImageViewer
                                imageTopic={selectedImageTopics[2]}
                                show_controls={false}
                                title={selectedImageText[2]}
                                hideQualitySelector={true}
                                squareCanvas={true}
                              />          
                              : null
                              }

                        </div>

  

                        <div style={{ width: colFlexSize_2 }}>

                              {(selectedImageTopics[1] !== 'None' || selectedImageTopics[3] !== 'None' )?
                                <ImageViewer
                                  imageTopic={selectedImageTopics[1]}
                                  show_controls={false}
                                  title={selectedImageText[1]}
                                  hideQualitySelector={true}
                                  squareCanvas={true}
                                />          
                              : null
                              }

                              {(selectedImageTopics[3] !== 'None')?
                                <ImageViewer
                                  imageTopic={selectedImageTopics[3]}
                                  show_controls={false}
                                  title={selectedImageText[3]}
                                  hideQualitySelector={true}
                                  squareCanvas={true}
                                />          
                              : null
                              }

                        </div>


                </div>


          </div>

          <div style={{ width: flexSize[1] }}>
          </div>

          <div hidden={this.state.showFullscreen}>

          <div style={{width: flexSize[2]}}>


                     <Label title={"Img 1"} div style={{display:"flex", flexWrap:"wrap", justifyContent:"space-between"}}>
                      <Select onChange={this.onChangeInputImgSelection} 
                        id="ImageSelector_0"
                        value={selectedImageTopics[0]}>                       
                        {imageOptions}
                      </Select>
                    </Label>
                    <Label title={"Img 2"} style={{ marginRight: '15px', minWidth: '200px' }}>
                      <Select onChange={this.onChangeInputImgSelection} 
                          id="ImageSelector_1"
                          value={selectedImageTopics[1]}>
                          {imageOptions}
                      </Select>
                    </Label>
                    <Label title={"Img 3"} style={{ marginRight: '20px', minWidth: '150px' }}>
                      <Select onChange={this.onChangeInputImgSelection} 
                        id="ImageSelector_2"
                        value={selectedImageTopics[2]}>
                        {imageOptions}
                      </Select>
                    </Label>
                    <Label title={"Img 4"} style={{ marginRight: '25px', minWidth: '60px' }}>
                      <Select onChange={this.onChangeInputImgSelection} 
                        id="ImageSelector_3"
                        value={selectedImageTopics[3]}>
                        {imageOptions}
                      </Select>
                    </Label>

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


                    <div hidden={appNamespace === null}>
                    <NepiIFSaveData
                    saveNamespace={appNamespace}
                    title={"Nepi_IF_SaveData"}
                    />
                    </div>


          </div>
      
      </div>
      </div>


    )
  }


}

export default ImageViewerApp
