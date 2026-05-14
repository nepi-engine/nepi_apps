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
import { Route, Switch, withRouter } from "react-router-dom"
import { observer, inject } from "mobx-react"

import Page from "./Page"
import Nav from "./Nav"
import HorizontalDivider from "./HorizontalDivider"
//import PageLock from "./PageLock"


import Dashboard from "./NepiDashboard"
import NepiMgr from "./NepiSystemAdmin"
import NepiIFAppSelector from "./Nepi_IF_AppSelector"

import NepiAppTemplate from "./NepiAppTemplate"
import AiModelsMgr from "./NepiSystemAiModels"
import AiDetectorMgr from "./NepiMgrAiDetector"

//const IS_LOCAL = window.location.hostname === "localhost"

@inject("ros")
@withRouter
@observer
class MainMenuDeploy extends Component {

  componentDidMount() {
    this.props.ros.watchdogNepiCounter = 0
    this.props.ros.checkROSConnection()
  }

  render() {
    const { license_valid, license_server, license_type } = this.props.ros
    const unlicensed = (license_server !== null) && 
      (license_server.readyState === 1) && 
      (license_valid === false) 
      return (
      <Page>
        <Nav
          unlicensed={unlicensed}
          license_type={license_type}
          pages={[
            { path: "/", label: "Dashboard" },
            { path: "/pan_tilt_auto_app", label: "Application"},
            { path: "/system_selector", label: "System"},
            {
              path: "/help",
              label: "Help",
              subItems: [
                { path: "/docs", label: "Docs" },
                { path: "/tuts", label: "Tutorials" },
                { path: "/vids", label: "Videos" },
              ]
            }
          ]}
        />
        <HorizontalDivider />
        <Switch>
          <Route exact path="/" component={Dashboard} />
          <Route exact path="/admin" component={NepiMgr} />

          <Route exact path="/pan_tilt_auto_app" component={NepiAppTemplate} />

          <Route 
            path="/system_selector" 
            render={(props) => <NepiIFAppSelector {...props} app_id={'SYSTEM'} />} 
          />
        
          <Route 
            path="/ai_models_mgr" 
            render={(props) => <AiModelsMgr {...props} app_id={'AI_MODELS_MGR'} />} 
          />          
          <Route 
            path="/ai_detectors_mgr" 
            render={(props) => <AiDetectorMgr {...props} app_id={'AI_DETECTORS_MGR'} />} 
          />        

          <Route path='/docs' component={() => {
             window.location.href = 'https://nepi.com/documentation/';
             return null;
            }}/>
          <Route path='/tuts' component={() => {
             window.location.href = 'https://nepi.com/tutorials/';
             return null;
            }}/>
          <Route path='/vids' component={() => {
             window.location.href = 'https://nepi.com/videos/';
             return null;
            }}/>
          
        </Switch>
      </Page>
    )
  }
}

export default MainMenuDeploy
