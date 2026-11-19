#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
#
# License: NEPI Applications repo source-code and NEPI Images that use this source-code
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
cmake_minimum_required(VERSION 3.5)
project(nepi_app_nav_sim)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(FILES
  NepiAppNmeaSimStatus.msg
  NepiAppHNavSimStatus.msg
  NepiAppGpsSimStatus.msg
  NepiAppNavSimMasterStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/nav_sim_app_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY etc/
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(NOT NEPI_BASE)
  set(NEPI_BASE /opt/nepi)
endif()

if(NOT NEPI_RUI)
  set(NEPI_RUI ${NEPI_BASE}/nepi_rui)
endif()

if(NOT NEPI_ENGINE)
  set(NEPI_ENGINE ${NEPI_BASE}/nepi_engine)
endif()

set(NEPI_APP_PARAMS_FOLDER ${NEPI_ENGINE}/share/nepi_apps/params)
install(DIRECTORY params/ DESTINATION ${NEPI_APP_PARAMS_FOLDER})

set(NEPI_API_FOLDER ${NEPI_ENGINE}/lib/python3/dist-packages/nepi_api)
install(DIRECTORY api/
  DESTINATION ${NEPI_API_FOLDER}
  FILES_MATCHING PATTERN "*.py"
)

set(NEPI_RUI_PARAMS_FOLDER ${NEPI_RUI}/src/rui_webserver/rui-app/src/apps)
install(DIRECTORY params/ DESTINATION ${NEPI_RUI_PARAMS_FOLDER})

set(NEPI_RUI_SOURCE_FOLDER ${NEPI_RUI}/src/rui_webserver/rui-app/src)
install(DIRECTORY rui/ DESTINATION ${NEPI_RUI_SOURCE_FOLDER})
