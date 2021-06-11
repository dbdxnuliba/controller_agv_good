# Install script for directory: /home/zhou/work/codes/Stage/worlds

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/worlds" TYPE FILE FILES
    "/home/zhou/work/codes/Stage/worlds/amcl-sonar.cfg"
    "/home/zhou/work/codes/Stage/worlds/autolab.cfg"
    "/home/zhou/work/codes/Stage/worlds/camera.cfg"
    "/home/zhou/work/codes/Stage/worlds/everything.cfg"
    "/home/zhou/work/codes/Stage/worlds/lsp_test.cfg"
    "/home/zhou/work/codes/Stage/worlds/mbicp.cfg"
    "/home/zhou/work/codes/Stage/worlds/nd.cfg"
    "/home/zhou/work/codes/Stage/worlds/roomba.cfg"
    "/home/zhou/work/codes/Stage/worlds/simple.cfg"
    "/home/zhou/work/codes/Stage/worlds/test.cfg"
    "/home/zhou/work/codes/Stage/worlds/uoa_robotics_lab.cfg"
    "/home/zhou/work/codes/Stage/worlds/vfh.cfg"
    "/home/zhou/work/codes/Stage/worlds/wavefront-remote.cfg"
    "/home/zhou/work/codes/Stage/worlds/wavefront.cfg"
    "/home/zhou/work/codes/Stage/worlds/wifi.cfg"
    "/home/zhou/work/codes/Stage/worlds/SFU.world"
    "/home/zhou/work/codes/Stage/worlds/autolab.world"
    "/home/zhou/work/codes/Stage/worlds/camera.world"
    "/home/zhou/work/codes/Stage/worlds/circuit.world"
    "/home/zhou/work/codes/Stage/worlds/everything.world"
    "/home/zhou/work/codes/Stage/worlds/fasr.world"
    "/home/zhou/work/codes/Stage/worlds/fasr2.world"
    "/home/zhou/work/codes/Stage/worlds/fasr_plan.world"
    "/home/zhou/work/codes/Stage/worlds/large.world"
    "/home/zhou/work/codes/Stage/worlds/lsp_test.world"
    "/home/zhou/work/codes/Stage/worlds/mbicp.world"
    "/home/zhou/work/codes/Stage/worlds/pioneer_flocking.world"
    "/home/zhou/work/codes/Stage/worlds/pioneer_follow.world"
    "/home/zhou/work/codes/Stage/worlds/pioneer_walle.world"
    "/home/zhou/work/codes/Stage/worlds/roomba.world"
    "/home/zhou/work/codes/Stage/worlds/sensor_noise_demo.world"
    "/home/zhou/work/codes/Stage/worlds/sensor_noise_module_demo.world"
    "/home/zhou/work/codes/Stage/worlds/simple.world"
    "/home/zhou/work/codes/Stage/worlds/uoa_robotics_lab.world"
    "/home/zhou/work/codes/Stage/worlds/wifi.world"
    "/home/zhou/work/codes/Stage/worlds/beacons.inc"
    "/home/zhou/work/codes/Stage/worlds/chatterbox.inc"
    "/home/zhou/work/codes/Stage/worlds/hokuyo.inc"
    "/home/zhou/work/codes/Stage/worlds/irobot.inc"
    "/home/zhou/work/codes/Stage/worlds/map.inc"
    "/home/zhou/work/codes/Stage/worlds/objects.inc"
    "/home/zhou/work/codes/Stage/worlds/pantilt.inc"
    "/home/zhou/work/codes/Stage/worlds/pioneer.inc"
    "/home/zhou/work/codes/Stage/worlds/sick.inc"
    "/home/zhou/work/codes/Stage/worlds/ubot.inc"
    "/home/zhou/work/codes/Stage/worlds/uoa_robotics_lab_models.inc"
    "/home/zhou/work/codes/Stage/worlds/walle.inc"
    "/home/zhou/work/codes/Stage/worlds/cfggen.sh"
    "/home/zhou/work/codes/Stage/worlds/test.sh"
    "/home/zhou/work/codes/Stage/worlds/worldgen.sh"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zhou/work/codes/Stage/worlds/benchmark/cmake_install.cmake")
  include("/home/zhou/work/codes/Stage/worlds/bitmaps/cmake_install.cmake")
  include("/home/zhou/work/codes/Stage/worlds/wifi/cmake_install.cmake")

endif()

