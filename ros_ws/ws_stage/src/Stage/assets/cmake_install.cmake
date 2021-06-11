# Install script for directory: /home/zhou/work/codes/Stage/assets

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/assets" TYPE FILE FILES
    "/home/zhou/work/codes/Stage/assets/blue.png"
    "/home/zhou/work/codes/Stage/assets/death.png"
    "/home/zhou/work/codes/Stage/assets/green.png"
    "/home/zhou/work/codes/Stage/assets/logo.png"
    "/home/zhou/work/codes/Stage/assets/mains.png"
    "/home/zhou/work/codes/Stage/assets/mainspower.png"
    "/home/zhou/work/codes/Stage/assets/question_mark.png"
    "/home/zhou/work/codes/Stage/assets/red.png"
    "/home/zhou/work/codes/Stage/assets/stagelogo.png"
    "/home/zhou/work/codes/Stage/assets/stall-old.png"
    "/home/zhou/work/codes/Stage/assets/stall.png"
    "/home/zhou/work/codes/Stage/assets/rgb.txt"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage" TYPE FILE FILES "/home/zhou/work/codes/Stage/assets/rgb.txt")
endif()

