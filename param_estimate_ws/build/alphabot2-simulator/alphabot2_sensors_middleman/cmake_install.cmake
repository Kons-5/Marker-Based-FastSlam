# Install script for directory: /home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/src/alphabot2-simulator/alphabot2_sensors_middleman

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alphabot2_sensors_middleman/cmake" TYPE FILE FILES "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/build/alphabot2-simulator/alphabot2_sensors_middleman/catkin_generated/installspace/alphabot2_sensors_middleman-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/devel/share/roseus/ros/alphabot2_sensors_middleman")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/devel/lib/python3/dist-packages/alphabot2_sensors_middleman")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/devel/lib/python3/dist-packages/alphabot2_sensors_middleman")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/build/alphabot2-simulator/alphabot2_sensors_middleman/catkin_generated/installspace/alphabot2_sensors_middleman.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alphabot2_sensors_middleman/cmake" TYPE FILE FILES "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/build/alphabot2-simulator/alphabot2_sensors_middleman/catkin_generated/installspace/alphabot2_sensors_middleman-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alphabot2_sensors_middleman/cmake" TYPE FILE FILES
    "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/build/alphabot2-simulator/alphabot2_sensors_middleman/catkin_generated/installspace/alphabot2_sensors_middlemanConfig.cmake"
    "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/build/alphabot2-simulator/alphabot2_sensors_middleman/catkin_generated/installspace/alphabot2_sensors_middlemanConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alphabot2_sensors_middleman" TYPE FILE FILES "/home/moth/Documents/Marker-Based-FastSlam/param_estimate_ws/src/alphabot2-simulator/alphabot2_sensors_middleman/package.xml")
endif()

