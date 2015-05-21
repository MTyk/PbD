# Install script for directory: /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so"
         RPATH "/usr/local/lib:/usr/local/lib/orocos:/usr/local/lib/orocos/xenomai/ProactiveAssistance/types:/home/intelligentrobotics/ws/orocos/orocos_toolchain/ocl/lib:/home/intelligentrobotics/ws/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/groovy/lib:/home/intelligentrobotics/ws/orocos/orocos_toolchain/install/lib:/usr/local/lib:/home/intelligentrobotics/ws/orocos/kuka_robot_hardware/kuka_lwr_fri/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/kuka_robot_hardware/lwr_fri/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_ros_comm/rtt_std_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/motion_control/rtt_motion_control_msgs/lib/orocos/xenomai/types")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types" TYPE SHARED_LIBRARY FILES "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/lib/orocos/xenomai/types/libProactiveAssistance-typekit-xenomai.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so"
         OLD_RPATH "/home/intelligentrobotics/ws/orocos/orocos_toolchain/ocl/lib:/home/intelligentrobotics/ws/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/groovy/lib:/home/intelligentrobotics/ws/orocos/orocos_toolchain/install/lib:/usr/local/lib:/home/intelligentrobotics/ws/orocos/kuka_robot_hardware/kuka_lwr_fri/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/kuka_robot_hardware/lwr_fri/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_ros_comm/rtt_std_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/motion_control/rtt_motion_control_msgs/lib/orocos/xenomai/types:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:/usr/local/lib/orocos:/usr/local/lib/orocos/xenomai/ProactiveAssistance/types:/home/intelligentrobotics/ws/orocos/orocos_toolchain/ocl/lib:/home/intelligentrobotics/ws/orocos/orocos_toolchain/log4cpp/../install/lib:/opt/ros/groovy/lib:/home/intelligentrobotics/ws/orocos/orocos_toolchain/install/lib:/usr/local/lib:/home/intelligentrobotics/ws/orocos/kuka_robot_hardware/kuka_lwr_fri/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/kuka_robot_hardware/lwr_fri/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_ros_comm/rtt_std_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/xenomai/types:/home/intelligentrobotics/ws/orocos/motion_control/rtt_motion_control_msgs/lib/orocos/xenomai/types")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/xenomai/ProactiveAssistance/types/libProactiveAssistance-typekit-xenomai.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/ProactiveAssistance" TYPE FILE FILES "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/Types.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/ProactiveAssistance/typekit" TYPE FILE FILES
    "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/Plugin.hpp"
    "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/type_info/BoostSerialization.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/ProactiveAssistance/types/ProactiveAssistance/." TYPE FILE FILES "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/../include/ProactiveAssistance/ProactiveAssistance-types.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit/ProactiveAssistance-typekit-xenomai.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orogen" TYPE FILE FILES
    "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/ProactiveAssistance.tlb"
    "/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/ProactiveAssistance.typelist"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit/transports/corba/cmake_install.cmake")
  INCLUDE("/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit/transports/mqueue/cmake_install.cmake")
  INCLUDE("/home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit/transports/typelib/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)
