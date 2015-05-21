# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

# This policy indicates that the code below is cmake 2.6 compliant. This is necessary
# To properly escape the add_definitions arguments.
cmake_policy(VERSION 2.6)




orogen_pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})

orogen_pkg_check_modules(RTT_Typelib REQUIRED rtt_typelib-${OROCOS_TARGET})
include_directories(${RTT_Typelib_INCLUDE_DIRS})
link_directories(${RTT_Typelib_LIBRARY_DIRS})

set(TYPEKIT_REGISTRY ${CMAKE_INSTALL_PREFIX}/share/orogen/ProactiveAssistance.tlb)
add_definitions(-DTYPEKIT_REGISTRY="${TYPEKIT_REGISTRY}")
set(libname_typelib ProactiveAssistance-transport-typelib-${OROCOS_TARGET})
add_library(${libname_typelib} SHARED
    /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/transports/typelib/ProactiveAssistanceData___std__string.cpp
    /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/transports/typelib/__std__vector__double__.cpp
    /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit/transports/typelib/TransportPlugin.cpp)

target_link_libraries(${libname_typelib}
    ${TYPELIB_LIBRARIES}
    ${OrocosRTT_LIBRARIES}
    ${RTT_Typelib_LIBRARIES}
    )



# rospack support: if we're a ROS package, use rospack to return the location of our .tlb file.
if ( IS_ROS_PACKAGE )
      set(ROS_ROOT $ENV{ROS_ROOT})
      # Catkin style of fetching ROS deps
      FIND_PACKAGE( ROS QUIET COMPONENTS roslib )

      IF(NOT ROS_FOUND) # pre-Fuerte or Groovy
        # TODO: This block to get roslib is deprecated as of ROS Fuerte, but is left in for pre-Fuerte compatibility
        set(roslib_PACKAGE_PATH ${ROS_ROOT}/core/roslib) ### XXX hardcoded
        message("Falling back to roslib in: ${roslib_PACKAGE_PATH}")
        find_library(ROS_LIBRARIES roslib ${roslib_PACKAGE_PATH}/lib )
        if ( NOT ROS_LIBRARIES )
          find_package( ROS COMPONENTS roslib ) # Yells at user (non-QUIET !)
        else()
          set(ROS_FOUND TRUE)
          set(ROS_INCLUDE_DIRS ${roslib_PACKAGE_PATH}/include)
        endif()
      ENDIF(NOT ROS_FOUND)
      
      IF(ROS_FOUND)
        include_directories( ${ROS_INCLUDE_DIRS} )
        add_definitions( -DHAS_ROSLIB )
	target_link_libraries( ${libname_typelib} ${ROS_LIBRARIES} )
      ENDIF(ROS_FOUND)

endif( IS_ROS_PACKAGE )

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_BINARY_DIR}/ProactiveAssistance-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(ProactiveAssistance-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(FILES
    TransportPlugin.hpp
    DESTINATION include/orocos/ProactiveAssistance/transports/typelib)
install(TARGETS ${libname_typelib} LIBRARY DESTINATION lib/orocos${OROCOS_PKG_DIR}/types)

