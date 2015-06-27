# Generated from orogen/lib/orogen/templates/typekit/CMakeLists.txt

##
# Header for typegen-generated typekits
#

project("ProactiveAssistance-typekit")
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")

# In Orogen components, the build target is specified at generation time
# But we allow to override in case of typegen/USE_OROCOS_RTT:
if (NOT OROCOS_TARGET)
  set(OROCOS_TARGET "xenomai")
endif()
# If UseOrocos-RTT.cmake is used, install libs in /target/package subdir in order to allow
# multi-target/package installs.
if ( USE_OROCOS_RTT )
  if ( NOT OROCOS_SUFFIX )
    set (OROCOS_SUFFIX "/${OROCOS_TARGET}")
  endif()
  set(OROCOS_PKG_DIR "${OROCOS_SUFFIX}/ProactiveAssistance")
endif()

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall", CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif (CXX_SUPPORTS_WALL)

# Define the necessary RPath information for all binaries. That can be stripped
# later
option(WITH_RPATH "enables or disables embedding RPath information in binaries" ON)
if(WITH_RPATH)
    include(RPATHHandling)
    CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/orocos:${CMAKE_INSTALL_PREFIX}/lib/orocos${OROCOS_PKG_DIR}/types")
endif(WITH_RPATH)

# Set the build type to debug by default
IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug )
ENDIF ( NOT CMAKE_BUILD_TYPE )

include(OrogenPkgCheckModules)
orogen_pkg_check_modules(OrocosRTT REQUIRED "orocos-rtt-${OROCOS_TARGET}>=1.99.4")

ADD_CUSTOM_TARGET(regen
    
    typegen --output /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit ProactiveAssistance include/ProactiveAssistance/ProactiveAssistance-types.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)


# END if typekit.standalone?
##

# Manually define '-DIS_ROS_PACKAGE' or include the UseOrocos.cmake file to automatically detect&set this flag.
if ( IS_ROS_PACKAGE )
    # When we import a typekit in a ROS package, we expect it to be in the package/lib directory
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX}/types)
endif( IS_ROS_PACKAGE ) 

execute_process(COMMAND cmake -E create_symlink
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/ProactiveAssistance)

include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/types)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OrocosRTT_INCLUDE_DIRS})
add_definitions(${OrocosRTT_CFLAGS_OTHER})
link_directories(${OrocosRTT_LIBRARY_DIRS})



# Now set up the dependencies

set(PKG_CFLAGS ${OrocosRTT_CFLAGS})

string(REPLACE ";" "\" \"" PKG_CFLAGS "\"${PKG_CFLAGS}\"")

# Generate the base typekit shared library
set(libname ProactiveAssistance-typekit-${OROCOS_TARGET})
add_library(${libname} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/Plugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/type_info/ProactiveAssistanceData___std__string.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/type_info/__std__vector__double__.cpp
    ${TYPEKIT_ADDITIONAL_SOURCES}
    ${TOOLKIT_ADDITIONAL_SOURCES})


target_link_libraries(${libname} ${OrocosRTT_LIBRARIES} ${TYPEKIT_ADDITIONAL_LIBRARIES} ${TOOLKIT_ADDITIONAL_LIBRARIES})
target_link_libraries(${libname} LINK_INTERFACE_LIBRARIES ${OrocosRTT_LIBRARIES})

set(PKG_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/ProactiveAssistance-typekit-${OROCOS_TARGET}.pc)
configure_file(ProactiveAssistance-typekit.pc.in ${PKG_CONFIG_FILE} @ONLY)

install(TARGETS ${libname} LIBRARY DESTINATION lib/orocos${OROCOS_PKG_DIR}/types)
install(FILES
    Types.hpp
    
    DESTINATION include/orocos/ProactiveAssistance)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Plugin.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/type_info/BoostSerialization.hpp
    DESTINATION include/orocos/ProactiveAssistance/typekit)


install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../include/ProactiveAssistance/ProactiveAssistance-types.hpp
    DESTINATION include/orocos/ProactiveAssistance/types/ProactiveAssistance/.)

install(FILES ${PKG_CONFIG_FILE}
    DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ProactiveAssistance.tlb
    ${CMAKE_CURRENT_SOURCE_DIR}/ProactiveAssistance.typelist
    DESTINATION share/orogen)


    add_subdirectory(transports/corba)
    
    add_subdirectory(transports/mqueue)
    
    add_subdirectory(transports/typelib)
    

# Force the user to regenerate its typekit if the inputs changed
set(TK_STAMP "${CMAKE_CURRENT_SOURCE_DIR}/stamp")
add_custom_command(
    OUTPUT "${TK_STAMP}"
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../include/ProactiveAssistance/ProactiveAssistance-types.hpp 
    COMMENT "Typekit input changed. Run make regen in your build directory first"
    COMMAND /bin/false)
add_custom_target(check-typekit-uptodate ALL DEPENDS "${TK_STAMP}")
add_dependencies(${libname} check-typekit-uptodate)

