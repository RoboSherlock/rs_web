cmake_minimum_required(VERSION 2.8.3)
project(knowrob_robosherlock)


find_package(catkin REQUIRED rosjava_build_tools rosprolog
             designator_integration_msgs knowrob_common knowrob_actions knowrob_srdl
             tf tf_conversions knowrob_vis json_prolog iai_robosherlock_msgs)

#catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

#install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
#        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
#install(DIRECTORY prolog/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog})


find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})
message(STATUS "SWIPL found.")

find_package(RapidJSON QUIET)
if(${RapidJSON_FOUND})
    add_definitions(-DWITH_JSON -DRAPIDJSON_HAS_STDSTRING)
endif()    



catkin_package(
    DEPENDS designator_integration_msgs iai_robosherlock_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        )
