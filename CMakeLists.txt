cmake_minimum_required(VERSION 2.8.3)
project(rs_kbreasoning)


find_package(catkin REQUIRED rosjava_build_tools rosprolog designator_integration_cpp
             designator_integration_msgs knowrob_common knowrob_actions knowrob_srdl
             knowrob_vis robosherlock json_prolog robosherlock_demos rs_addons roslib rospack iai_robosherlock_msgs)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
install(DIRECTORY prolog/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog})
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})
message("SWIPL" ${SWIPL_LIBRARIES})
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_kbreasoning)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_kbreasoning/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)


################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)

################################################################################
## Find all include directories                                               ##
################################################################################

find_include_dirs(RS_INCLUDE_DIRS_LIST)


catkin_package(
    DEPENDS designator_integration_msgs iai_robosherlock_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )

################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################

##Update xml list of annotators inside analysis engines
update_analysis_engines(robosherlock robosherlock_demos rs_addons)

## generate classes from the typesystem xml files
generate_type_system(robosherlock robosherlock_demos rs_addons)

#find all relevant files
find_additional_files()


add_library(rs_designator_wrapper SHARED src/DesignatorWrapper.cpp)
add_dependencies(rs_designator_wrapper designator_integration_msgs_gencpp)
target_link_libraries(rs_designator_wrapper ${catkin_LIBRARIES})

rs_add_library(rs_handleAnnotator src/annotators/HandleAnnotator.cpp src/annotators/handle_extractor.cpp)
target_link_libraries(rs_handleAnnotator ${catkin_LIBRARIES})

rs_add_executable(runAE src/runAECpp.cpp src/RSControledAnalysisEngine.cpp)
add_dependencies(runAE iai_robosherlock_msgs_gencpp)
target_link_libraries(runAE rs_designator_wrapper ${catkin_LIBRARIES})

rs_add_library(rs_kbResultAdvertiser src/ResultAdvertiser.cpp)
target_link_libraries(rs_kbResultAdvertiser rs_designator_wrapper ${catkin_LIBRARIES})

rs_add_library(rs_clusterToPartsSegmenter src/ClusterToPartsSegmenter.cpp)
target_link_libraries(rs_clusterToPartsSegmenter ${catkin_LIBRARIES})


rs_add_library(rs_prologrulescpp src/PrologQueries.cpp src/RSControledAnalysisEngine.cpp)
target_link_libraries(rs_prologrulescpp
  rs_designator_wrapper
  ${SWIPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
