cmake_minimum_required(VERSION 2.8.3)
project(rs_queryanswering)

find_package(catkin REQUIRED rosjava_build_tools rosprolog designator_integration_cpp
             designator_integration_msgs knowrob_common knowrob_actions knowrob_srdl
             tf tf_conversions knowrob_vis robosherlock robosherlock_demos
             rs_addons rs_simulation roslib rospack iai_robosherlock_msgs image_transport
             json_prolog)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})
message(STATUS "SWIPL found.")

find_package(RapidJSON QUIET)
if(${RapidJSON_FOUND})
    add_definitions(-DWITH_JSON -DRAPIDJSON_HAS_STDSTRING)
endif()    

################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_queryanswering)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_queryanswering/types)
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
    INCLUDE_DIRS ${RS_INCLUDE_DIRS_LIST} ${SWIPL_INCLUDE_DIRS}
    CATKIN_DEPENDS designator_integration_msgs iai_robosherlock_msgs
                   robosherlock_demos designator_integration_cpp json_prolog
    LIBRARIES rs_designator_wrapper rs_kbAEmanagement ${SWIPL_LIBRARY_DIRS}
    CFG_EXTRAS ${PROJECT_NAME}_config.cmake
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
update_analysis_engines(robosherlock robosherlock_demos rs_addons rs_simulation)

## generate classes from the typesystem xml files
generate_type_system(robosherlock robosherlock_demos rs_addons rs_simulation)

#find all relevant files
find_additional_files()


add_library(rs_designator_wrapper SHARED 
	src/DesignatorWrapper.cpp)
add_dependencies(rs_designator_wrapper 
	designator_integration_msgs_gencpp)
target_link_libraries(rs_designator_wrapper 
	${catkin_LIBRARIES})


add_library(rs_kbAEmanagement 
	src/RSControledAnalysisEngine.cpp 
	src/RSProcessManager.cpp 
	src/PrologInterface.cpp)
target_link_libraries(rs_kbAEmanagement 
	rs_designator_wrapper 
        ${SWIPL_LIBRARIES}
	${catkin_LIBRARIES})

rs_add_executable(runAE 
	src/runAECpp.cpp)
add_dependencies(runAE 
	iai_robosherlock_msgs_gencpp)
target_link_libraries(runAE 
	rs_kbAEmanagement ${catkin_LIBRARIES})


rs_add_library(rs_prologrulescpp 
	src/PrologQueries.cpp 
	src/RSControledAnalysisEngine.cpp 
	src/RSProcessManager.cpp)
target_link_libraries(rs_prologrulescpp
	rs_designator_wrapper
	rs_kbAEmanagement
	${SWIPL_LIBRARIES}
	${catkin_LIBRARIES})

rs_add_library(rs_handleAnnotator 
	src/annotators/HandleAnnotator.cpp 
	src/annotators/HandleExtractor.cpp)
target_link_libraries(rs_handleAnnotator 
	${catkin_LIBRARIES})

add_library(rs_templateMatcher 
	src/annotators/TemplateAlignment.cpp)
target_link_libraries(rs_templateMatcher 
	${catkin_LIBRARIES})

rs_add_library(rs_templateAlignmentAnnotator 
	src/annotators/TemplateAlignmentAnnotator.cpp)
target_link_libraries(rs_templateAlignmentAnnotator 
	rs_templateMatcher 
	${catkin_LIBRARIES})

rs_add_library(rs_clusterToPartsSegmenter 
	src/annotators/ClusterToPartsSegmenter.cpp)
target_link_libraries(rs_clusterToPartsSegmenter 
	${catkin_LIBRARIES}
	${SWIPL_LIBRARIES})
