cmake_minimum_required(VERSION 2.8.3)
project(rs_kbreasoning)


find_package(json_prolog QUIET)
 if(json_prolog_FOUND)
    message("OPTIONAL MODULE json_prolog FOUND")
    set(OPTIONAL_json json_prolog)
    add_definitions( -DJSON_PROLOG_FOUND)
 endif(json_prolog_FOUND)

find_package(catkin REQUIRED rosjava_build_tools rosprolog designator_integration_cpp
             designator_integration_msgs knowrob_common knowrob_actions knowrob_srdl
             knowrob_vis robosherlock ${OPTIONAL_json})

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

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
    DEPENDS designator_integration_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )

################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################

##Update xml list of annotators inside analysis engines
update_analysis_engines(robosherlock)

## generate classes from the typesystem xml files
generate_type_system(robosherlock)

#find all relevant files
find_additional_files()


add_library(rs_designator_wrapper SHARED src/DesignatorWrapper.cpp)
add_dependencies(rs_designator_wrapper designator_integration_msgs_gencpp)
target_link_libraries(rs_designator_wrapper designator_integration rs_core)

rs_add_executable(runAE src/runAECpp.cpp src/RSPipelineManager.cpp)
#set_target_properties(rs_runAE PROPERTIES COMPILE_DEFINITIONS RS_PATH="${${RS_CMAKE_PROJECT_NAME}_SOURCE_DIR}")
target_link_libraries(runAE rs_core rs_io rs_designator_wrapper ${catkin_LIBRARIES})
