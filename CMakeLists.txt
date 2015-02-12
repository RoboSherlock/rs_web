
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_robosherlock)

find_package(catkin REQUIRED rosjava_build_tools rosprolog designator_integration_msgs knowrob_common knowrob_actions knowrob_srdl)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
    DEPENDS designator_integration_msgs 
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
