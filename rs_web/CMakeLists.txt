cmake_minimum_required(VERSION 2.8.3)
project(rs_web)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  knowrob_robosherlock roscpp rospack roslib
  rosprolog
)

## System dependencies are found with CMake's conventions
set(BOOST_COMPONENTS system thread filesystem date_time)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
        set(BOOST_COMPONENTS ${BOOST_COMPONENTS} regex)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST_REGEX")
    endif()
endif()

find_package(Boost REQUIRED ${BOOST_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rs_web
#  CATKIN_DEPENDS knowrob_robosherlock rosprolog
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(SYSTEM
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(http_server src/http_examples.cpp)
target_link_libraries(http_server 
	${Boost_LIBRARIES} 
	${CMAKE_THREAD_LIBS_INIT} 
	${catkin_LIBRARIES})
