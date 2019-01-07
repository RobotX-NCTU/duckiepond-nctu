cmake_minimum_required(VERSION 2.8.3)
project(moos_behavior)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostopic
  std_msgs
  duckiepond
)

find_package(MOOS REQUIRED)
find_package(Boost 1.58 COMPONENTS thread system REQUIRED)

catkin_package(

)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
# include
  ${MOOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${MOOS_LIBRARY_DIRS}
)

#link_directories(${MOOS_LIBRARY_DIRS})
set(LIBS ${LIBS} ${MOOS_LIBRARIES})

INCLUDE(FindPackageMessage) 

message("----------------------------------------------")
if(MOOS_FOUND)
  message("MOOS FOUND!")
else(MOOS_FOUND)
  message("MOOS NOT FOUND")
endif(MOOS_FOUND)
message("----------------------------------------------")

add_executable( Bridge 
  include/MsgContainer.h 
  include/MOOSNode.cpp 
  include/Bridge.cpp
  )

target_link_libraries(Bridge ${catkin_LIBRARIES})
target_link_libraries(Bridge MOOS)
target_link_libraries(Bridge ${Boost_LIBRARIES})