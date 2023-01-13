cmake_minimum_required(VERSION 3.0.2)
project(greenball_tracking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  
  message_generation
  geometry_msgs
  unitree_legged_msgs
)

catkin_package(
    #CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
    CATKIN_DEPENDS roscpp rospy std_msgs
)

find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_INCLUDE_DIR})
    message(STATUS ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

set(SDKLIBS paho-mqttpp3 paho-mqtt3as unitree_camera tstc_V4L2_xu_camera udev systemlog unitree_legged_sdk ${OpenCV_LIBS})

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
link_directories(${PROJECT_SOURCE_DIR}/lib/amd64/)
else()
link_directories(${PROJECT_SOURCE_DIR}/lib/arm64/)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(greenball_tracking src/green_ball_node.cc)
target_link_libraries(greenball_tracking
    ${catkin_LIBRARIES}
    ${SDKLIBS}
)

add_executable(movement_node src/movement_node.cc)
target_link_libraries(movement_node
    ${catkin_LIBRARIES}
    ${SDKLIBS}
)

add_executable(ros_udp src/ros_udp.cpp)
target_link_libraries(ros_udp
    ${catkin_LIBRARIES}
    ${SDKLIBS}
)

add_executable(joystick_sub src/joystick_sub.cpp)
target_link_libraries(joystick_sub
    ${catkin_LIBRARIES}
    ${SDKLIBS}
)



