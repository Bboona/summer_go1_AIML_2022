cmake_minimum_required(VERSION 3.0.2)
project(greenball_tracking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  #  image_transport
  #  cv_bridge
  roscpp
  rospy
  std_msgs  
  message_generation
  #  camera_info_manager
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

add_executable(joystick_node src/joystick_node.cc)
target_link_libraries(joystick_node
    ${catkin_LIBRARIES}
    ${SDKLIBS}
)



