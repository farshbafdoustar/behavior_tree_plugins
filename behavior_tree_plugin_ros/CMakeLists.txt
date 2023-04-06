cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_plugin_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(ROS_DEPENDENCIES
    roscpp
    behaviortree_cpp
    std_msgs
    actionlib_msgs
    actionlib
    message_serialization
    ros_babel_fish)


find_package(catkin REQUIRED COMPONENTS 
  ${ROS_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS    ${ROS_DEPENDENCIES}
)
#############
## Build ##
#############

include_directories(
 include
  ${catkin_INCLUDE_DIRS}

)


add_library(${PROJECT_NAME}
  src/utilities/tree_node_manager.cpp
  src/call_any_service.cpp
  src/compose_any_type_from_input_ports.cpp
  src/decompose_any_type_to_output_ports.cpp
  src/index_array.cpp
  src/publish_any_topic.cpp
  src/ros_wrapper.cpp
  src/subscribe_any_topic.cpp

)


add_dependencies(${PROJECT_NAME}
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED COMPONENTS)
  set (CMAKE_CXX_STANDARD 11)
  include_directories(${catkin_INCLUDE_DIRS} include )

 # add_rostest_gtest(plan_joints_trajectory_test
      # test file should be defined on the top otherwise the implementation in src/*.cpp is not found during link
 #    test/sth_unittest_launch.test
 #    test/sth_unittest.cpp
 #    src/utilities/sth.cpp
 #    src/sth.cpp
 #   )

 #target_link_libraries(plan_joints_trajectory_test gtest gmock gtest_main gmock_main ${catkin_LIBRARIES} )

endif()
