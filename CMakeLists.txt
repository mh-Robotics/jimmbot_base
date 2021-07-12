cmake_minimum_required(VERSION 2.8.3)
project(jimmbot_base)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  jimmbot_msgs
  controller_manager
  hardware_interface
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jimmbot_base
  CATKIN_DEPENDS roscpp jimmbot_msgs
#  DEPENDS system_lib
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_can_msg_wrapper src/can_msg_wrapper.cpp)

add_dependencies(${PROJECT_NAME}_can_msg_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_hardware_interface src/hardware_interface.cpp)
add_executable(${PROJECT_NAME}_wheel_commander src/hil_wheel_commander.cpp)
add_executable(${PROJECT_NAME}_extended_joy src/extended_joy.cpp)

set_target_properties(${PROJECT_NAME}_hardware_interface PROPERTIES OUTPUT_NAME hardware_interface PREFIX "")
set_target_properties(${PROJECT_NAME}_wheel_commander PROPERTIES OUTPUT_NAME wheel_commander PREFIX "")
set_target_properties(${PROJECT_NAME}_extended_joy PROPERTIES OUTPUT_NAME extended_joy PREFIX "")

add_dependencies(${PROJECT_NAME}_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_wheel_commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_extended_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_hardware_interface ${catkin_LIBRARIES} ${PROJECT_NAME}_can_msg_wrapper)
target_link_libraries(${PROJECT_NAME}_wheel_commander ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_extended_joy ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_hardware_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_wheel_commander
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_extended_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_can_msg_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(can_msg_wrapper-test test/can_msg_wrapper_test.cpp)
  if(TARGET can_msg_wrapper-test)
    target_link_libraries(can_msg_wrapper-test ${catkin_LIBRARIES} ${PROJECT_NAME}_can_msg_wrapper)
  endif()

  add_rostest_gtest(hardware_interface-test test/hardware_interface_test.test test/hardware_interface_test.cpp)
  if(TARGET hardware_interface-test)
    target_link_libraries(hardware_interface-test ${catkin_LIBRARIES})
  endif()
endif(CATKIN_ENABLE_TESTING) # CATKIN_ENABLE_TESTING
