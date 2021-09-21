cmake_minimum_required(VERSION 2.8.3)

project(applications_aerotech)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_ctrl src/joint_ctrl.cpp)
target_link_libraries(joint_ctrl ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS script/joint_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS script/simple_action_server_aero.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  catkin_install_python(PROGRAMS script/node_read_joint_states.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
