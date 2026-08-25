cmake_minimum_required(VERSION 3.0.2)
project(leo_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  message_generation
)

# Service ajoute le 2026-08-21 : le sélecteur de pose passe de 2 a 3 sources
# (MINS, VINS, sqrtVINS). std_srvs/SetBool ne pouvait pas porter un troisieme
# etat. ~set_source reste expose en SetBool pour les appelants existants.
add_service_files(
  FILES
  SetPoseSource.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(PROGRAMS
  scripts/pose_selector.py
  scripts/imu_sanitizer.py
  scripts/carolus_vicon_bridge.py
  scripts/fill_mins_camchain.py
  scripts/wheel_remap.py
  scripts/vins_rotation_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# leo_backend_launch.sh is BASH, deliberately NOT in catkin_install_python —
# that macro wraps entries in a Python devel-space shim, which crash-loops on
# bash syntax. roslaunch resolves type="leo_backend_launch.sh" straight from
# scripts/ (executable bit is enough); plain install() covers the install space.
install(PROGRAMS scripts/leo_backend_launch.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
