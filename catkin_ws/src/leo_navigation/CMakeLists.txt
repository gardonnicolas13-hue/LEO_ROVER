cmake_minimum_required(VERSION 3.0.2)
project(leo_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

catkin_package()

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
