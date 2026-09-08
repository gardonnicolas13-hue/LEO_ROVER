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
  # roscpp/sensor_msgs (2026-09-08): first C++ node in this package
  # (minimu9_node.cpp, Pololu MinIMU-9 v5 driver). Every other node here is
  # Python, so this pulls in the C++ toolchain for the first time.
  roscpp
  sensor_msgs
  # leo_msgs (2026-09-08) : minimu9_node publie leo_msgs/Imu et non
  # sensor_msgs/Imu, pour se brancher a la SOURCE de la chaine que lit
  # imu_sanitizer. sensor_msgs reste requis pour MagneticField.
  leo_msgs
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
  CATKIN_DEPENDS message_runtime std_msgs roscpp sensor_msgs leo_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# minimu9_node (2026-09-08): reads the Pololu MinIMU-9 v5 (LSM6DS33 +
# LIS3MDL) over the Pi's own I2C bus via linux/i2c-dev.h -- no external I2C
# library, so no extra target_link_libraries entry for one is needed beyond
# catkin's own.
add_executable(minimu9_node src/minimu9_node.cpp)
add_dependencies(minimu9_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(minimu9_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/pose_selector.py
  scripts/imu_sanitizer.py
  scripts/carolus_vicon_bridge.py
  # Ecrit le 30/07 et jamais cable : sans cette ligne catkin ne genere pas le
  # relais devel/, donc roslaunch ne peut pas le resoudre par pkg/type. C'est
  # LUI qui publie `beacon_link`, le repere que carolus_vicon_bridge ci-dessus
  # attend pour produire /mins/external_ref/carolus — sans quoi ce topic reste
  # muet, ce qu'il etait depuis toujours.
  scripts/carolus_tf_bridge.py
  scripts/fill_mins_camchain.py
  scripts/wheel_remap.py
  scripts/vins_rotation_guard.py
  scripts/pi_health_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# leo_backend_launch.sh is BASH, deliberately NOT in catkin_install_python —
# that macro wraps entries in a Python devel-space shim, which crash-loops on
# bash syntax. roslaunch resolves type="leo_backend_launch.sh" straight from
# scripts/ (executable bit is enough); plain install() covers the install space.
install(PROGRAMS scripts/leo_backend_launch.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
