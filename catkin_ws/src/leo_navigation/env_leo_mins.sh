#!/usr/bin/env bash
# Source BOTH catkin_ws (leo_navigation, kalibr, open_vins, realsense) and
# mins_ws (mins) in the same shell.
#
# WHY THIS EXISTS: they are sibling workspaces, both independently configured
# to extend /opt/ros/noetic (mins_ws needs its own thirdparty/open_vins fork,
# so it can't just extend catkin_ws's open_vins). Catkin-generated devel/
# setup.bash scripts are NOT designed to be sourced back-to-back like ROS1's
# classic rosbuild layout — each one resets ROS_PACKAGE_PATH/CMAKE_PREFIX_PATH/
# PATH/LD_LIBRARY_PATH/PYTHONPATH to its OWN chain (self -> its recorded
# --extend target), so sourcing catkin_ws then mins_ws (or vice versa) makes
# the second one silently clobber the first — this bit us as
# `rospkg.common.ResourceNotFound: mins` when catkin_ws was sourced last, and
# would equally have broken leo_navigation (imu_sanitizer, wheel_remap,
# carolus_vicon_bridge — all needed by mins.launch) if mins_ws had been last.
#
# `catkin config --extend` (making mins_ws formally extend catkin_ws/devel)
# is the "proper" catkin fix but requires `catkin clean` + a full rebuild to
# regenerate consistently. This script does the same thing without touching
# either workspace's build: source each independently, then manually union
# the five env vars catkin's setup scripts populate.
#
# Usage:  source env_leo_mins.sh
# NOTE: deliberately no `set -u` — ROS's own setup.bash/setup.sh reference
# variables that may be unset (e.g. on first source), a known incompatibility;
# `set -u` here silently breaks the env union without raising an error.
source /opt/ros/noetic/setup.bash

_RPP="${ROS_PACKAGE_PATH:-}"
_CMPP="${CMAKE_PREFIX_PATH:-}"
_PATH="${PATH:-}"
_LDLP="${LD_LIBRARY_PATH:-}"
_PYP="${PYTHONPATH:-}"
source /home/lab272/TOUT/catkin_ws/devel/setup.bash

source /home/lab272/TOUT/mins_ws/devel/setup.bash
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$_RPP"
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$_CMPP"
export PATH="$PATH:$_PATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$_LDLP"
export PYTHONPATH="$PYTHONPATH:$_PYP"
unset _RPP _CMPP _PATH _LDLP _PYP

echo "[env_leo_mins] leo_navigation: $(rospack find leo_navigation 2>&1)"
echo "[env_leo_mins] mins:           $(rospack find mins 2>&1)"
