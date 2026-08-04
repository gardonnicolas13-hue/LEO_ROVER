#!/usr/bin/env bash
# Standalone launcher: unions catkin_ws + mins_ws envs (see env_leo_mins.sh's
# header comment for why they can't just be sourced back-to-back), then runs
# roslaunch leo_navigation <file> (default mins.launch, override with $1 --
# e.g. `launch_mins.sh navigation_master.launch`). Kept as its own script so
# it can be invoked as a single simple `nohup ... &` command.
# Unconditional, not a ${VAR:-default} fallback: each Bash tool invocation is a
# fresh shell (no persisted env), and this script is meant to always target the
# robot's roscore regardless of ambient state — a fallback that silently no-ops
# here once bit us: roslaunch found ROS_MASTER_URI unset/unreachable and
# auto-started its OWN local rosmaster, so MINS registered in an isolated
# bubble with none of the robot's real topics.
# 2026-07-20 : source robot_env.sh au lieu de valeurs figées — reste
# UNCONDITIONNEL (robot_env.sh fait des `export` francs, jamais un fallback
# ${VAR:-...}), donc la leçon ci-dessus tient toujours : jamais de silent
# no-op sur un ROS_MASTER_URI ambiant hérité d'un shell parent.
# shellcheck disable=SC1091
source /home/lab272/TOUT/tools/robot_env.sh

source /opt/ros/noetic/setup.bash
source /home/lab272/TOUT/catkin_ws/devel/setup.bash
_RPP="$ROS_PACKAGE_PATH"; _CMPP="$CMAKE_PREFIX_PATH"; _PATH="$PATH"
_LDLP="$LD_LIBRARY_PATH"; _PYP="$PYTHONPATH"
source /home/lab272/TOUT/mins_ws/devel/setup.bash
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$_RPP"
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$_CMPP"
export PATH="$PATH:$_PATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$_LDLP"
export PYTHONPATH="$PYTHONPATH:$_PYP"

exec roslaunch leo_navigation "${1:-mins.launch}"
