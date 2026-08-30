#!/usr/bin/env bash
# ============================================================================
# Robotics Studio 1 - one-shot environment setup.
#
# Source this from ~/.bashrc so every new terminal is ready to run the sample
# generator + the rs1_ws parrot simulation:
#
#     echo 'source "$HOME/git/robotics-studio-1/setup_env.sh"' >> ~/.bashrc
#     source ~/.bashrc
#
# What it does:
#   * sources ROS 2 Humble
#   * sources the sample_generator overlay  (sample_gen + plugin + msgs)
#   * sources the rs1_ws overlay            (41068_ignition_bringup)
#   * sets the Gazebo resource path so `model://trees/...` mesh URIs resolve
#   * makes the object_label_plugin library findable by Ignition Gazebo
# ============================================================================

# ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

REPO_ROOT="${HOME}/git/robotics-studio-1"

# Sample generator workspace (sample_gen, sample_gen_gazebo_plugins, sample_gen_msgs)
if [ -f "${REPO_ROOT}/sample_generator/install/setup.bash" ]; then
  source "${REPO_ROOT}/sample_generator/install/setup.bash"
fi

# rs1_ws workspace (41068_ignition_bringup, rs1_sim)
if [ -f "${REPO_ROOT}/rs1_ws/install/setup.bash" ]; then
  source "${REPO_ROOT}/rs1_ws/install/setup.bash"
fi

# Local tree models (parent of trees/) so `model://trees/oak_tree/...` resolve.
export IGN_GAZEBO_RESOURCE_PATH="${REPO_ROOT}/rs1_ws/src/rs1_sim/models${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"

# object_label_plugin shared library used by the tree models.
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${REPO_ROOT}/sample_generator/install/sample_gen_gazebo_plugins/share/sample_gen_gazebo_plugins${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}}"
