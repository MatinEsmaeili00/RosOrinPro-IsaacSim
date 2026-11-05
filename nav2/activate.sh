# pixi activation (Linux): ROS 2 defaults that match the simulator, then the rosorin_nav2 overlay if built.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ -f "$PIXI_PROJECT_ROOT/install/local_setup.sh" ]; then . "$PIXI_PROJECT_ROOT/install/local_setup.sh"; fi
