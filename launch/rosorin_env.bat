@echo off
REM ---------------------------------------------------------------------------------------------
REM ROS 2 environment for Isaac Sim 6.2 + ROSOrin Pro (call this before python.bat / isaac-sim.bat)
REM
REM Default: the ROS 2 Humble libraries bundled with Isaac Sim + FastDDS, i.e. the same middleware
REM as the Jetson on the real robot, so `ros2 topic list` on the robot / any Humble machine on the
REM LAN sees the simulator. Override before calling:
REM     set ROS_DISTRO=jazzy            (bundled Jazzy instead of Humble)
REM     set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
REM     set ROS_DOMAIN_ID=30            (keep the sim separate from the real robot on the same network)
REM ---------------------------------------------------------------------------------------------
set "ROSORIN_REPO=%~dp0.."
for %%I in ("%ROSORIN_REPO%") do set "ROSORIN_REPO=%%~fI"
if "%ISAAC_SIM_DIR%"=="" set "ISAAC_SIM_DIR=%ROSORIN_REPO%\.."
for %%I in ("%ISAAC_SIM_DIR%") do set "ISAAC_SIM_DIR=%%~fI"
if "%ROS_DISTRO%"=="" set "ROS_DISTRO=humble"
if "%RMW_IMPLEMENTATION%"=="" set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
if "%ROS_DOMAIN_ID%"=="" set "ROS_DOMAIN_ID=0"
set "PATH=%ISAAC_SIM_DIR%\exts\isaacsim.ros2.core\%ROS_DISTRO%\lib;%PATH%"
echo [rosorin] Isaac Sim: %ISAAC_SIM_DIR%
echo [rosorin] ROS_DISTRO=%ROS_DISTRO%  RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%  ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
