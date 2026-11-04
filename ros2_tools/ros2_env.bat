@echo off
REM Configure a plain Windows shell to use the ROS 2 libraries bundled with Isaac Sim 6.2
REM (no ROS install needed). Usage:  call ros2_tools\ros2_env.bat   [humble|jazzy]
REM Defaults: ROS 2 Humble + FastDDS (what the Jetson on the real robot runs).
if "%ISAAC_SIM_DIR%"=="" set "ISAAC_SIM_DIR=%~dp0..\.."
for %%I in ("%ISAAC_SIM_DIR%") do set "ISAAC_SIM_DIR=%%~fI"
if not "%~1"=="" (set "ROS_DISTRO=%~1") else (if "%ROS_DISTRO%"=="" set "ROS_DISTRO=humble")
if "%RMW_IMPLEMENTATION%"=="" set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
if "%ROS_DOMAIN_ID%"=="" set "ROS_DOMAIN_ID=0"
set "ROS2_BUNDLE=%ISAAC_SIM_DIR%\exts\isaacsim.ros2.core\%ROS_DISTRO%"
set "PATH=%ROS2_BUNDLE%\lib;%PATH%"
REM The numpy copy inside the bundled rclpy folder lacks its DLLs outside Isaac Sim: put Isaac Sim's own
REM numpy (pip_prebundle) first, exactly like inside the simulator.
set "ISAAC_PIP="
for /d %%D in ("%ISAAC_SIM_DIR%\extscache\omni.kit.pip_archive-*") do set "ISAAC_PIP=%%D\pip_prebundle"
set "PYTHONPATH=%ISAAC_PIP%;%ROS2_BUNDLE%\rclpy;%PYTHONPATH%"
set "ISAAC_PYTHON=%ISAAC_SIM_DIR%\kit\python\python.exe"
echo [ros2_env] ROS_DISTRO=%ROS_DISTRO% RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION% ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
