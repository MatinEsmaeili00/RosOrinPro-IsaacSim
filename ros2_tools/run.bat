@echo off
REM Run a Python ROS 2 tool with Isaac Sim's bundled ROS 2 (no ROS install needed).
REM Usage: ros2_tools\run.bat ros2_tools\topic_list.py [args...]
setlocal
call "%~dp0ros2_env.bat"
"%ISAAC_PYTHON%" %*
endlocal
