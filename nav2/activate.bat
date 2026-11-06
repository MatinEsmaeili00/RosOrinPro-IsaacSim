@echo off
REM pixi activation (Windows): ROS 2 defaults that match the simulator, then the rosorin_nav2 overlay if built.
if "%ROS_DOMAIN_ID%"=="" set "ROS_DOMAIN_ID=0"
if "%RMW_IMPLEMENTATION%"=="" set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
if exist "%PIXI_PROJECT_ROOT%\install\local_setup.bat" call "%PIXI_PROJECT_ROOT%\install\local_setup.bat"
