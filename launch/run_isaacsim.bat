@echo off
REM ---------------------------------------------------------------------------------------------------
REM Start the Isaac Sim GUI with the ROSOrin Pro scene and the ROS 2 Humble/FastDDS environment.
REM   launch\run_isaacsim.bat                         opens ..\RosOrin_Claude.usd
REM   launch\run_isaacsim.bat scenes\rosorin_world.usd
REM   set ROSORIN_AUTOPLAY=1                          press Play automatically
REM   set ROSORIN_TWIN_DOMAIN=0 & set ROS_DOMAIN_ID=30 digital-twin mode: follow the real robot (domain 0),
REM                                                   publish the simulated sensors on domain 30
REM Uses the Fabric variant of Isaac Sim (physics -> Fabric, ~25 %% faster than isaac-sim.bat).
REM Then drive it from any ROS 2 terminal:  ros2_tools\run.bat ros2_tools\teleop_keyboard.py
REM ---------------------------------------------------------------------------------------------------
setlocal
call "%~dp0rosorin_env.bat"
if "%~1"=="" (set "ROSORIN_SCENE=%ISAAC_SIM_DIR%\RosOrin_Claude.usd") else (set "ROSORIN_SCENE=%~f1")
echo [rosorin] scene: %ROSORIN_SCENE%
call "%ISAAC_SIM_DIR%\isaac-sim.fabric.bat" --exec "%ROSORIN_REPO%\launch\open_scene.py"
endlocal
