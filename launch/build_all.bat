@echo off
REM Rebuild everything from the Hiwonder description:  URDF -> USD import -> robot asset -> scene(s)
REM   1. scripts\build_sim_urdf.py   description\urdf\rosorin_pro_sim.urdf (+ wheel/roller meshes)
REM   2. scripts\import_to_usd.py    usd\rosorin_pro\ (Isaac Sim URDF importer package)
REM   3. scripts\configure_robot.py  usd\rosorin_pro.usd (physics, sensors, ROS 2 graphs)
REM   4. scripts\build_scene.py      ..\RosOrin_Claude.usd  and  scenes\rosorin_world.usd
setlocal
call "%~dp0rosorin_env.bat"
pushd "%ROSORIN_REPO%"
call "%ISAAC_SIM_DIR%\python.bat" scripts\build_sim_urdf.py %ROSORIN_URDF_ARGS% || goto :fail
call "%ISAAC_SIM_DIR%\python.bat" scripts\import_to_usd.py || goto :fail
call "%ISAAC_SIM_DIR%\python.bat" scripts\configure_robot.py || goto :fail
call "%ISAAC_SIM_DIR%\python.bat" scripts\build_scene.py || goto :fail
call "%ISAAC_SIM_DIR%\python.bat" scripts\build_scene.py --scene scenes\rosorin_world.usd --new || goto :fail
popd
echo [rosorin] BUILD OK
endlocal
exit /b 0
:fail
popd
echo [rosorin] BUILD FAILED
endlocal
exit /b 1
