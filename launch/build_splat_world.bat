@echo off
REM ---------------------------------------------------------------------------------------------------
REM Build the Gaussian-splat navigation scene scenes\splat_world.usd (see docs\SPLAT_ENVIRONMENT.md):
REM   1. PLY -> USD splat, Nav2 map, collision/lidar proxy        (scripts\build_splat_scene.py --stage assets)
REM   2. physics scene + robot + /clock, no arena                (scripts\build_scene.py --new --no-arena)
REM   3. add splat + proxy, robot at the spawn pose              (scripts\build_splat_scene.py --stage compose)
REM   launch\build_splat_world.bat [--skip-convert]   (--skip-convert reuses usd\splat\building.usd)
REM Inputs: config\splat_world.yaml (PLY path, thresholds), nav2\rosorin_nav2\config\worlds.yaml (spawn, places)
REM ---------------------------------------------------------------------------------------------------
setlocal
call "%~dp0rosorin_env.bat"
pushd "%ROSORIN_REPO%"
echo [rosorin] 1/3 splat assets
call "%ISAAC_SIM_DIR%\python.bat" scripts\build_splat_scene.py --stage assets %*
if errorlevel 1 goto :fail
echo [rosorin] 2/3 scene with the robot
call "%ISAAC_SIM_DIR%\python.bat" scripts\build_scene.py --scene scenes\splat_world.usd --new --no-arena
if errorlevel 1 goto :fail
echo [rosorin] 3/3 compose
call "%ISAAC_SIM_DIR%\python.bat" scripts\build_splat_scene.py --stage compose
if errorlevel 1 goto :fail
echo [rosorin] splat world built: %ROSORIN_REPO%\scenes\splat_world.usd
popd
endlocal
exit /b 0
:fail
echo [rosorin] BUILD FAILED
popd
endlocal
exit /b 1
