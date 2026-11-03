@echo off
REM Run any Isaac Sim Python script of this repo with the ROSOrin ROS 2 environment.
REM Usage: launch\run_python.bat tests\tune_mecanum.py [args...]
setlocal
call "%~dp0rosorin_env.bat"
pushd "%ROSORIN_REPO%"
call "%ISAAC_SIM_DIR%\python.bat" %*
popd
endlocal
