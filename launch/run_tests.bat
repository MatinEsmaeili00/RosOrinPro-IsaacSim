@echo off
REM Headless end-to-end test (robot motion, arm, every ROS 2 topic). Results -> tests\results\report.json
setlocal
call "%~dp0rosorin_env.bat"
call "%ISAAC_SIM_DIR%\python.bat" "%ROSORIN_REPO%\tests\test_sim_headless.py" %*
endlocal
