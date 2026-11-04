@echo off
REM Headless ROSOrin Pro simulation (no window). Args: see launch\run_headless.py
setlocal
call "%~dp0rosorin_env.bat"
call "%ISAAC_SIM_DIR%\python.bat" "%ROSORIN_REPO%\launch\run_headless.py" %*
endlocal
