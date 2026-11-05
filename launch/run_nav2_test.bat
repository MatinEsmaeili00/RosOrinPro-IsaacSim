@echo off
REM End-to-end Nav2 test (tests\test_nav2.py): headless simulation + Nav2 + a tour of goals.
REM   launch\run_nav2_test.bat                        splat building, AMCL, full tour
REM   launch\run_nav2_test.bat --world arena --slam   test arena with slam_toolbox
REM Results: tests\results\report_nav2_<world>.json (+ logs). Uses Isaac Sim's bundled python.exe (stdlib only).
setlocal
set "ROSORIN_REPO=%~dp0.."
"%ROSORIN_REPO%\..\kit\python\python.exe" "%ROSORIN_REPO%\tests\test_nav2.py" %*
set "RC=%ERRORLEVEL%"
endlocal & exit /b %RC%
