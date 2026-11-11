@echo off
cd /d "%~dp0"
call settings.bat
echo Closing any OLD copy of the demo...
for /f "tokens=5" %%a in ('netstat -aon ^| findstr "127.0.0.1:47831" ^| findstr LISTENING') do taskkill /F /PID %%a >nul 2>&1
timeout /t 1 >nul
echo.
echo In the video window:  Y = set the yard (click the corners, ENTER)   C = calibrate on an EMPTY yard   Q = quit
py hazard_sw.py --camera %CAMERA% %DSHOW% --arduino %COMPORT% --log mission.csv
pause
