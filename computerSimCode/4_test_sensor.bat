@echo off
cd /d "%~dp0"
call settings.bat
echo Hold your hand above the distance sensor. You should see alt= numbers change.
py sensor_check.py %COMPORT%
pause
