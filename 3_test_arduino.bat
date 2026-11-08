@echo off
cd /d "%~dp0"
call settings.bat
echo Close the Arduino Serial Monitor first. Using port %COMPORT%
py landing_demo.py %COMPORT%
pause
