@echo off
cd /d "%~dp0"
call settings.bat
echo Detection only (no Arduino): good for setting the yard, calibrating and testing.
py hazard_sw.py --camera %CAMERA% %DSHOW%
pause
