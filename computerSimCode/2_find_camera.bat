@echo off
cd /d "%~dp0"
echo Make sure Camo Studio is running and your phone is connected.
py list_cameras.py
echo.
echo Open the saved camera pictures, find the one from your phone, and put its number in settings.bat (CAMERA=...)
pause
