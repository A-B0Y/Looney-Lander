@echo off
REM ============ YOUR SETTINGS (edit with Notepad, save, close) ============
REM Arduino port: in the Arduino IDE look at Tools > Port, for example COM5
set COMPORT=COM3
REM Camera number: run 2_find_camera.bat to find the number for your phone (Camo)
set CAMERA=1
REM If the camera will not open or looks laggy, change the next line to:  set DSHOW=--dshow
set DSHOW=
