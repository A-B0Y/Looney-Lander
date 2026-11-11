@echo off
cd /d "%~dp0"
echo Installing the libraries (this can take a few minutes)...
py -m pip install opencv-python numpy pyserial
echo.
py hazard_sw.py --selftest
echo.
echo If you see "self-test OK" above, step 1 is done.
pause
