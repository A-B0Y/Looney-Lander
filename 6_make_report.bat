@echo off
cd /d "%~dp0"
py mission_report.py mission.csv
start "" mission_report.html
pause
