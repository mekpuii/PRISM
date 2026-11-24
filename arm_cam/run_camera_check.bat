@echo off
cd /d "%~dp0"
echo Checking cameras, about 1 minute ...
"%~dp0venv\Scripts\python.exe" camera_report.py
echo.
echo Done. Result is in camera_report.txt. You can close this window.
pause
