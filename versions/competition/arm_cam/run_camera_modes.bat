@echo off
cd /d "%~dp0"
echo Testing camera modes, about 3 minutes. Close any camera preview first ...
"%~dp0venv\Scripts\python.exe" camera_modes.py 0
echo.
echo Done. Result is in camera_modes.txt. You can close this window.
pause
