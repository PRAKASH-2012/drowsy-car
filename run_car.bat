@echo off
REM Double-click launcher: runs the controller with the project venv (Python 3.11 + mediapipe 0.10.14).
cd /d %~dp0
.\.venv\Scripts\python.exe -u controler.py
pause
