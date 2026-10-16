@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo [VinSort] Chua co moi truong .venv.
  echo Chay: python -m venv .venv ^&^& .venv\Scripts\pip install -r requirements.txt
  pause
  exit /b 1
)

echo [VinSort] Dang khoi dong...
".venv\Scripts\python.exe" main.py %*

if errorlevel 1 (
  echo.
  echo [VinSort] Chuong trinh dung voi loi %errorlevel%.
  pause
)
