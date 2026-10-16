@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo [VinSort] Chua co moi truong .venv.
  echo Chay: python -m venv .venv ^&^& .venv\Scripts\pip install -r requirements.txt
  pause
  exit /b 1
)

rem NVIDIA Optimus launcher exports NvOptimusEnablement before PyBullet creates OpenGL.
rem ponytail: launcher is Windows x64 / Python 3.11 only; fallback keeps other installations working.
set "PY=.venv\Scripts\python.exe"
if exist ".venv\Scripts\vin-gpu-python.exe" if exist ".venv\Scripts\python311.dll" if exist ".venv\Scripts\python3.dll" set "PY=.venv\Scripts\vin-gpu-python.exe"
if "%PY%"==".venv\Scripts\python.exe" echo [VinSort] GPU launcher chua duoc build; chay build_gpu_launcher.bat neu muon dung NVIDIA.
echo [VinSort] Dang khoi dong: %PY%
"%PY%" main.py %*

if errorlevel 1 (
  echo.
  echo [VinSort] Chuong trinh dung voi loi %errorlevel%.
  pause
)
