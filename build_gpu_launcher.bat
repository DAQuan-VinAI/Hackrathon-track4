@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (echo Create .venv first & exit /b 1)
for /f "tokens=2 delims== " %%P in ('findstr /b "home = " ".venv\pyvenv.cfg"') do set "BASE=%%P"
if not exist "%BASE%\libs\python311.lib" (echo Python 3.11 development files missing & exit /b 1)
set "VC=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VC%" (echo Visual Studio 2022 C++ tools missing & exit /b 1)
call "%VC%" >nul
if errorlevel 1 exit /b 1
cl /nologo /O2 /utf-8 /I"%BASE%\include" gpu_python.c /link /NOLOGO /LIBPATH:"%BASE%\libs" python311.lib /OUT:".venv\Scripts\vin-gpu-python.exe"
if errorlevel 1 exit /b 1
copy /y "%BASE%\python311.dll" ".venv\Scripts\python311.dll" >nul
if errorlevel 1 exit /b 1
copy /y "%BASE%\python3.dll" ".venv\Scripts\python3.dll" >nul
if errorlevel 1 exit /b 1
".venv\Scripts\vin-gpu-python.exe" -c "import sys,cv2;assert sys.prefix.endswith('.venv');print('GPU launcher OK:',sys.executable)"
