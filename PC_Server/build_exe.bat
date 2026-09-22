@echo off
setlocal enabledelayedexpansion

rem ==================================================================
rem  Build OTA_Upgrade_Tool.exe  (single-file GUI tool)
rem
rem  What this does:
rem    1. creates a private virtualenv in PC_Server\.venv (once)
rem    2. installs PyInstaller into it (once)
rem    3. freezes ota_gui.py into dist\OTA_Upgrade_Tool.exe
rem
rem  Why a private venv instead of the system Python:
rem    nothing here needs admin rights, the machine's Python stays
rem    untouched, and the build is reproducible - if it ever goes
rem    wrong you just delete .venv and run this again.
rem
rem  ota_gui.py uses only the standard library (tkinter, socket,
rem  threading, select, struct, zlib, ctypes), so the exe is small and
rem  needs no Python on the target machine.
rem
rem  The result is ONE file. Hand it to the field/production PC as is:
rem      dist\OTA_Upgrade_Tool.exe
rem
rem  IMPORTANT - keep this file pure ASCII.
rem    cmd.exe parses a .bat using the console codepage (GBK on Chinese
rem    Windows). Non-ASCII bytes shift the parser and cmd starts running
rem    fragments of lines. Use English here, or save the file as GBK.
rem
rem  Usage:  build_exe.bat
rem ==================================================================

cd /d "%~dp0"

set "VENV=.venv"
set "VPY=%VENV%\Scripts\python.exe"

rem ---------------------------------------------------------------
rem  1. find a Python to bootstrap the venv with
rem ---------------------------------------------------------------
set "PY="
for %%P in (py python python3) do (
  if not defined PY (
    %%P -c "import sys" >nul 2>&1
    if not errorlevel 1 set "PY=%%P"
  )
)

if not defined PY (
  echo.
  echo [ERROR] No Python found on PATH.
  echo         Install Python 3 from python.org and tick
  echo         "Add python.exe to PATH" during setup.
  echo.
  pause
  exit /b 1
)

rem ---------------------------------------------------------------
rem  2. create the venv (only the first time)
rem    Guarded by the python.exe inside it, not by the folder: a venv
rem    that failed halfway leaves .venv behind without a working
rem    interpreter, and testing for the folder alone would then skip
rem    the repair and fail later with a confusing error.
rem ---------------------------------------------------------------
if exist "%VPY%" (
  echo Using existing virtualenv: %VENV%
) else (
  echo.
  echo ================================
  echo  Creating virtualenv %VENV%
  echo ================================
  if exist "%VENV%" (
    echo   [WARN] %VENV% exists but has no interpreter - rebuilding it.
    rmdir /s /q "%VENV%"
  )
  %PY% -m venv "%VENV%"
  if errorlevel 1 (
    echo.
    echo [ERROR] could not create the virtualenv.
    echo.
    pause
    exit /b 1
  )
)

rem ---------------------------------------------------------------
rem  3. install PyInstaller
rem    --upgrade first because an old pip occasionally refuses wheels
rem    it does not recognise.
rem ---------------------------------------------------------------
echo.
echo ================================
echo  Installing PyInstaller
echo ================================
"%VPY%" -m pip install --upgrade pip
"%VPY%" -m pip install --upgrade pyinstaller
if errorlevel 1 (
  echo.
  echo [ERROR] pip install failed. If this machine is offline, the
  echo         wheels have to be fetched by hand first.
  echo.
  pause
  exit /b 1
)

rem ---------------------------------------------------------------
rem  4. freeze
rem    --onefile     one exe, no accompanying folder
rem    --windowed    no console window (it is a GUI tool)
rem    --noconfirm   overwrite dist\ without asking
rem    --clean       drop cached analysis, so a stale .toc cannot
rem                  silently produce an exe with old code in it
rem
rem    Want a console window for troubleshooting? Replace
rem    --windowed with --console and rebuild.
rem ---------------------------------------------------------------
echo.
echo ================================
echo  Freezing ota_gui.py
echo ================================
"%VPY%" -m PyInstaller --onefile --windowed --noconfirm --clean ^
  --name OTA_Upgrade_Tool ota_gui.py
if errorlevel 1 (
  echo.
  echo [ERROR] PyInstaller failed, see the messages above.
  echo.
  pause
  exit /b 1
)

echo.
echo ================================
echo  Done
echo ================================
for %%F in ("dist\OTA_Upgrade_Tool.exe") do echo   dist\OTA_Upgrade_Tool.exe   size=%%~zF bytes
echo.
echo   Copy that single .exe to the field PC. No Python needed there.
echo.
echo   Reminder - the upgrade still needs BOTH images next to each other:
echo       YH-F429_vX.Y.Z.W_A.bin
echo       YH-F429_vX.Y.Z.W_B.bin
echo   Select either one in the tool; it finds the other by itself.
echo   The tool listens on the address the board dials, which is fixed
echo   at 192.168.1.99:5002 - so set this PC's IP accordingly.
echo.
pause
