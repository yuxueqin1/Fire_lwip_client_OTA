@echo off
setlocal enabledelayedexpansion

rem ==================================================================
rem  OTA A/B dual-image build
rem
rem  Why two images must be built together:
rem    A/B ping-pong OTA always writes the new firmware into the slot the
rem    device is NOT currently running, so the same source tree has to be
rem    built twice with different link addresses. The only differences:
rem      LwIP_A   IROM1=0x08008000 (APP1)  OTA_SLOT_SEL=1  vectors 0x8000
rem      LwIP_B   IROM1=0x08080000 (APP2)  OTA_SLOT_SEL=2  vectors 0x80000
rem
rem  Output for the OTA upgrade tool (give BOTH files, same folder, pick either):
rem      ..\Output\ota\YH-F429_v1.0.0.5_A.bin
rem      ..\Output\ota\YH-F429_v1.0.0.5_B.bin
rem  Output for the factory/production line (ONE file, program and done):
rem      ..\Output\ota\YH-F429_v1.0.0.5_production.hex
rem        = Bootloader (0x08000000) + A image (0x08008000), merged by
rem          Project\merge_hex.py. Put this A image on the PC tool's side
rem          too, so the factory-provisioned firmware and the OTA image
rem          are byte-identical.
rem
rem  The version number in the filename is read from ..\User\ota\ota_config.h,
rem  so the filename, the version the PC tool auto-detects, and the version the
rem  firmware itself reports are guaranteed to be the same number.
rem
rem  IMPORTANT - keep this file pure ASCII.
rem    cmd.exe parses a .bat file using the console codepage (GBK on Chinese
rem    Windows). Non-ASCII bytes here shift the parser position and cmd starts
rem    executing fragments of lines (a "chcp 65001" line is far too late to
rem    help - it runs after the damage). Use English here, or save as GBK.
rem
rem  Usage:  build_ota.bat
rem ==================================================================

cd /d "%~dp0"

rem ---------------------------------------------------------------
rem  1. locate UV4.exe - if your Keil is somewhere else, add the path
rem     to this list (keep the quotes)
rem ---------------------------------------------------------------
set "UV4="
for %%P in (
  "D:\Program Files\Keil_v5\UV4\UV4.exe"
  "C:\Keil_v5\UV4\UV4.exe"
  "D:\Keil_v5\UV4\UV4.exe"
  "E:\Keil_v5\UV4\UV4.exe"
  "C:\Keil\UV4\UV4.exe"
) do if not defined UV4 if exist %%P set "UV4=%%~P"

if not defined UV4 (
  echo.
  echo [ERROR] UV4.exe not found.
  echo         Open this script in Notepad and edit the for-list above
  echo         so it points at your Keil installation.
  echo.
  pause
  exit /b 1
)
echo Using Keil: %UV4%

rem ---------------------------------------------------------------
rem  2. read the firmware version out of ota_config.h
rem ---------------------------------------------------------------
set "CFG=..\User\ota\ota_config.h"
set "MAJOR="
set "MINOR="
set "REV="
set "BLD="

for /f "tokens=3" %%a in ('findstr /c:"#define FW_VERSION_MAJOR" "%CFG%"') do set "MAJOR=%%a"
for /f "tokens=3" %%a in ('findstr /c:"#define FW_VERSION_MINOR" "%CFG%"') do set "MINOR=%%a"
for /f "tokens=3" %%a in ('findstr /c:"#define FW_VERSION_REVISION" "%CFG%"') do set "REV=%%a"
for /f "tokens=3" %%a in ('findstr /c:"#define FW_VERSION_BUILD" "%CFG%"') do set "BLD=%%a"

set "VER="
if defined MAJOR if defined MINOR if defined REV if defined BLD set "VER=v!MAJOR!.!MINOR!.!REV!.!BLD!"

echo.
if defined VER (
  echo Firmware version: !VER!   from %CFG%
  echo   NOTE: it must be greater than the version running on the board,
  echo         otherwise the device answers REJECT_UPDATE and nothing happens.
) else (
  echo [WARN] cannot read FW_VERSION_* from %CFG%
  echo        Images will be named without a version number.
  echo        Enter the version manually in the PC tool.
)

rem ---------------------------------------------------------------
rem  3. build both targets
rem    The two targets MUST keep separate output directories (Output\A,
rem    Output\B). If they share one, switching targets makes Keil believe
rem    the .o files are still current, so it links A's objects into B's
rem    image. OTA_SLOT_SEL and VECT_TAB_OFFSET are compile-time constants:
rem    after such a mix-up the download and CRC checks still pass and the
rem    image only fails after the device jumps to it.
rem ---------------------------------------------------------------
set "RC=0"

for %%T in (LwIP_A LwIP_B) do (
  echo.
  echo ================================
  echo  Building target %%T
  echo ================================
  "%UV4%" -j0 -b YH-F429.uvprojx -t "%%T" -o "build_%%T.log"
  set "CODE=!ERRORLEVEL!"
  if !CODE! GEQ 2 (
    echo [FAILED] target %%T has errors, UV4 returned !CODE!, see build_%%T.log
    set "RC=1"
  ) else (
    if !CODE! EQU 1 (
      echo [ OK ] target %%T built with warnings, see build_%%T.log
    ) else (
      echo [ OK ] target %%T built, no warnings
    )
  )
)

if "!RC!" NEQ "0" (
  echo.
  echo Build failed, images were not collected.
  pause
  exit /b 1
)

rem ---------------------------------------------------------------
rem  4. collect the images
rem ---------------------------------------------------------------
if not exist "..\Output\ota" mkdir "..\Output\ota"

if defined VER (
  copy /y "..\Output\A\YH-F429_A.bin" "..\Output\ota\YH-F429_!VER!_A.bin" >nul
  copy /y "..\Output\B\YH-F429_B.bin" "..\Output\ota\YH-F429_!VER!_B.bin" >nul
  set "BINA=..\Output\ota\YH-F429_!VER!_A.bin"
  set "BINB=..\Output\ota\YH-F429_!VER!_B.bin"
) else (
  copy /y "..\Output\A\YH-F429_A.bin" "..\Output\ota\YH-F429_A.bin" >nul
  copy /y "..\Output\B\YH-F429_B.bin" "..\Output\ota\YH-F429_B.bin" >nul
  set "BINA=..\Output\ota\YH-F429_A.bin"
  set "BINB=..\Output\ota\YH-F429_B.bin"
)

rem ---------------------------------------------------------------
rem  5. merge Bootloader + A image into ONE production hex
rem    A/B ping-pong needs two .bin files, but the factory only wants
rem    one shot. Both images sit at fixed addresses (Bootloader
rem    0x08000000, A image 0x08008000), so a plain Intel HEX merge is
rem    enough - nothing has to be relocated.
rem
rem    BootInfo is deliberately NOT included: the IAP self-heals it on
rem    first boot (magic is blank -> it writes the default slot A and
rem    prints "boot_info missing, default slot A written"). Burning a
rem    BootInfo here would only override that first-boot decision.
rem
rem    Why the bootloader hex path is not on the command line:
rem    it contains non-ASCII characters, and this file must stay pure
rem    ASCII. merge_hex.py carries that path as its built-in default.
rem ---------------------------------------------------------------
set "PROD="
set "PY="
for %%P in (py python python3) do (
  if not defined PY (
    %%P -c "import sys" >nul 2>&1
    if not errorlevel 1 set "PY=%%P"
  )
)

echo.
echo ================================
echo  Production hex
echo ================================
if not defined PY (
  echo   [WARN] no python found - the production hex was NOT generated.
  echo          The A/B .bin files above are still fine for the upgrade tool.
  echo          To make it by hand, install Python 3 and run:
  echo            cd Project
  echo            python merge_hex.py -o ..\Output\ota\YH-F429_production.hex
) else (
  if defined VER (
    set "PROD=..\Output\ota\YH-F429_!VER!_production.hex"
  ) else (
    set "PROD=..\Output\ota\YH-F429_production.hex"
  )
  %PY% merge_hex.py --app "..\Output\A\YH-F429_A.hex" -o "!PROD!"
  if errorlevel 1 (
    set "PROD="
    echo.
    echo   [FAILED] merge failed - NO production hex was written.
    echo            Read the [ERROR] line above. The A/B .bin files are
    echo            still good, so an OTA upgrade is still possible; only
    echo            the single-file factory image is missing.
  )
)

echo.
echo ================================
echo  Images
echo ================================
for %%F in ("!BINA!") do echo   A image: %%~nxF   size=%%~zF bytes   - to APP1
for %%F in ("!BINB!") do echo   B image: %%~nxF   size=%%~zF bytes   - to APP2
if defined PROD for %%F in ("!PROD!") do echo   Production: %%~nxF   size=%%~zF bytes   - factory, ONE file
echo.
echo   Per-image limit is 384 KB. APP2 is only 384 KB, so anything bigger
echo   must fail here rather than halfway through an upgrade.
echo.
echo   PC tool: put both .bin files in the same folder and select either one -
echo   PC_Server\ota_gui.py pairs them and reads the version automatically.
echo.
pause
