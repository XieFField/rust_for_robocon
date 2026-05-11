@echo off
set CHIP=STM32H723ZG
set TARGET=target\thumbv7em-none-eabihf\debug

echo.
echo === Flash tool ===
echo Available binaries:
echo.
dir /b "%TARGET%\gdut-r1-by-rust" 2>nul
dir /b "%TARGET%\examples\*.elf" 2>nul
echo.

set /p NAME=Enter binary name (e.g. gdut-r1-by-rust or rtt_debugtool_demo):

if "%NAME%"=="" (
    echo Cancelled.
    exit /b 1
)

:: try main first
if exist "%TARGET%\%NAME%" (
    set "ELF=%TARGET%\%NAME%"
    goto :flash
)

:: try examples
if exist "%TARGET%\examples\%NAME%" (
    set "ELF=%TARGET%\examples\%NAME%"
    goto :flash
)

echo Not found: %NAME%
exit /b 1

:flash
echo.
echo [1/2] Downloading %ELF% ...
probe-rs download --chip %CHIP% "%ELF%"
if errorlevel 1 (echo FAILED & exit /b 1)

echo [2/2] Reset (waiting 2s for probe to settle)...
timeout /t 2 /nobreak >nul
probe-rs reset --chip %CHIP% 2>nul
if errorlevel 1 (
    echo Reset via probe failed - press RESET button on board
)

echo.
echo Done. Run: cargo run -p rtt_debug_tool
