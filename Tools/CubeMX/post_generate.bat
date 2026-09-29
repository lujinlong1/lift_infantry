@echo off
setlocal
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0post_generate.ps1"
set "exit_code=%errorlevel%"
if not "%exit_code%"=="0" echo [CubeMX post-generation] FAILED with exit code %exit_code% 1>&2
exit /b %exit_code%
