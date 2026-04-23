@echo off
REM run_verifyM4_regress.bat — fire-and-forget launcher for verifyM4_regress.
REM
REM Mirrors run_verifyM5.bat (same diary + -noFigureWindows pattern) so the
REM regression runs through the identical execution environment that just
REM verified M5. See that file for the full rationale on -batch, diary
REM buffering, and quote escaping.
REM
REM USAGE
REM   start "" /B scripts\run_verifyM4_regress.bat
REM
REM ARTIFACTS
REM   logs\verifyM4regress_<timestamp>.log         MATLAB stdout/stderr
REM   ..\PROGRESS_M5_screenshots\m4_regress_NN_*.png   saved figures
REM   ..\PROGRESS_M5_screenshots\_REGRESS_DONE.marker  clean-exit signal

setlocal

set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%.."
set "PROJECT_ROOT=%CD%"
popd

if not exist "%PROJECT_ROOT%\logs" mkdir "%PROJECT_ROOT%\logs"

for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value ^| find "="') do set "LDT=%%I"
set "STAMP=%LDT:~0,8%_%LDT:~8,6%"
set "LOGFILE=%PROJECT_ROOT%\logs\verifyM4regress_%STAMP%.log"

cd /d "%PROJECT_ROOT%"
matlab -batch "diary('%LOGFILE%.diary');diary on;addpath(genpath('src'));addpath('scripts');try;verifyM4_regress;catch ME;fprintf(2,'ERROR: %%s\n',ME.message);for i=1:numel(ME.stack);fprintf(2,'  at %%s (line %%d)\n',ME.stack(i).name,ME.stack(i).line);end;end;diary off;close all force" -noFigureWindows > "%LOGFILE%" 2>&1

endlocal
