@echo off
REM run_verifyM5.bat — fire-and-forget launcher for verifyM5_endToEnd.
REM
REM Spawned from inside the MATLAB MCP session because the MCP's
REM evaluate_matlab_code call has a 60s timeout and runSingleScenario
REM takes ~60-90s. This .bat sidesteps the nested-quote escaping that
REM broke earlier inline `start "" /B cmd /c matlab -batch ...` attempts.
REM
REM USAGE
REM   start "" /B scripts\run_verifyM5.bat
REM
REM ARTIFACTS
REM   logs\verifyM5_<timestamp>.log              MATLAB stdout/stderr
REM   ..\PROGRESS_M5_screenshots\m5_multi_NN_*.png   saved figures

setlocal

REM --- resolve project root (this .bat lives in scripts\) ------------
set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%.."
set "PROJECT_ROOT=%CD%"
popd

REM --- ensure logs dir exists ----------------------------------------
if not exist "%PROJECT_ROOT%\logs" mkdir "%PROJECT_ROOT%\logs"

REM --- timestamped log file ------------------------------------------
for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value ^| find "="') do set "LDT=%%I"
set "STAMP=%LDT:~0,8%_%LDT:~8,6%"
set "LOGFILE=%PROJECT_ROOT%\logs\verifyM5_%STAMP%.log"

REM --- run MATLAB in batch mode --------------------------------------
REM   -batch suppresses the splash + desktop, runs the command, then
REM   exits with the script's return code. stdout/stderr → %LOGFILE%.
cd /d "%PROJECT_ROOT%"
REM -noFigureWindows (Windows equivalent of -nodisplay): figures still
REM render off-screen for exportgraphics, but no GUI backend — avoids
REM uifigure event-loop hangs that prevent -batch from exiting.
REM
REM diary gives line-buffered progress we can tail; stdout redirection
REM is block-buffered by MATLAB and only flushes on close, which is why
REM the previous run looked stuck.
matlab -batch "diary('%LOGFILE%.diary');diary on;addpath(genpath('src'));addpath('scripts');try;verifyM5_endToEnd;catch ME;fprintf(2,'ERROR: %%s\n',ME.message);for i=1:numel(ME.stack);fprintf(2,'  at %%s (line %%d)\n',ME.stack(i).name,ME.stack(i).line);end;end;diary off;close all force" -noFigureWindows > "%LOGFILE%" 2>&1

endlocal
