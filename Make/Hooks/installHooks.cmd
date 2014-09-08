@echo off
REM SET DEBUG=1
for %%x in (%cmdcmdline%) do if %%~x==/c set DOUBLECLICKED=1
git >nul 2>nul
if errorlevel 9009 GOTO CHECKGIT

:INSTALL
for /f "tokens=1* delims=" %%a in ('cygpath "%~dp0"') do SET CURDIR=%%a
if defined DOUBLECLICKED (
  for /f "tokens=1* delims=" %%a in ('cygpath -t windows /bin') do "%%a\bash" --login -i -c 'alias clear=;"%CURDIR%installHooks" -f %1 %2 %3 %4 %5 %6 %7 %8 %9'
  if defined DOUBLECLICKED pause
) ELSE (
  for /f "tokens=1* delims=" %%a in ('cygpath -t windows /bin') do "%%a\bash" --login -i -c '"%CURDIR%installHooks" %1 %2 %3 %4 %5 %6 %7 %8 %9'
)
GOTO EXIT

:CHECKGIT
IF NOT EXIST gitpath GOTO NOGIT
SET /P INPUT= <gitpath
SET PATH=%INPUT%;%PATH%
git >nul 2>nul
if errorlevel 9009 GOTO NOGIT
GOTO INSTALL

:NOGIT
SET /P INPUT=Please enter your Git\bin dir: 
ECHO.
ECHO %INPUT%>gitpath
SET PATH=%INPUT%;%PATH%
GOTO INSTALL

:EXIT