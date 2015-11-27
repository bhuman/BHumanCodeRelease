@echo off
rem ***********************************************************
rem * ODE Windows Binary Release Script
rem * Originally written by Jason Perkins (starkos@gmail.com)
rem *
rem * See README.txt in this directory for complete release
rem * instructions before running this script.
rem *
rem * Prerequisites:
rem *  Command-line svn installed on path
rem *  Command-line 7z (7zip) installed on path
rem *  Command-line doxygen installed on path
rem *  Run from Visual Studio 2003 command prompt
rem ***********************************************************

rem * Check arguments
if "%1"=="" goto show_usage


rem ***********************************************************
rem * Pre-build checklist
rem ***********************************************************

echo. 
echo STARTING PREBUILD CHECKLIST, PRESS ^^C TO ABORT.
echo.
echo Are you running at the VS2003 command prompt?
pause
echo.
echo Is the version number "%1" correct?
pause
echo.
echo Does the release branch "%1" exist in SVN?
pause
echo.
echo Are 'svn', '7z', and 'doxygen' on the path?
pause
echo.
echo Okay, ready to build the Windows binary packages for version %1!
pause


rem ***********************************************************
rem * Retrieve source code
rem ***********************************************************

echo.
echo RETRIEVING SOURCE CODE FROM REPOSITORY...
echo.

svn export https://opende.svn.sourceforge.net/svnroot/opende/branches/%1 ode-%1



rem ***********************************************************
rem * Prepare source code
rem ***********************************************************

echo.
echo PREPARING SOURCE TREE...
echo.

cd ode-%1
copy build\config-default.h include\ode\config.h

cd ode\doc
doxygen

cd ..\..\..


rem ***********************************************************
rem * Build the binaries
rem ***********************************************************

echo.
echo BUILDING RELEASE BINARIES (this will take a while)...
echo.

cd ode-%1\build\vs2003
devenv.exe ode.sln /build DebugLib /project ode
devenv.exe ode.sln /build DebugDLL /project ode
devenv.exe ode.sln /build ReleaseLib /project ode
devenv.exe ode.sln /build ReleaseDLL /project ode


rem ***********************************************************
rem * Package things up
rem ***********************************************************

cd ..\..
move lib\ReleaseDLL\ode.lib lib\ReleaseDLL\ode-imports.lib

cd ..
7z a -tzip ode-win32-%1.zip ode-%1\*.txt ode-%1\include\ode\*.h ode-%1\lib\* ode-%1\docs\*


rem ***********************************************************
rem * Clean up
rem ***********************************************************

echo.
echo CLEANING UP...
echo.

rmdir /s /q ode-%1


rem ***********************************************************
rem * Upload to SF.net
rem ***********************************************************

echo.
echo Ready to upload package to SourceForce, press ^^C to abort.
pause

echo "anonymous" > ftp.txt
echo "starkos" >> ftp.txt
echo "cd incoming" >> ftp.txt
echo "bin" >> ftp.txt
echo "put ode-win32-%1.zip" >> ftp.txt
echo "quit" >> ftp.txt

ftp -s:ftp.txt upload.sourceforge.net
erase ftp.txt

goto done


rem ***********************************************************
rem * Error messages
rem ***********************************************************

:show_usage
echo Usage: msw_release.bat version_number
goto done

:done
