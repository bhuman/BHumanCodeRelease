@echo off
setlocal enabledelayedexpansion

set "count=0"
set "driveLetter="

REM Loop through each drive and check if it is a removable drive
for /f "tokens=1,2,3" %%a in ('powershell "Get-WmiObject -Class Win32_LogicalDisk | Select-Object Caption, DriveType, Size"') do (
  if %%b EQU 2 (
    set /a count+=1
    set "driveLetter=%%a"
    set "driveSizeBytes=%%c"
    set "driveSizeGB=?"
    if not "!driveSizeBytes!" == "" set /a driveSizeGB=!driveSizeBytes:~0,-9!
  )
)

REM Check the number of removable drives found
if !count! GTR 1 (
  echo Error: Multiple drives are inserted. To ensure correct USB drive labels, please insert them one by one.
  exit /b
)

if !count! EQU 0 (
  echo No external drive found.
  exit /b
)

set /p stickNumber="Found single external drive: !driveLetter! (!driveSizeGB! GB). What number is written on the USB drive? "
set "label=!stickNumber!_B-HUMAN"

REM Format the drive
format !driveLetter! /FS:exFAT /V:!label! /Q /Y
if %ERRORLEVEL% EQU 0 (
  echo !driveLetter! has been formatted as exFAT with label !label!.

  REM Eject the drive
  powershell "$driveEject = New-Object -comObject Shell.Application; $driveEject.Namespace(17).ParseName('!driveLetter!').InvokeVerb('Eject')"
  echo It should be safe to eject.
) else (
  echo Error formatting drive !driveLetter!.
)

endlocal
