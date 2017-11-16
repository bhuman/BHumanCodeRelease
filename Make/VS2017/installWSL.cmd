@echo off
pushd "%~dp0"

which which >nul 2>nul
if errorlevel 1 goto notfound
echo Error: You already have a Unix environment in your search path
echo %cmdcmdline% | findstr /ic:"%~f0" >nul && pause
goto end

:notfound
lxrun >nul 2>nul
if not errorlevel 1 goto found
echo Activate the Windows Subsystem for Linux
OptionalFeatures
goto end

:found
rem Copy file to a place that is available in elevated mode
copy DeveloperMode.reg "%TEMP%\DeveloperMode.reg" /Y
echo Windows Subsystem for Linux requires Developer Mode (accept to activate it)
regedit /s "%TEMP%\DeveloperMode.reg"
del "%TEMP%\DeveloperMode.reg"

echo Downloading and installing Windows Subsystem for Linux
lxrun /install /y

echo Downloading and installing necessary packages
bash -c "sudo apt update; sudo apt -y install ccache clang"

:end
popd
