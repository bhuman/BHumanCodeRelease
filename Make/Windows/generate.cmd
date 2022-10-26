@echo off
pushd "%~dp0"
bash ./generate
if exist B-Human.lnk goto linkExists
powershell "$s=(New-Object -COM WScript.Shell).CreateShortcut('%CD%\B-Human.lnk');$s.TargetPath='%CD%\..\..\Build\Windows\CMake\B-Human.sln';$s.Save()"
:linkExists
popd
