@echo off
pushd "%~dp0"
..\..\Util\mare\Windows\bin\mare.exe --vcxproj=2017
echo Setting SDK version...
for %%f in (*.vcxproj); do (
  del 2>nul temp.txt
  for /f "tokens=1* delims=§" %%l in (%%%f); do (
    if "%%l" == "  <PropertyGroup Label="Globals">"; (
      set <nul >>temp.txt /p="<PropertyGroup Condition="$^([MSBuild]::ValueOrDefault($^([MSBuild]::GetRegistryValue^('HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Microsoft SDKs\Windows\v10.0','ProductVersion'^)^), $^(^[MSBuild]::GetRegistryValue^('HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\Windows\v10.0','ProductVersion'^)^)^)^) != ''" Label="Globals">"
      echo.>>temp.txt
      set <nul >>temp.txt /p="<WindowsTargetPlatformVersion>$([System.IO.Path]::GetFileName('$([System.IO.Directory]::GetDirectories('$([MSBuild]::ValueOrDefault($([MSBuild]::GetRegistryValue('HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Microsoft SDKs\Windows\v10.0','InstallationFolder')), $([MSBuild]::GetRegistryValue('HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\Windows\v10.0','InstallationFolder'))))Include').GetValue($([System.IO.Directory]::GetDirectories('$([MSBuild]::ValueOrDefault($([MSBuild]::GetRegistryValue('HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Microsoft SDKs\Windows\v10.0','InstallationFolder')), $([MSBuild]::GetRegistryValue('HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\Windows\v10.0','InstallationFolder'))))Include').GetUpperBound(0))))').Trim())</WindowsTargetPlatformVersion>
      echo.>>temp.txt
      set <nul >>temp.txt /p="</PropertyGroup>"
      echo.>>temp.txt
      set <nul >>temp.txt /p="<PropertyGroup Condition="$^([MSBuild]::ValueOrDefault^($^([MSBuild]::GetRegistryValue^('HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Microsoft SDKs\Windows\v10.0','ProductVersion'^)^), $^([MSBuild]::GetRegistryValue^('HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\Windows\v10.0','ProductVersion'^)^)^)^) == '' and $^([MSBuild]::ValueOrDefault^($^([MSBuild]::GetRegistryValue^('HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Microsoft SDKs\Windows\v8.1','ProductVersion'^)^), $^([MSBuild]::GetRegistryValue('HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\Windows\v8.1','ProductVersion'^)^)^)^) != ''" Label="Globals">"
      echo.>>temp.txt
      set <nul >>temp.txt /p="<WindowsTargetPlatformVersion>8.1</WindowsTargetPlatformVersion>"
      echo.>>temp.txt
      set <nul >>temp.txt /p="</PropertyGroup>"
      echo.>>temp.txt
    )
    set <nul >>temp.txt /p="%%l"
    echo.>>temp.txt
  )
  move temp.txt %%f >nul
)
popd
