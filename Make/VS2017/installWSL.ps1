if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) { Start-Process powershell.exe "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`"" -Verb RunAs; exit }

Write-Host "Activate the Windows Subsystem for Linux"

Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux

Write-Host "Download Ubuntu-16.04 ..."

if(!(Test-Path -Path "~\Ubuntu.appx")){
    Invoke-WebRequest -Uri https://aka.ms/wsl-ubuntu-1604 -OutFile ~/Ubuntu.appx -UseBasicParsing
    Write-Host "  ... to ~\Ubuntu.appx"
}else{
 Write-Host "  ... found ~\Ubuntu.appx"
}

Write-Host "Install Ubuntu-16.04 ..."

$ErrorActionPreference = "Stop"

Try{
    Add-AppxPackage -Path "~/Ubuntu.appx"
}
Catch
{
  Write-Host "The Ubuntu binary was incomplete."
  #Remove-Item –Path "~\Ubuntu.appx"
  Write-Host "Please remove: `"~\Ubuntu.appx`" ..."
  Write-Host "Please run this script again. If this does not work intall WSL via Microsoft-Store. Then install clang and ccache via apt-get."
  pause
  Exit
}

Write-Host "Installing, please press Ctrl+C when asked for a username"
ubuntu config --default-user root

Write-Host "Downloading and installing necessary packages"
ubuntu run "apt update; apt -y install ccache clang-6.0 make; ln -s /usr/bin/clang-6.0 /usr/bin/clang; ln -s /usr/bin/clang++-6.0 /usr/bin/clang++"


pause
