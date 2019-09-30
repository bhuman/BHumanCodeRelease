if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
  Start-Process powershell.exe "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`"" -Verb RunAs
  exit
}

$wsl = "$PSScriptRoot/../../Util/WSL"

$ErrorActionPreference = "Stop"

Try {
    Write-Host "Activating the Windows Subsystem for Linux"
    Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux

    if(!(Test-Path -Path "$wsl") -and !(Test-Path -Path "$wsl.zip")) {
        Write-Host "Downloading Ubuntu-18.04 ..."
        Invoke-WebRequest -Uri "https://aka.ms/wsl-ubuntu-1804" -OutFile "$wsl.zip" -UseBasicParsing
    } else {
        Write-Host "Using existing archive $wsl.zip"
    }

    if(!(Test-Path -Path "$wsl")) {
        Write-Host "Unpacking archive ..."
        Expand-Archive -Path "$wsl.zip" -DestinationPath "$wsl"
    } else {
        Write-Host "Using existing folder $wsl"
    }

    Write-Host "Installing Ubuntu-18.04 ..."
    & "$wsl/ubuntu1804.exe" install --root
    Write-Host "Downloading and installing necessary packages"
    wsl apt update
    wsl apt -y install ccache clang make

    Write-Host "Deleting installation files ..."
    Remove-Item -Path "$wsl.zip"
    Remove-Item -Path "$wsl/install.tar.gz"
    Remove-Item -Path "$wsl/Appx*" -Recurse
    Remove-Item -Path "$wsl/Assets/*"
    Remove-Item -Path "$wsl/Assets"
    Remove-Item -Path "$wsl/resources.pri"
    Remove-Item -Path "$wsl/*.xml"
    Remove-Item -Path "$wsl/ubuntu1804.exe"
} Catch {
    Write-Host "$_"
}

pause
