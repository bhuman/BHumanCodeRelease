@echo off
pushd "%~dp0"
net localgroup | grep '*' | head -n 1 | sed "s!.\(.*\)!\1 ../../Config/Keys/id_rsa_nao!" | xargs chgrp 
..\..\Util\mare\Win32\bin\mare.exe --vcxproj=2012
popd
