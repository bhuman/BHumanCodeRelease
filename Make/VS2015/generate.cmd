@echo off
pushd "%~dp0"
net localgroup | grep '*' | grep -v '_' | head -n 1 | sed "s! !\\\ !g" | sed "s!.\(.*\)!\1 ../../Config/Keys/id_rsa_nao!" | xargs chgrp
..\..\Util\SimRobot\Util\mare\Windows\bin\mare.exe --vcxproj=2015
popd
