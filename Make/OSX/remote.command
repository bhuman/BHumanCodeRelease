#!/bin/sh
scriptPath=$(echo $0 | sed "s|^\.\./|`pwd`/../|" | sed "s|^\./|`pwd`/|")
basePath=$(dirname ${scriptPath})
cd $basePath
open `ls -t ../../Build/OSX/SimRobot/*/* | head -1 | sed "s%\(.*/\).*%\1SimRobot.app%"` --args "$basePath/../../Config/Scenes/RemoteRobot.ros2"