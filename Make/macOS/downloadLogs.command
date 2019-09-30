#!/bin/sh
# Download all log files from a certain NAO.
# This script can be launched from the Finder. It is an interactive
# front end to the common downloadLogs script.

if [ $TERM == dumb ]; then open $0; exit; fi
clear
cd `dirname $0`
set -eu
TEAM=`grep <../../Config/settings.cfg "teamNumber" | sed "s%[^=]*=[ ]*\([0-9]*\).*%\1%"`
WIRED=`grep "%teamID%" ../../Install/Network/wired.service | sed "s/.*eth0 *\([^%]*\).*/\1/"`
SUBNET=$WIRED$TEAM.
read -p "Download from $SUBNET" IP
REMOTE=$SUBNET$IP

if [ `ping -t 1 -c 1 $REMOTE >/dev/null && echo 1 || echo 0` = "0" ]; then
  read -p "$REMOTE not reachable" DUMMY
  exit 1
fi

NAME=`grep -H $REMOTE ../../Config/Robots/*/network.cfg | sed "s%.*/\(.*\)/network.cfg.*%\1%"`
./downloadLogs -d $NAME $REMOTE
