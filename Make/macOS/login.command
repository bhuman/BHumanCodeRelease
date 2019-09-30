#!/bin/sh
# Open a SSH connection to a NAO robot.
# This script can be launched from the Finder. It is an interactive
# front end to the common login script.

if [ $TERM == dumb ]; then open $0; exit; fi
clear
cd "$(dirname "$(which "$0")")"
set -eu
read -p 'Cable or WLAN (C/w): ' MODE
TEAM=`grep <../../Config/settings.cfg "teamNumber" | sed "s%[^=]*=[ ]*\([0-9]*\).*%\1%"`
WIRED=`grep "%teamID%" ../../Install/Network/wired.service | sed "s/.*eth0 *\([^%]*\).*/\1/"`
WLAN=`grep "%teamID%" ../../Install/Network/wireless.service | sed "s/.*eth0 *\([^%]*\).*/\1/"`
if [ "$MODE" == "w" ]; then
  SUBNET=$WLAN$TEAM.
else
  SUBNET=$WIRED$TEAM.
fi
read -p "Connect to $SUBNET" IP
REMOTE=$SUBNET$IP
if [ `ping -t 1 -c 1 $REMOTE >/dev/null && echo 1 || echo 0` = "0" ]; then
  read -p "$REMOTE not reachable" DUMMY
  exit 1
fi
./login $REMOTE
