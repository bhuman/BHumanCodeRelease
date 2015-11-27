#!/bin/bash
scriptPath=$(echo $0 | sed "s|^\.\./|`pwd`/../|" | sed "s|^\./|`pwd`/|")
hooksPath=$(dirname ${scriptPath})
gitPath=$(dirname ${hooksPath})
wdPath=$(dirname ${gitPath})

if ! [ -z "$GIT_WORK_TREE" -o "$GIT_WORK_TREE" = "." -o "$GIT_WORK_TREE" = "$wdPath" ]
then
  echo "ERROR:GIT_WORK_TREE is set to '$GIT_WORK_TREE'. This is not supported by the generate hooks."
  exit 0;
fi

scriptPath="`dirname "$0"`"

execute() {
  if [ -x "$1" ]
  then
    $1
  fi
}

if [ "$(git config hooks.generateProject)" = "true" ]
then
  case "$OSTYPE" in
    "cygwin"|"msys") if [ -f "${scriptPath}/../../Make/VS2015/generate.cmd" ]
                     then
                       cmd "/C call $(cygpath -t windows "${scriptPath}/../../Make/VS2015/generate.cmd")"
                     fi
    ;;
#    linux*)          execute "${scriptPath}/../../Make/Linux/generate";;
    darwin*)         execute "${scriptPath}/../../Make/OSX/generate";;
    *)               echo "WARNING: Unknown platform. Project files not generated.";;
  esac
fi
