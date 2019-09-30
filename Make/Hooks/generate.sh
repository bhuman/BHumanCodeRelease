#!/bin/bash
scriptPath=$(cd "$(dirname "$(which "$0")")" && pwd)
gitPath=$(dirname "${scriptPath}")
wdPath=$(dirname "${gitPath}")

if ! [ -z "$GIT_WORK_TREE" -o "$GIT_WORK_TREE" = "." -o "$GIT_WORK_TREE" = "$wdPath" ]
then
  echo "ERROR:GIT_WORK_TREE is set to '$GIT_WORK_TREE'. This is not supported by the generate hooks."
  exit 0;
fi

execute() {
  if [ -x "$1" ]
  then
    $1 >/dev/null
  fi
}

if [ "$('GIT' config hooks.generateProject)" = "true" ]
then
  case "$OSTYPE" in
    "cygwin"|"msys")
      if [ -f "${scriptPath}/../../Make/VS2019/generate.cmd" ]; then
        pushd "${scriptPath}/../../Make/VS2019" >/dev/null
        ./generate
        popd >/dev/null
      fi
      ;;
    linux*)
      if [ -e /proc/version -a ! -z "`grep Microsoft </proc/version`" ]; then
        if [ -f "${scriptPath}/../../Make/VS2019/generate.cmd" ]; then
          pushd "${scriptPath}/../../Make/VS2019" >/dev/null
          ./generate
          popd >/dev/null
        fi
      else
        execute "${scriptPath}/../../Make/LinuxCodeLite/generate"
      fi
      ;;
    darwin*)
      execute "${scriptPath}/../../Make/macOS/generate"
      ;;
    *)
      echo "Warning: Unknown platform. Project files not generated." >&2
      ;;
  esac
fi
