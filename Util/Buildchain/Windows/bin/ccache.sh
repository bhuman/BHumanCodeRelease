#!/bin/bash
if [ ! -z "`which ccache 2> /dev/null`" ] ; then
  ccache $*
else
  $*
fi
