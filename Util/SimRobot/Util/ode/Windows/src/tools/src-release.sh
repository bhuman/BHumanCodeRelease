#!/bin/sh
###################################################################
# ODE Source Code Release Script
# Originally written by Jason Perkins (starkos@gmail.com)
#
# See README.txt in this directory for complete release
# instructions before running this script.
#
# Prerequisites:
#  Command-line svn installed on path
#  Command-line zip installed on path
#  Command-line doxygen installed on path
#  Autotools support installed
#  Run from a Posix-like shell (Linux, OS X, Cygwin)
###################################################################

# Check arguments
if [ $# -ne 1 ]; then
  echo 1>&2 "Usage: $0 version_number"
  exit 1
fi


###################################################################
# Pre-build checklist
###################################################################

echo "" 
echo "STARTING PREBUILD CHECKLIST, PRESS ^C TO ABORT."
echo ""
echo "Is the version number '$1' correct?"
read line
echo ""
echo "Have you created a release branch named '$1' in SVN?"
read line
echo ""
echo Are 'svn', 'zip', and 'doxygen' on the path?
read line
echo ""
echo "Okay, ready to build the source code package for version $1!"
read line


###################################################################
# Retrieve source code
###################################################################

echo ""
echo "RETRIEVING SOURCE CODE FROM REPOSITORY..."
echo ""

svn export https://opende.svn.sourceforge.net/svnroot/opende/branches/$1 ode-$1


###################################################################
# Prepare source code
###################################################################

echo ""
echo "PREPARING SOURCE TREE..."
echo ""

cd ode-$1
chmod 755 autogen.sh
./autogen.sh
rm -rf autom4te.cache

cp build/config-default.h include/ode/config.h

cd ode/doc
doxygen

cd ../../..


###################################################################
# Package source code
###################################################################

echo ""
echo "PACKAGING SOURCE CODE..."
echo ""

zip -r9 ode-src-$1.zip ode-$1/*


###################################################################
# Clean up
###################################################################

echo ""
echo "CLEANING UP..."
echo ""

rm -rf ode-$1


#####################################################################
# Send the files to SourceForge
#####################################################################

echo ""
echo "Upload packages to SourceForge?"
read line
if [ $line = "y" ]; then
	echo "Uploading to SourceForge..."

	echo "user anonymous starkos" > ftp.txt
	echo "cd incoming" >> ftp.txt
	echo "bin" >> ftp.txt
	echo "put ode-src-$1.zip" >> ftp.txt
	echo "quit" >> ftp.txt

	ftp -n upload.sourceforge.net < ftp.txt

	rm -f ftp.txt
fi
