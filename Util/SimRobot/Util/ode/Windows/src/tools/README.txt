HOW TO MAKE A RELEASE

Originally by Jason Perkins (starkos@industriousone.com)


PREREQUISITES

In order to build an OpenDE release you'll need:

* A Windows system
* A Posix-like system (Linux, Mac OS X, Cygwin)
* Visual Studio 2003 or later (for C++ release)
* Visual Studio 2005 or later (for .NET release)
* Subversion (command line version)
* 7zip (command line version)
* Doxygen

All command line binaries (svn, 7z, doxygen) must be available on 
the system search path. 



INSTRUCTIONS

* Update the version numbers in configure.in

    AC_INIT(ODE,0.9.0-rc1,ode@ode.org)
    ODE_CURRENT=0
    ODE_REVISION=9
    ODE_AGE=0-rc1


* Create a release branch in Subversion. Using TortoiseSVN:

    Update working copy
    Right-click working copy folder and choose Branch/Tag
    To https://opende.svn.sourceforge.net/svnroot/branches/0.9.0-rc1
    Enter log message "Branching for 0.9.0-rc1 release"
    OK

  Using the command line:

    $ cd ode
    $ svn update
    $ svn copy . -m "Branching for..." https://opende.... (see above)


* Run msw-release.bat from a Visual Studio command prompt to create
  the Windows binary package. I've used VS2003 for all releases since
  0.5, but am thinking about switching up to VS2005. The format of
  this command is:

    > msw-release.bat 0.9.0-rc1

  The version argument supplied to the script must match the name of
  the release branch in Subversion.


* Run dotnet-release.bat to create the .NET bindings package. This
  script must be run from a Visual Studio 2005 (or later) command
  prompt. The format of the command is:

    > dotnet-release.bat 0.9.0-rc1

  The version argument supplied to the script must match the name of
  the release branch in Subversion.


* Run src-release.sh to create the source package. This script must be
  run from a Posix-like environment in order to prepare the configure
  script. I've tried it under Linux, Mac OS X, and Windows with Cygwin.

    $ ./src-release.sh 0.9.0-rc1

  The version argument supplied to the script must match the name of
  the release branch in Subversion.


* Visit the project site on SourceForge (http://sf.net/projects/opende)
  and create a new file release including all of the files.
  
  
SANITY CHECKING

A few optional, but recommeded, checks to make sure that everything
worked properly.

  The binaries exist in lib/

  include/ode/config.h exists

  configure script exists (if not, make sure autotools is installed)

  docs exist in docs/ (if not, make sure doxygen is on the path)
