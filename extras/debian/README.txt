
How to maintain RL for Launchpad release

https://launchpad.net/~roblib/+archive/ppa


1. Build binary Debian package locally and
resolve any errors.
# export MAKEFLAGS+='-j8'
 dpkg-buildpackage -rfakeroot

2. Build source package for Launchpad

2.1. Make sure you have an OpenPGP key signing with 
your name and email address
that is exported to your Launchpad account 
 export DEBFULLNAME='Andre Gaschler'
 export DEBEMAIL='gaschler@cs.tum.edu'

2.2. Build source package
 dpkg-buildpackage -rfakeroot -S


3. Launchpad Build

https://help.launchpad.net/Packaging/PPA/Uploading


3.1. Upload changes files to Launchpad
 
 dput ppa:roblib/ppa ../librl_0.6.1*.changes

3.2. Review error logs and retry


