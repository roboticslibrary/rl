
All-in-one Windows installer for the Robotics Library

Please find (or update!) current installation instructions in
  http://www.roboticslibrary.org/install-windows.html
  http://sourceforge.net/apps/mediawiki/roblib/index.php?title=Windows_Installer
  http://www6.in.tum.de/~rickert/rl/install-windows.html


* How to generate the all-in-one Windows installer (for maintainers)

0. Correct all information in extras/w32installer/CMakeLists.txt,
Especially, make sure all the version numbers are accurate, 
they are spread over 
[- rl\extras\w32installer\rl-config-version.cmake]
- rl\extras\w32installer\CMakeLists.txt
- rl\CMakeLists.txt
1. Start from a rather default Windows, uninstall previous RL,
remove PATH variables. Rename libraries that may be falsely found and linked,
like iconv.
2. (First run: Install Dependencies)
CMake into an empty directory with the generator VS2010
- enable option make_thirdparty, *disable* make_rl in the first run
3. CMake Generate, MSBuild ALL_BUILD and PACKAGE in VS2010 (~40 min)
 install the generated exe to get dependecies installed
at exactly the right place
4. Actually set PATH C:\Program Files (x86)\rl-0.6.1\bin (adapt!)
5. (Second run: Compile and package RL)
Now enable make_rl in CMake, set 
CMAKE_INSTALL_PREFIX = C:/Program Files (x86)/rl-0.6.1 (adapt!)
configure and verify advanced options
that all include and libraries paths are *exactly* those that are written
by the depedency installer
6. Verify that all PARENT_SCOPE patches are inside the top CMakeLists.txt (as in rev 474)
6.1. CMake Configure and Generate
6.2. Verify that CPACK_NSIS_EXTRA_INSTALL_COMMANDS is printed correctly
((7. Set C:/Program Files (x86)/rl-?.?.? "Full Control" for Users))
8. In VS2010, reload, choose build type "Release", 
build project PACKAGE (~90 min)
9. (You can inspect the exe installer with 7zip and make sure it looks ok)
10. Test install and verify that a sample program 
correctly cmakes, links, and runs (important), even check
with Dependency walker that the exact version is linked
11. Distribute and announce




