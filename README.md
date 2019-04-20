# Robotics Library

The [Robotics Library](https://www.roboticslibrary.org/) (RL) is a self-contained C++ library for rigid body kinematics and dynamics, motion planning, and control. It covers spatial vector algebra, multibody systems, hardware abstraction, path planning, collision detection, and visualization. It is being used in research projects and in education, available under a BSD license, and free for use in commercial applications. RL runs on many different systems, including Linux, macOS, and Windows. It uses CMake as a build system and can be compiled with Clang, GCC, and Visual Studio.

## Getting Started

We offer precompiled Ubuntu packages on [Launchpad](https://launchpad.net/~roblib/+archive/ubuntu/ppa) as well as Windows binaries on [GitHub](https://github.com/roboticslibrary/rl/releases) for the latest release version, while [Homebrew](https://brew.sh/) can be used on macOS to build corresponding packages. Tutorials on our website provide further information on how to develop applications using RL.

These tutorials include instructions on how to

*   install the latest release on [Ubuntu](https://www.roboticslibrary.org/tutorials/install-ubuntu), [Windows](https://www.roboticslibrary.org/tutorials/install-windows), or [macOS](https://www.roboticslibrary.org/tutorials/install-macos),
*   create your first program using RL on [Linux](https://www.roboticslibrary.org/tutorials/first-steps-linux) or [Windows](https://www.roboticslibrary.org/tutorials/first-steps-windows),
*   have a look at our short API [overview](https://www.roboticslibrary.org/api) and our [documentation](http://doc.roboticslibrary.org/),
*   create your [robot model](https://www.roboticslibrary.org/tutorials/create-a-robot-model) with a kinematics and geometry definition,
*   plan a collision-free path in your [path planning scenario](https://www.roboticslibrary.org/tutorials/specify-a-path-planning-scenario),
*   build RL from source on [Ubuntu](https://www.roboticslibrary.org/tutorials/build-ubuntu), [Windows](https://www.roboticslibrary.org/tutorials/build-windows), or [macOS](https://www.roboticslibrary.org/tutorials/build-macos).

## Next Steps

RL includes a number of demo [applications](demos) and a selection of [kinematics](examples/rlmdl), [geometry](examples/rlsg), and [path planning](examples/rlplan) examples that demonstrate how to use it for more advanced applications. Due to their size, a larger set of examples can be found in a [separate repository](https://github.com/roboticslibrary/rl-examples).

Among several others, these demo applications include

*   a tool for [converting](demos/rlRotationConverterDemo) between rotation matrices, angle axis, quaternions, and Euler angles,
*   the visualization of [collision detection](demos/rlCollisionDemo) queries that can highlight intersections, minimum distance, and penetration depth,
*   a [kinematics simulator](demos/rlCoachMdl) that uses a TCP port for joint position updates,
*   a [dynamics simulator](demos/rlSimulator) that listens for joint torque updates,
*   the calculation of a collision-free path using a [Probabilistic Roadmap](demos/rlPrmDemo) or a [Rapidly-Exploring Random Tree](demos/rlRrtDemo),
*   the visualization of [path planning](demos/rlPlanDemo) queries based on scenario definitions from an XML file,
*   robot [forward and inverse dynamics](demos/rlDynamics1Demo) using the Recursive Newton-Euler and Articulated-Body Algorithm methods,
*   the calculation of [dynamics properties](demos/rlDynamics2Demo) such as mass matrix, centrifugal and Coriolis forces, or gravity compensation,
*   the calculation and sending of a [trajectory](demos/rlAxisControllerDemo) to a robot controller based on a cubic or quintic polynomial.

## Publications

For more detailed information on the design of the Robotics Library, please have a look at our IROS paper. The reference is

Markus Rickert and Andre Gaschler. [Robotics Library: An object-oriented approach to robot applications](https://www.roboticslibrary.org/Rickert2017a.pdf). In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pages 733&ndash;740, Vancouver, BC, Canada, September 2017.

	@InProceedings{Rickert2017a,
	  author    = {Markus Rickert and Andre Gaschler},
	  title     = {{R}obotics {L}ibrary: An Object-Oriented Approach to Robot Applications},
	  booktitle = {Proceedings of the {IEEE}/{RSJ} International Conference on Intelligent Robots and Systems},
	  year      = {2017},
	  pages     = {733--740},
	  address   = {Vancouver, BC, Canada},
	  month     = sep,
	  doi       = {10.1109/IROS.2017.8202232},
	}

## License

All source code files of RL are licensed under the permissive [BSD 2-clause license](LICENSE.md). For the licenses of third-party dependencies, please refer to the respective projects.
