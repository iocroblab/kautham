Download
========

Kautham can be downloaded its public version from `here <http://github.com/iocroblab/kautham>`_

The latest version is 4.0.5 available from September 30th, 2019.

Installation
------------

There are two options to install Kautham in your computer: install a binary package or build it. By now, we only provide Debian or Ubuntu native packages.


Debian package
^^^^^^^^^^^^^^^^

If you are using a Debian distribution you can install Kautham package. Kautham has a Debian package built for Debian Stretch and Buster. It's located at the debian-robotics repository at UPC-IOC. The latest version is just for Debian Buster.

    `<http://sir.upc.edu/debian-robotics/pool/main/k/kautham/>`_

To install the package you need:

If you are using Debian Stretch ::

    $ sudo sh -c 'echo "deb http://sir.upc.edu/debian-robotics stretch-robotics main" > /etc/apt/sources.list.d/debian-robotics.list'

If you are using Debian Buster ::

    $ sudo sh -c 'echo "deb http://sir.upc.edu/debian-robotics buster-robotics main" > /etc/apt/sources.list.d/debian-robotics.list'

Then in both cases you need to install it: ::

    $ sudo apt-key adv --keyserver pgp.mit.edu --recv-keys 63DE76AC0B6779BF
    $ sudo apt-get update
    $ sudo apt-get install kautham kautham-ros


Ubuntu package
^^^^^^^^^^^^^^^^

Kautham has a package built for **Ubuntu Bionic 18.04 LTS** that is located in the debian-robotics repository at `launchpad <https://launchpad.net/~deb-rob/>`_).

Follow these steps to add some sources and install the Kautham package: ::


    $ sudo add-apt-repository ppa:deb-rob/ros-bionic
    $ sudo apt-get update
    $ sudo apt-get install kautham kautham-tools kautham-ros-osrf kautham-demos-osrf
    $ sudo apt-get upgrade

    
Also there are packages built for Ubuntu Trusty 14.04 LTS and Xenial 16.04 LTS  but for OLDER versions of Kautham.
    
If you have an Ubuntu Xenial, follow these steps to add some sources and install the Kautham package: ::


    $ sudo add-apt-repository ppa:deb-rob/ros-xenial
    $ sudo apt-get update
    $ sudo apt-get install kautham

If you have an Ubuntu Trusty, follow these steps to add some sources and install the Kautham package: ::


    $ sudo add-apt-repository ppa:deb-rob/ros-trusty
    $ sudo apt-get update
    $ sudo apt-get install kautham


ROS stuff
^^^^^^^^^^^

Currently the kautham packages are built with ROS activated (Ubuntu Bionic and Debian Buster). The Debian ones are using the Debian version of ROS and install it in /usr. In the case of Ubuntu packages, kautham is built against the ROS packages in Universe repo (from Debian). We have not found any incompatibilities between this version and the OSRF version of Melodic.

There are a kautham-ros-osrf and kautham-demos-osrf packages that create the needed links between the /usr original installation and the /opt/ros/melodic OSRF ROS installation.


Build Kautham fom sources
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install packages that Kautham depends on. First add the required ppa sources (see above for Ubuntu or Debian). Developed version of Kautham just works in Debian Stretch and Buster and Ubuntu Bionic because a Qt5 migration and Coin and SoQt dependencies. ::

    $ sudo apt-get install libompl-dev cmake libboost-system-dev libboost-serialization-dev libboost-thread-dev libfcl-dev libassimp-dev  libarmadillo-dev libode-dev libpugixml-dev libeigen3-dev   freeglut3-dev libsoqt520-dev libcoin-dev libroscpp-dev libtrajectory-msgs-dev  ros-message-generation


Dowload the Kautham source code from `github <https://github.com/iocroblab/kautham>`_ at e.g. ~/kautham

Build the package. From the Kautham folder: ::

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

There are three apps: kautham-gui, kautham-console and kautham-ros.

You can activate/deactivate their building modifying the corresponing flags in the CMakeLists.txt file; by default kautham-gui and kautham-console are ON and kautham-ros is OFF.

If kautham-ros is OFF the executable files are located at the kautham/build/apps folder.

If kautham-ros is ON the executables are located at kautham/build/devel/lib/kautham, since catkin is used for the building.
