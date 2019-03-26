Download
========

Kautham can be downloaded its public version from `here <http://github.com/iocroblab/kautham>`_ 

The latest version is 4.0.0 available from February 28, 2019.

Installation
------------

There are two options to install Kautham in your computer: install a binary package or build it. By now, we only provide Debian or Ubuntu native packages.


Debian package
^^^^^^^^^^^^^^

If you are using a Debian distribution you can install Kautham package. Kautham has a Debian package built for Debian Jessie and Stretch. It's located at the debian-robotics repository at UPC-IOC.

    `<http://sir.upc.edu/debian-robotics/pool/main/k/kautham/>`_ 

To install the package you need: ::

    $ sudo sh -c 'echo "deb http://sir.upc.edu/debian-robotics XXXXX-robotics main" > /etc/apt/sources.list.d/debian-robotics.list'

Where XXXXX could be jessie or stretch. ::

    $ sudo apt-key adv --keyserver pgp.mit.edu --recv-keys 63DE76AC0B6779BF
    $ sudo apt-get update
    $ sudo apt-get install kautham


    
Ubuntu package
^^^^^^^^^^^^^^

Kautham has a Debian package built for Ubuntu (Trusty 14.04 LTS or Xenial 16.04 LTS) that is located in the debian-robotics repository at `launchpad <https://launchpad.net/~deb-rob/>`_).

If you have an Ubuntu Trusty, follow these steps to add some sources and install the Kautham package: ::

    $ sudo apt-key adv --keyserver pgp.mit.edu --recv-keys 63DE76AC0B6779BF
    $ sudo add-apt-repository ppa:deb-rob/ros-trusty
    $ sudo apt-get update
    $ sudo apt-get install kautham


If you have an Ubuntu Xenial, follow these steps to add some sources and install the Kautham package: ::

    $ sudo apt-key adv --keyserver pgp.mit.edu --recv-keys 63DE76AC0B6779BF
    $ sudo add-apt-repository ppa:deb-rob/ros-xenial
    $ sudo apt-get update
    $ sudo apt-get install kautham


Build Kautham fom sources
^^^^^^^^^^^^^^^^^^^^^^^^^

Install packages that Kautham depends on. First add the required ppa sources (see above for Ubuntu of Debian). ::

    $ sudo apt-get install libompl-dev libsoqt4-dev libcoin80-dev cmake libboost-system-dev libboost-serialization-dev libboost-thread-dev libfcl-dev libassimp-dev  libarmadillo-dev libode-dev libpugixml-dev libeigen3-dev   freeglut3-dev


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
