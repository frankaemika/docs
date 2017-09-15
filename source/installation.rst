.. highlight:: shell

Installation instructions
=========================

In this chapter, the steps to set up a real-time kernel on Ubuntu as well as building
``libfranka`` and ``franka_ros`` are described.


Setting up a real-time kernel
-----------------------------

In order to control your robot using ``libfranka``, the controller program on the workstation
PC must run with `real-time priority` under a ``PREEMPT_RT`` kernel. The procedure of patching a
kernel to support ``PREEMPT_RT`` and creating an installation package is described by the
following online resources:

 * `Installing a Kernel with the RT Patch
   <http://home.gwu.edu/~jcmarsh/wiki/pmwiki.php%3Fn=Notes.RTPatch.html>`_
 * `Howto setup Linux with PREEMPT_RT properly
   <https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup>`_

.. _installation-real-time:

Allow user to set real-time permissions for its processes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the ``PREEMPT_RT`` kernel is installed and running, add a group named `realtime` and
add the user controlling your robot to this group::

    sudo addgroup realtime
    sudo usermod -a -G realtime $(whoami)

Afterwards, add the following limits to the `realtime` group in ``/etc/security/limits.conf``::

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400


Building from source
--------------------

While it should in principle be possible to build ``libfranka`` and the ``franka_ros`` components
on different operating systems and distributions, official support is currently only provided for
Ubuntu 16.04 LTS `Xenial Xerus` and ROS `Kinetic Kame`.

Building libfranka
^^^^^^^^^^^^^^^^^^

To build ``libfranka``, install the following dependencies from Ubuntu's package manager::

    sudo apt install build-essential cmake git libpoco-dev

Then, download the source code by cloning ``libfranka`` from
`GitHub <https://github.com/frankaemika/libfranka>`__::

    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka

In the source directory, create a build directory and run CMake::

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .


If a systemwide installation is desired, execute the following instructions::

    cd libfranka/build
    cpack
    sudo dpkg -i libfranka-*.deb

.. _installing_ros:

Building the ROS packages
^^^^^^^^^^^^^^^^^^^^^^^^^

This part is optional. If you want to control your robot using `ROS <http://www.ros.org/>`_ please
follow these instructions.

After `setting up ROS Kinetic <https://wiki.ros.org/kinetic/Installation/Ubuntu>`_, create a Catkin
workspace in a directory of your choice:

.. code-block:: shell

    cd /path/to/desired/folder
    mkdir -p catkin_ws/src
    cd catkin_ws
    source /opt/ros/kinetic/setup.sh
    catkin_init_workspace src

Then clone the ``franka_ros`` repository from `GitHub <https://github.com/frankaemika/franka_ros>`__
, install any missing dependencies and build the packages:

.. code-block:: shell

    git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
    # Install all missing dependencies
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys Franka
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
    source devel/setup.sh

If you installed ``libfranka`` systemwide, specifying ``Franka_DIR`` is not necessary.
