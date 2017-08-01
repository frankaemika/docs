.. highlight:: shell

Installation instructions
=========================

In this chapter, the steps to set up a realtime kernel on Ubuntu as well as building
``libfranka`` and ``franka_ros`` are described.


Setting up a realtime kernel
----------------------------

In order to control the FRANKA ARM using ``libfranka``, the controller program on the workstation
PC must run with `real-time priority` under a ``PREEMPT_RT`` kernel. The procedure of patching a
kernel to support ``PREEMPT_RT`` and creating an installation package is described by the
following online resources:

 * `Installing a Kernel with the RT Patch
   <http://home.gwu.edu/~jcmarsh/wiki/pmwiki.php%3Fn=Notes.RTPatch.html>`_
 * `Howto setup Linux with PREEMPT_RT properly
   <https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup>`_


Allow user to set realtime permissions for its processes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the ``PREEMPT_RT`` kernel is installed and running, add a group named `realtime` and
add the user controlling the robot::

    sudo addgroup realtime
    sudo adduser $(whoami) realtime


Afterwards, add the limits to the `realtime` group in ``/etc/security/limits.conf``::

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

To build ``libfranka`` and the API documentation install the following dependencies from
Ubuntu's package manager::

    sudo apt install build-essential cmake doxygen git libpoco-dev

Then, download the source code by cloning ``libfranka`` from
`GitHub <https://github.com/frankaemika/libfranka>`__::

    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka

In the source directory, create a build directory and run CMake::

    mkdir build
    cd build
    cmake ..
    cmake --build .


If a systemwide installation is desired, execute the following instructions::

    cd libfranka/build
    cpack
    sudo dpkg -i libfranka-*.deb


Building the ROS packages
^^^^^^^^^^^^^^^^^^^^^^^^^

After `setting up ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_, download
``franka_ros`` from `GitHub <https://github.com/frankaemika/franka_ros>`__ and put it into
the Catkin workspace::

    cd catkin_ws/src
    git clone --recursive https://github.com/frankaemika/franka_ros

In the Catkin workspace, execute ``catkin_make`` with the path to the ``libfranka`` build
directory. If you installed ``libfranka`` systemwide, specifying``Franka_DIR`` is not
necessary.

.. code-block:: shell

    cd catkin_ws
    catkin_make -D Franka_DIR=/path/to/libfranka/build
