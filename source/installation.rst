.. highlight:: shell

Installation instructions
=========================

Setting up a realtime kernel
----------------------------

In order to control your FRANKA using ``libfranka``, your controller program must run with
`real-time priority` under a ``PREEMPT_RT`` kernel. Therefore, you must first apply the
``PREEMPT_RT`` patch to a matching Linux kernel and then build and install the patched kernel.
The procedure for patching the kernel and creating an installation package in the following online
resources:

 * `Installing a Kernel with the RT Patch
   <http://home.gwu.edu/~jcmarsh/wiki/pmwiki.php%3Fn=Notes.RTPatch.html>`_
 * `HOWTO setup Linux with PREEMPT_RT properly
   <https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup>`_


Allow user to set realtime permissions for its processes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, add a realtime group and add your user to it::

    sudo addgroup realtime
    sudo adduser $(whoami) realtime


Then, edit  ``/etc/security/limits.conf`` and add::

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400


Building from source
--------------------

While it should in principle be possible to build ``libfranka`` and the FRANKA ROS components on
different operating systems and distributions, official support is currently only provided for
Ubuntu 16.04 LTS `Xenial Xerus` and ROS `Kinetic Kame`.

Building libfranka
^^^^^^^^^^^^^^^^^^

Install the necessary dependencies for building the library and API documentation from Ubuntu's
package manager::

    sudo apt install build-essential cmake doxygen git libpoco-dev

Clone the ``libfranka`` source code from `GitHub <https://github.com/frankaemika/libfranka>`__::

    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka

In the source directory, create a build directory and run CMake::

    mkdir build
    cd build
    cmake ..
    cmake --build .


Building the ROS packages
^^^^^^^^^^^^^^^^^^^^^^^^^

If you have `set up ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_, download the
FRANKA ROS packages from `GitHub <https://github.com/frankaemika/franka_ros>`__ and put them into
your Catkin workspace::

    cd catkin_ws/src
    git clone --recursive https://github.com/frankaemika/franka_ros

In your Catkin workspace, execute ``catkin_make`` with the path to the
``libfranka`` build directory::

    cd catkin_ws
    catkin_make -D Franka_DIR=/path/to/libfranka/build
