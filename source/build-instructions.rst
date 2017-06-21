.. highlight:: shell

Build Instructions
==================

While it should in principle be possible to build ``libfranka`` on different operating systems and distributions, official support is only provided for Ubuntu 16.04 LTS `Xenial Xerus`. First, make sure, that you have installed a RT kernel.


Setting up a realtime kernel
----------------------------

In order to control your Franka using ``libfranka``, your controller program must run with `real-time priority` under a ``RT-PREEMPT`` kernel. Therefore, you must first apply the ``RT-PREEMPT`` patch to a matching Linux kernel and then build and install the patched kernel. The procedure for patching the kernel and creating an installation package in the following online resources: 

* `Installing a Kernel with the RT Patch <http://home.gwu.edu/~jcmarsh/wiki/pmwiki.php%3Fn=Notes.RTPatch.html>`_  
* `HOWTO setup Linux with PREEMPT_RT properly <https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup>`_


Allow user to set realtime permissions for its processes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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


Building libfranka on Ubuntu
--------------------------------

First, install the necessary dependencies for building the library and API documentation from Ubuntu's package manager::

    sudo apt install build-essential cmake libpoco-dev doxygen

In the ``libfranka`` source directory, create a build directory and run CMake::

    mkdir build
    cd build
    cmake ..
    cmake --build .

If the build was successful make sure, that you use a RT kernel.


After you booted into a RT patched kernel and built ``libfranka``, you can move on to the :doc:`tutorial`.

Installing ROS on Ubuntu
----------------------------

Please have a look at the `ROS Wiki <http://wiki.ros.org>`_:

 * `Install ROS <http://wiki.ros.org/lunar/Installation/Ubuntu>`_
 * `Getting Started <http://wiki.ros.org/ROS/StartGuide>`_