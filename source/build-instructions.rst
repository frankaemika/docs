.. highlight:: shell

Build Instructions
==================

While it should in principle be possible to build ``libfranka`` on different operating systems and distributions, official support is only provided for Ubuntu 16.04 LTS `Xenial Xerus`.

Building on Ubuntu 16.04
------------------------

First, install the necessary dependencies for building the library and API documentation from Ubuntu's package manager::

    sudo apt install build-essential cmake libpoco-dev doxygen

In the ``libfranka`` source directory, create a build directory and run CMake::

    mkdir build
    cd build
    cmake ..
    cmake --build .

If the build was successful, you can move on to the :doc:`tutorial`.

Setting up a realtime kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In order to control your Franka using ``libfranka``, your controller program must run with `real-time priority` under an ``RT-PREEMPT`` kernel. Therefore, you must first apply the ``RT-PREEMPT`` patch, and then build and install the patched kernel.
As building the Linux kernel is quite time consuming, it is advised that you make a deb package so the installation can be seamlessly done on multiple machines. The procedure for patching the kernel and creating an installation package can be found here: http://home.gwu.edu/~jcmarsh/wiki/pmwiki.php%3Fn=Notes.RTPatch.html



