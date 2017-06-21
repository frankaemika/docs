.. highlight:: shell

Build Instructions
==================

While it should in principle be possible to build ``libfranka`` on different operating systems and distributions, official support is only provided for Ubuntu 16.04 LTS `Xenial Xerus`. First, make sure, that you have installed a RT kernel.

Setting up a realtime kernel
----------------------------

In order to control your Franka using ``libfranka``, your controller program must run with `real-time priority` under a ``RT-PREEMPT`` kernel. Therefore, you must first apply the ``RT-PREEMPT`` patch to a matching Linux kernel and then build and install the patched kernel. The procedure for patching the kernel and creating an installation package in the following online resources: 

* `Installing a Kernel with the RT Patch <http://home.gwu.edu/~jcmarsh/wiki/pmwiki.php%3Fn=Notes.RTPatch.html>`_  
* `HOWTO setup Linux with PREEMPT_RT properly <https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup>`_


Building on Ubuntu
---------------------------

First, install the necessary dependencies for building the library and API documentation from Ubuntu's package manager::

    sudo apt install build-essential cmake libpoco-dev doxygen

In the ``libfranka`` source directory, create a build directory and run CMake::

    mkdir build
    cd build
    cmake ..
    cmake --build .

If the build was successful make sure, that you use a RT kernel.


After you booted into a RT patched kernel and built ``libfranka``, you can move on to the :doc:`tutorial`.