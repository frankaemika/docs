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

.. todo::

  Add realtime kernel docu
