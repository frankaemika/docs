Installation on Windows
=======================

Starting from ``libfranka`` >= 0.6.0, **experimental** Windows support is provided.
This chapter describes how to install ``libfranka`` on Windows.
``franka_ros`` is not supported on Windows.

.. note::

 ``libfranka`` is currently only supported for Windows 10 and
 Visual Studio 2017.


Building from source
--------------------

To build ``libfranka``, install the following dependencies:

* `Eigen3 <http://eigen.tuxfamily.org>`__

* `Poco <https://pocoproject.org/>`__

.. hint::

 Both can be easily installed with `vcpkg <https://docs.microsoft.com/en-us/cpp/vcpkg>`__ via cmd prompt:

 .. code-block:: shell

        cd /path/to/vcpkg
        vcpkg install eigen3
        vcpkg install poco

Download the source code by cloning ``libfranka`` from `GitHub <https://github.com/frankaemika/libfranka>`__:

.. code-block:: shell

 git clone --recursive https://github.com/frankaemika/libfranka

By default, this will check out the newest release of ``libfranka``. If you want to build
a particular version of ``libfranka`` instead, check out the corresponding Git tag::

 git checkout <version>
 git submodule update

.. important::
 Only ``libfranka`` >= 0.6.0 has Windows support!

To build ``libfranka`` with Visual Studio open it as a CMake Project.
Choose **File** > **Open** > **CMake** > **/path/to/libfranka/CMakeLists.txt**.

Generate the ``CMakeSettings.json``, which contains the CMake project settings.
Select **CMake** > **Cache** > **Generate** > **CMakeSettings.json**. The file will be placed in
your main ``libfranka`` directory.

The next step is to solve the build dependencies.
Since Windows does not have default binary directories, make sure the compiler is able to find
the required dependencies. This could be done either by copying all needed .dll libries into
the chosen build root determined by ``CMakeSettings.json`` or using cmakeCommandArgs.
Open **CMake** > **Change CMake settings** > **libfranka** and add

.. code-block:: json

 {
 "..."
 "cmakeCommandArgs": "-DPoco_DIR=/Path/To/Poco/CMake/Config -DEigen3_DIR=/Path/To/Eigen/CMake/Config",
 "..."
 }

Choose **CMake** > **Build** to build ``libfranka`` into the build directory,
determined in ``CMakeSettings.json``

.. hint::

 Alternatively you can build libfranka on `Developer Command Propmpt for VS`:

 .. code-block:: shell

    cd /path/to/libfranka
    mkdir build
    cd build
    cmake -DPoco_DIR=/Path/To/Poco/CMake/Config -DEigen3_DIR=/Path/To/Eigen/CMake/Config -G Ninja ..
    ninja
