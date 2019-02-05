Installation Windows
====================

This chapter describes how to install ``libfranka``, either
as binary packages or by building from source. Since ROS is not supported for Windows, you 
wont be able to use ``franka_ros`` on a Windwos workstation.

.. note::

 ``libfranka`` is currently only provided for Windwos 10 and Visual Studio Professional/Express 2014 or higher.

Binary package setup
--------------------

**TODO**


Building from source
--------------------

To build ``libfranka``, install the following dependencies:

* `Eigen3 <http://eigen.tuxfamily.org/index.php?title=Main_Page>`__

* `Poco <https://pocoproject.org/>`__ 

.. hint::

 Both can be easily installed with `vcpkg <https://docs.microsoft.com/en-us/cpp/vcpkg?view=vs-2017>`__ via cmd prompt:

 .. code-block:: shell

        cd /path/to/vcpkg
        vcpkg install eigen3
        vcpkg install poco

Download the source code by cloning ``libfranka`` from `GitHub <https://github.com/frankaemika/libfranka>`__:

.. code-block:: shell

 git clone --recursive https://github.com/frankaemika/libfranka

By default, this will check out the newest release of ``libfranka``. If you want to build a particular version of
``libfranka`` instead, check out the corresponding Git tag::

 git checkout <version>
 git submodule update

.. important::
 Only libfranka from release 0.5.0 has Windows support! 

To build ``libfranka`` with Visiual Studio open it as a CMake Project.
Choose **File** > **Open** > **CMake** > **/path/to/libfranka/CMakeLists.txt**.

Generate the ``CMakeSettings.json``, which contains the CMake project settings.
Select **CMake** > **Cache** > **Generate** > **CMakeSettings.json**. The file will be placed in your
main ``libfranka`` directory.

The next step is to solve the build dependencies. Since Windows does not have default binary directories, make sure the compiler is able to find the dependencies.
We did this either by copying all needed .dll libries into the chosen build root determined by ``CMakeSettings.json`` or using cmakeCommandArgs.
Open **CMake** > **Change CMake settings** > **libfranka** and add

.. code-block:: json

 {
 "..."
 "cmakeCommandArgs": "-DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake",
 "..."
 }

Choose **CMake** > **Build** to build ``libfranka`` into the build directory, determined in ``CMakeSettings.json``