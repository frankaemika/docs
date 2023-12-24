Installation
============

.. important::
    Franka Matlab is based on the `Franka Control Interface (FCI) <https://frankaemika.github.io/docs/>`_ and 
    the `libfranka C++ interface <https://frankaemika.github.io/docs/libfranka.html>`_. 
    All the same 
    `system and network requirements <https://frankaemika.github.io/docs/requirements.html>`_  do therefore apply.

Linux System Setup
------------------

.. important::
    Make sure that you have installed the `matlab-support package <https://packages.ubuntu.com/search?keywords=matlab-support>`_ for your system with:

    .. code-block:: shell

        sudo apt install matlab-support

A system wide installation of libfranka, **as a Debian package**, is required for the Franka Matlab Toolbox.

Please follow the `libfranka installation instructions <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>`_ for building from source.

Do not forget to build and install the libfranka Debian package as well:

.. code-block:: shell

    $ cd libfranka/build
    $ cpack -G DEB
    $ sudo dpkg -i libfranka*.deb

.. important::
    Make sure that the Real Time Kernel is properly installed as described in the 
    `libfranka documentation <https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_.

.. hint::
    before proceeding with the Franka Matlab Toolbox, it would be a good practice to execute a couple of the libfranka examples, under the build/examples folder, in order to ensure that the libfranka 
    installation has been succesful and that the system can operate under the Real-Time control constraints.

Windows System Setup
--------------------

.. warning::
    Support for Windows is still experimental. Issues could arise due to lack of hard Real-Time scheduling capabilities of the generic Windows distributions.

Please make sure first to install the Visual Studio 2017 Community Edition (English Version) on a Windows 10 PC.

Additionally the following software components must be installed for windows:

* git
* cmake
* vcpkg
* ninja

Make sure that the vcpkg & ninja paths are exposed through the PATH environment variable. 
You can modify the PATH environment variable in Windows 10:

1. Open the Start Search, type in “env”, and choose “Edit the system environment variables”. 
2. Click the “Environment Variables” button. 
3. Under the “System Variables” section, find the row with “Path” in the first column, and click edit. 
4. Add the vcpkg & ninja paths, like e.g C:\\Users\\{user name}\\vcpkg & C:\\Users\\{user name}\\ninja
5. Verify by opening a terminal and evaluating the `vcpkg` and `ninja` commands.  

You can then install the 64bit versions of eigen3 and poco packages:

.. code-block:: shell

    vcpkg install eigen3:x64-windows
    vcpkg install poco[netssl]:x64-windows

You can now install libfranka in Windows: 

After restarting the x64 Native Tools Command Prompt for VS 2017 do:

.. code-block:: winbatch

    cd C:\Users\{user name}
    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka
    mkdir build
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_TOOLCHAIN_FILE=C:\Users\{user name}\vcpkg\scripts\buildsystems\vcpkg.cmake -G Ninja ..
    ninja

.. important::
    Make sure that you add the libfranka\\build folder also added in the PATH environment variable as well! 
    This is a necessary requirement for generating and building code with the Franka Matlab Toolbox.

Franka Matlab Toolbox Add-On Installation & License Management
--------------------------------------------------------------

For installing the Franka Matlab Toolbox, start by either double clicking on the franka_matlab.mltbx that you've received or by drag-and-dropping it to the current Matlab Command Window.

After the Franka Matlab Toolbox has been added as an Add-On the license number needs to be installed as well. 

For receiving the license number please contact Franka Emika and provide your Matlab license number. 

You can find you license number by simply typing in Matlab:

.. code-block:: shell

    license;

For installing the license you can type:

.. code-block:: shell

    franka_matlab_toolbox_install('your license number');

In case a removal of the Franka Matlab Toolbox is desired, we recommend starting by uninstalling the current license first with:

.. code-block:: shell

    franka_matlab_toolbox_uninstall();
