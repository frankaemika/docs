Installation
============

.. hint::
    The Franka Matlab Toolbox is based on the `Franka Control Interface (FCI) <https://frankaemika.github.io/docs/>`_ and 
    the `libfranka C++ interface <https://frankaemika.github.io/docs/libfranka.html>`_. 
    All the same 
    `system and network requirements <https://frankaemika.github.io/docs/requirements.html>`_  do therefore apply.

Linux System Setup
------------------

.. important::
    For Linux system we higly recommend installing the `matlab-support package <https://packages.ubuntu.com/search?keywords=matlab-support>`_:

    .. code-block:: shell

        sudo apt install matlab-support

Make sure that the following dependencies are installed:

    .. code-block:: shell

        sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

You can either let the Franka Matlab Toolbox auto-install the libfranka locally or you can proceed with
a system-wide libfranka.*.deb installation. We recommend the former.

.. important::
    Make sure that the Real-Time Kernel is properly installed as described in the 
    `libfranka documentation <https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_.
    
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

Franka Matlab Toolbox Add-On Installation & License Management
--------------------------------------------------------------

For installing the Franka Matlab Toolbox either drag-and-drop the franka_matlab.mltbx
to the current Matlab Command Window or you can use the Matlab Add-On manager.

After this process is complete simply follow the instructions in the Getting Started guided 
which should have been opened after the Franka Matlab Toolbox Add-on installation.

In short you will need to generate a unique identifier for you PC by executing:

.. code-block:: shell

    franka_matlab_toolbox_uid_gen();

Please then send this unique identifier to Franka Robotics for receiving a License Number for
the Franka Matlab Toolbox.

You can then proceed with the final installation step, by executing: 

.. code-block:: shell

    franka_matlab_toolbox_install('franka matlab toolbox license number as a string',['fr3' or 'fer']);

That's it the Franka Matlab Toolbox should be ready. 

Get a glimpse of what the capabilities of the Toolbox are by navigating through the examples provided with the Toolbox:

.. code-block:: shell

    franka_matlab_toolbox_examples();
