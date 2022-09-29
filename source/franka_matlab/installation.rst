Installation
============

Franka Matlab can be downloaded from the `Franka World Hub <https://franka.world/resources>`_.

Installation on Linux
---------------------

.. important::
    Franka Matlab is based on the `Franka Control Interface (FCI) <https://frankaemika.github.io/docs/>`_ and
    specifically the `libfranka C++ interface <https://frankaemika.github.io/docs/libfranka.html>`_,
    therefore all the same `system and network requirements <https://frankaemika.github.io/docs/requirements.html>`_
    apply.

.. important::
    Also, make sure that the Real Time Kernel is properly installed as described in the
    `libfranka documentation <https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_.

Same as with libfranka the following dependencies are required:

.. code-block:: shell

    $ sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

Add the 1st level of the franka_matlab to the path, by right clicking on the franka_matlab folder and selecting Add to Path >
Selected Folders.

Alternatively, the following command can be executed:

.. code-block:: shell

    >> addpath(<franka_matlab full path>);

Initialize the project with:

.. code-block:: shell

    >> init_franka_matlab();

In case no libfranka build is found in the franka_matlab directory, the init script will try to install it automatically.
Just press `Y` and enter when prompted. By default the init function will try to install libfranka 0.10.0 which is compatible with the
FR3 robot. For installing the previous libfranka version you can initialize the project with:

.. code-block:: shell

    >> init_franka_matlab('0.9.2');

.. warning::
    In case you are installing libfranka 0.9.x it is also necessary to mex the Matlab API once:

    .. code-block:: shell

        >> mex_franka_matlab_library();

After the init script is completed you can start utilizing the Simulink and Matlab scripts for the Franka Emika robot.

.. hint::
    It is highly recommended to start with the provided Simulink demos before writing custom applications from scratch.

Installation on Windows
-----------------------

.. warning::
    Support for Windows is still experimental. Issues could arise due to a lack of real-time kernel capabilities.
    Make sure that your windows system is powerful enough and in top condition.

Please make sure first to install the Visual Studio 2017 community edition (English Version) on a Windows 10 PC.

Make sure that vcpkg is also installed.

.. important::
    Make sure that the vcpkg path is exposed through the PATH environment variable.
    You can modify the PATH environment variable in Windows 10:

    1. Open the Start Search, type in “env”, and choose “Edit the system environment variables”.
       Alternatively hit *Windows Key + R* at the same time to get command prompt. Then type 'sysdm.cpl',
       go to advanced and select Environmental Variables.
    2. Click the “Environment Variables” button.
    3. Under the “System Variables” section, find the row with “Path” in the first column, and click edit.
    4. Add the path to the vcpkg executable, like e.g C:\src\vcpkg
    5. Verify by opening a terminal and hitting `vcpkg`.

You can then install the 64bit! versions of eigen3 and poco packages:

.. code-block:: shell

    vcpkg install eigen3:x64-windows
    vcpkg install poco:x64-windows

Same as with linux you can execute in Matlab:

.. code-block:: shell

    >> addpath(<franka_matlab full path>);

Initialize the project with:

.. code-block:: shell

    >> init_franka_matlab();

or for the previous libfranka version:

.. code-block:: shell

    >> init_franka_matlab('0.9.2');

Press `Y` when prompted for automatically installing libfranka.