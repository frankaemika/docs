Installation
============

For installing the Franka MATLAB Toolbox just drag-and-drop the franka_toolbox.mltbx
file to your current MATLAB Command Window and follow the installation instructions.

Alternatively, your can use the `MATLAB Add-On Manager <https://www.mathworks.com/help/matlab/matlab_env/get-add-ons.html>`_.

License Management & Activation
-------------------------------

For generating a license number a unique identifier for your system is required. 

Please generate the unique identifier by executing the following command in your MATLAB environment:

.. code-block:: shell

    >> franka_toolbox_uid_gen()

Please contact Franka Robotics and provide the generated number in order to retrieve your license. 

You can then finalize the installation by executing: 

.. code-block:: shell

    >> franka_toolbox_install('<your license number>');

In case you are working with a first generation FER Robot please provide the additional argument `'fer'`:

.. code-block:: shell

    >> franka_toolbox_install('<your license number>','fer');

At this point you should be good to go!

Check the "Getting Started" section bellow for start exploring the Toolbox. 

Linux System - Native Deployment & Execution
--------------------------------------------

In case you are planning to utilize your Linux Host PC for running the Franka Toolbox application natively, without the help of the Franka AI Companion please make sure:

1. You have Real-Time Kernel setup for your system. See the page `Setting up the real-time kernel <https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_.
2. You have libfranka installed and it's dependencies. See the page `Building libfranka <https://frankaemika.github.io/docs/installation_linux.html#building-libfranka>`_.

Make sure that the Debian package of libfranka is properly built and installed system-wide.

Alternatively you can proceed with a local (non-system-wide) libfranka auto-installation with the help of the following command:

.. code-block:: shell

    >> franka_toolbox_libfranka_install('<desired libfranka version>');