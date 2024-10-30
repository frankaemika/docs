Installation
============

Installation Methods
--------------------

Option 1: Direct Installation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Drag and drop the ``franka_toolbox.mltbx`` file into your MATLAB Command Window and follow the installation prompts.

Option 2: Add-On Manager
^^^^^^^^^^^^^^^^^^^^^^^^
Install using the `MATLAB Add-On Manager <https://www.mathworks.com/help/matlab/matlab_env/get-add-ons.html>`_.


License Management & Activation
-------------------------------

1. Generate System Identifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Execute the following command in MATLAB to generate your system's unique identifier:

.. code-block:: matlab

    franka_toolbox_uid_gen()

2. Obtain License
^^^^^^^^^^^^^^^^^
Contact Franka Robotics with your generated identifier to receive your license number.

3. Activate License
^^^^^^^^^^^^^^^^^^^
For standard robots:

.. code-block:: matlab

    franka_toolbox_install('<your_license_number>');

For first-generation `FER` robots:

.. code-block:: matlab

    franka_toolbox_install('<your_license_number>', 'fer');

System Software Dependencies
----------------------------

MATLAB Support Package
^^^^^^^^^^^^^^^^^^^^^^
In case where there is a single Ubuntu Host & Target machine, please ensure that 
you have installed the `matlab-support package <https://packages.ubuntu.com/search?keywords=matlab-support>`_ 
in order for Matlab to reference the system dynamic libraries instead of the precompiled ones that it ships with:

.. code-block:: shell

    sudo apt install matlab-support

libfranka
^^^^^^^^^
Starting with Franka Toolbox for MATLAB version 2.0.0, libfranka is included in the toolbox distribution. No separate installation is required.

System packages
^^^^^^^^^^^^^^^
Install required system packages on your target Ubuntu machine:

.. code-block:: bash

    sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

pinocchio
^^^^^^^^^
Since version 2.0.0 and in the case of using the FR3 robot, an installation of the `pinocchio library <https://stack-of-tasks.github.io/pinocchio/download.html>`_ is required. 

Uninstallation
--------------

1. Clean up installation:

.. code-block:: matlab

    franka_toolbox_uninstall();

2. Remove the toolbox using MATLAB Add-Ons Manager.