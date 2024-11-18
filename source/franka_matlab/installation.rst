Installation
============

Installation Methods
--------------------

Option 1: Direct Installation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Drag and drop the ``franka_toolbox.mltbx`` file into your MATLAB Command Window and follow the installation prompts.

Option 2: Programmatically
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: matlab

    uiopen('<path to your franka.mltbx file>', 1);

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
For Franka Research 3 robots:

.. code-block:: matlab

    franka_toolbox_install('<your_license_number>');

or for first-generation `FER` robots:

.. code-block:: matlab

    franka_toolbox_install('<your_license_number>', 'fer');


.. _libfranka_handling_options:

libfranka handling options for Target PC
----------------------------------------

libfranka
^^^^^^^^^
Starting with Franka Toolbox for MATLAB version 2.0.0, libfranka is included in the toolbox distribution. No separate installation is required.

Uninstall Toolbox
-----------------

1. Clean up installation:

.. code-block:: matlab

    franka_toolbox_uninstall();

2. Remove the toolbox using MATLAB Add-Ons Manager.