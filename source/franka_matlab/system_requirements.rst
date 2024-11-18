System Requirements
===================

Host PC Requirements
--------------------

MATLAB Version - Host PC OS Compatibility
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Host PC is the machine responsible for running MATLAB and Simulink.

+-------------------------+-------------------+----------------------------------------------------------------------------------------------------------+
| Franka Toolbox Version  | MATLAB Version    | Host PC OS - Windows / Ubuntu Version                                                                    |
+=========================+===================+==========================================================================================================+
| 2.0.0                   | R2022a or newer   |  [`Matlab System Requirements <https://www.mathworks.com/support/requirements/previous-releases.html>`_] |
+-------------------------+-------------------+----------------------------------------------------------------------------------------------------------+

MATLAB Toolbox Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following Mathworks products are required: 

* `MATLAB <https://www.mathworks.com/products/matlab.html>`_ 
* `Simulink <https://www.mathworks.com/products/simulink.html>`_
* `Simulink Coder <https://www.mathworks.com/products/simulink-coder.html>`_
* `Matlab Coder <https://www.mathworks.com/products/matlab-coder.html>`_

Some of demos provided with the franka_matlab need the following toolboxes:

* `Stateflow <https://www.mathworks.com/products/stateflow.html>`_ (required for the Simulink Example grasp_object.slx)
* `Matlab Robotics Toolbox <https://www.mathworks.com/products/robotics.html>`_ (required for the MATLAB example pick_and_place_with_RRT.mlx)

MATLAB Coder Support Package for NVIDIA Jetson
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**For working with the Franka AI Companion & NVIDIA Jetson platforms** please download and install the `MATLAB Coder Support Package for NVIDIA Jetson and NVIDIA DRIVE Platforms <https://www.mathworks.com/matlabcentral/fileexchange/68644-matlab-coder-support-package-for-nvidia-jetson-and-nvidia-drive-platforms>`_.

Target PC - AI Companion
------------------------

Franka Toolbox for MATLAB ships with precompiled libfranka which requires the following system dynamic libraries:

.. _system_dependencies_precompiled_ai_companion:

precompiled libfranka system dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+--------------+------------------------+---------------------------------------------------+
| Franka Robot | libfranka Version      | System Dependency                                 |
+==============+========================+===================================================+
| FR3          | 0.14.0                 | libpoco-dev (1.8.0.1-1ubuntu4), pinocchio (2.7.1) |
+--------------+------------------------+---------------------------------------------------+
| FER          | 0.9.2                  | libpoco-dev (1.8.0.1-1ubuntu4)                    |
+--------------+------------------------+---------------------------------------------------+


To install the required version of libpoco-dev (1.8.0.1-1ubuntu4) on your system, you can use the following commands:

.. code-block:: shell

    sudo apt-get update
    sudo apt-get install libpoco-dev=1.8.0.1-1ubuntu4

For installing the required version of pinocchio (2.7.1), please refer to the `Pinocchio installation documentation <https://stack-of-tasks.github.io/pinocchio/download.html>`_.

.. warning::

    In case the system dependencies cannot be met for libfranka, you can build and install libfranka from source, system-wide or locally in the scope of the Franka Toolbox only.
    For handling options, see :ref:`libfranka handling options for Target PC<libfranka_handling_options>`.


Target PC - RT Linux Host
-------------------------

System requirements when the Host & Target PC are the same machine.

System Requirements
^^^^^^^^^^^^^^^^^^^

Make sure that your target PC configuration complies with the following documentation.

1. `Franka Robot System Compatibility Documentation <https://frankaemika.github.io/docs/compatibility.html>`_ .
2. `System Requirements <https://frankaemika.github.io/docs/requirements.html#>`_ .
3. `Setting Up the Real-Time Kernel <https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_.

The Franka Toolbox for MATLAB ships with prebuilt libfranka and other binaries which are built based on the following Target PC Ubuntu version:

+-------------------------+---------------------------------------------+
| Franka Toolbox Version  | Target PC Ubuntu Version (recommended)      |
+=========================+=============================================+
| 2.0.0                   | 22.04 LTS                                   |
+-------------------------+---------------------------------------------+

MATLAB Support Package for Linux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In case where there is a single Ubuntu Host & Target machine, please ensure that 
you have installed the `matlab-support package <https://packages.ubuntu.com/search?keywords=matlab-support>`_ 
in order for Matlab to reference the system dynamic libraries instead of the precompiled ones that it ships with:

.. code-block:: shell

    sudo apt install matlab-support

.. _system_dependencies_precompiled_rt_linux_host:

precompiled libfranka system dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Franka Toolbox comes with prebuilt libfranka which requires the following system dynamic libraries:

+--------------+------------------------+-------------------------------------------+
| Franka Robot | libfranka Version      | System Dependency                         |
+==============+========================+===========================================+
| FR3          | 0.14.0                 | libpoco-dev (1.11.0-3), pinocchio (3.0.0) |
+--------------+------------------------+-------------------------------------------+
| FER          | 0.9.2                  | libpoco-dev (1.11.0-3)                    |
+--------------+------------------------+-------------------------------------------+

To install the required version of libpoco-dev (1.11.0-3) on your system, you can use the following commands:

.. code-block:: shell

    sudo apt-get update
    sudo apt-get install libpoco-dev=1.11.0-3

For installing the required version of pinocchio (3.0.0), please refer to the `Pinocchio installation documentation <https://stack-of-tasks.github.io/pinocchio/download.html>`_.

.. warning::

    In case the system dependencies cannot be met for libfranka, you can build and install libfranka from source, system-wide or locally in the scope of the Franka Toolbox only.
    For handling options, see :ref:`libfranka handling options for Target PC<libfranka_handling_options>`.

