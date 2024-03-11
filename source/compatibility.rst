Compatible versions
===================

.. _compatibility-libfranka:

Compatibility with libfranka
----------------------------

Different versions of open source components compatible.
The following table provides an overview. It is recommended to always work with up-to-date versions.
The ``>=`` indicates that it is not tested against newer robot system versions, meaning
compatibility is not guaranteed (i.e. libfranka 0.2.0 is not compatible with the robot
system version 4.0.0).

The Robot system versions 2.x.x are not listed in the table below,
but are included as compatible with the Robot system version >= 1.3.0. Therefore, these are
compatible with the libfranka version 0.4.0 and 0.5.0.

+----------------------+-------------------+-----------------+--------------------+-----------------+
| Robot system version | libfranka version | Robot / Gripper | franka_ros version | Ubuntu / ROS    |
|                      |                   | Server version  |                    |                 |
+======================+===================+=================+====================+=================+
| >= 5.2.0 (FR3)       | >= 0.10.0         | 6 / 3           | >= 0.10.0          | 20.04 / noetic  |
+----------------------+-------------------+-----------------+--------------------+-----------------+
| >= 4.2.1 (FER)       | >= 0.9.1 < 0.10.0 | 5 / 3           | >= 0.8.0           | 20.04 / noetic  |
+----------------------+-------------------+-----------------+--------------------+-----------------+
| >= 4.0.0 (FER)       | >= 0.8.0          | 4 / 3           | >= 0.8.0           | 20.04 / noetic  |
+----------------------+-------------------+-----------------+--------------------+-----------------+
| >= 3.0.0 (FER)       | 0.7.1             | 3 / 3           | 0.7.0              | 18.04 / melodic |
+----------------------+-------------------+-----------------+--------------------+-----------------+
| >= 1.3.0 (FER)       | 0.5.0             | 3 / 2           | 0.6.0              | 16.04 / kinetic |
+----------------------+-------------------+-----------------+--------------------+-----------------+
| >= 1.2.0 (FER)       | 0.3.0             | 2 / 2           | 0.4.0              | 16.04 / kinetic |
+----------------------+-------------------+-----------------+--------------------+-----------------+
| >= 1.1.0 (FER)       | 0.2.0             | 2 / 1           |                    |                 |
+----------------------+-------------------+-----------------+--------------------+-----------------+

`Robot version line 17
<https://github.com/frankaemika/libfranka-common/blob/master/include/research_interface/robot/service_types.h>`_
and `Gripper version line 17
<https://github.com/frankaemika/libfranka-common/blob/master/include/research_interface/gripper/types.h>`_
are part of libfranka-common repository, a submodule of libfranka repository.

Franka MATLAB® compatible versions are located :ref:`here<compatibility-franka-matlab>`.

.. caution::
    Franka Robotics currently does not provide any support for Windows or Arm

Compatibility with the kernel
-----------------------------

There are different kernels, which are compatible with different Ubuntu system versions.
The following table gives an overview of recommended Kernels.

+----------------+----------------------+
| Kernel version | Ubuntu               |
+================+======================+
| 5.9.1          | 20.04 (Focal Fossa)  |
+----------------+----------------------+
| 5.4.19         | 18.04 (Bionic)       |
+----------------+----------------------+
| 4.14.12        | 16.04 (Xenial Xerus) |
+----------------+----------------------+
