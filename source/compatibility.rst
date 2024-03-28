Compatible versions
===================

.. _compatibility-libfranka:

Compatibility with libfranka
----------------------------

Various versions of compatible components are available. 
The table below offers an overview, with a recommendation to utilize up-to-date versions whenever possible. 
The symbol '>= ' indicates that compatibility with newer robot system versions has not been tested, 
implying that compatibility is not guaranteed (e.g., libfranka 0.2.0 may not be compatible with robot system version 4.0.0).

The Robot system versions 2.x.x are not listed in the table below, but they are included as compatible with Robot system version >= 1.3.0. 
Therefore, they are compatible with libfranka versions 0.4.0 and 0.5.0.

+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| Robot system version | libfranka version | Robot / Gripper |franka_ros2 version| Ubuntu / ROS 2    | franka_ros version| Ubuntu / ROS 1 |
|                      |                   | Server version  |                   |                   |                   |                |
+======================+===================+=================+===================+===================+===================+================+
| >= 5.7.0 (FR3)       | >= 0.13.4         | 8 / 3           |                   | 22.04 / humble    |                   | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 5.5.0 (FR3)       | >= 0.13.3         | 7 / 3           |                   | 22.04 / humble    | >= 0.10.0         | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 5.2.0 (FR3)       | >= 0.13.0         | 6 / 3           | >= 0.1.8          | 22.04 / humble    | >= 0.10.0         | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 5.2.0 (FR3)       | >= 0.12.1         | 6 / 3           | >= 0.1.6          | 22.04 / humble    | >= 0.10.0         | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 5.2.0 (FR3)       | >= 0.11.0         | 6 / 3           | >= 0.1.3          | 22.04 / humble    | >= 0.10.0         | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 5.2.0 (FR3)       | >= 0.10.0         | 6 / 3           | >= 0.1.0          | 22.04 / humble    | >= 0.10.0         | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 4.2.1 (FER)       | >= 0.9.1 < 0.10.0 | 5 / 3           |                   |                   | >= 0.8.0          | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 4.0.0 (FER)       | >= 0.8.0          | 4 / 3           |                   |                   | >= 0.8.0          | 20.04 / noetic |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 3.0.0 (FER)       | 0.7.1             | 3 / 3           |                   |                   | 0.7.0             | 18.04 / melodic|
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 1.3.0 (FER)       | 0.5.0             | 3 / 2           |                   |                   | 0.6.0             | 16.04 / kinetic|
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 1.2.0 (FER)       | 0.3.0             | 2 / 2           |                   |                   | 0.4.0             | 16.04 / kinetic|
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+
| >= 1.1.0 (FER)       | 0.2.0             | 2 / 1           |                   |                   |                   |                |
+----------------------+-------------------+-----------------+-------------------+-------------------+-------------------+----------------+

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
