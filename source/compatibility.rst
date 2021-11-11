Compatible versions
===================

Compatibility with libfranka
----------------------------

For every robot system version there are different libfranka versions compatible.
The following table provides an overview. Furthermore, the recommended Ubuntu and
ROS versions are listed. It is recommended to always work with the up-to-date versions.
The ``>=`` indicates, that it is not tested against newer robot system versions meaning
compatibility is not guaranteed (i.e. libfranka 0.2.0 is not compatible with the robot
system version 4.0.0). The robot system versions 2.x.x are not listed in the table below,
but are included compatible with the robot system version >= 1.3.0. Therefore, these are
compatile with the libfranka version 0.4.0 and 0.5.0. Libfranka version 0.8.0 was also
tested against Ubuntu 18.04 and therefore, is compatible with ROS Melodic.

+-------------------+-----------------------+--------------+----------------------+-------------+
| libfranka version | Robot system version  | Release date | Ubuntu               | ROS version |
+===================+=======================+==============+======================+=============+
| 0.8.0             | >= 4.0.0              | 29.04.2020   | 20.04 (Focal Fossa)  | ROS Noetic  |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.7.1             | >= 3.1.0              | 08.07.2019   | 18.04 (Bionic)       | ROS Melodic |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.7.0             | >= 3.0.0              | 05.07.2019   | 18.04 (Bionic)       | ROS Melodic |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.6.0             | >= 3.0.0              | 06.02.2019   | 18.04 (Bionic)       | ROS Melodic |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.5.0             | >= 1.3.0              | 08.08.2018   | 16.04 (Xenial Xerus) | ROS Kinetic |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.4.0             | >= 1.3.0              | 21.06.2018   | 16.04 (Xenial Xerus) | ROS Kinetic |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.3.0             | >= 1.2.0              | 22.02.2018   | 16.04 (Xenial Xerus) | ROS Kinetic |
+-------------------+-----------------------+--------------+----------------------+-------------+
| 0.2.0             | >= 1.1.0              | 29.01.2018   |                      |             |
+-------------------+-----------------------+--------------+----------------------+-------------+


Windows usage is experimental. For this, libfranka version 0.6.0 is needed and thus, a
robot system version of at least 3.0.0 is needed. For usage under Windows, compatible
libfranka and robot system versions are needed as specified in the table above.

Compatiblity with the kernel
----------------------------

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
