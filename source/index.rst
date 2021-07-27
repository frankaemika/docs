Franka Control Interface Documentation
======================================

.. todolist::

The Franka Control Interface (FCI) can be accessed via several open source components which we
provide on `GitHub <https://github.com/frankaemika>`_. We welcome contributions and suggestions
for improvements.

These components are:

 * ``libfranka``, a C++ library that provides low-level control of Franka Emika research robots.
   Its source code is available at https://github.com/frankaemika/libfranka. API documentation is
   available at https://frankaemika.github.io/libfranka.
 * ``franka_ros``, our `ROS integration <https://wiki.ros.org/franka_ros>`_, including support for
   ROS Control and MoveIt!. It also contains ``franka_description``, a collection of URDF models and
   3D meshes that can be useful outside of ROS.
   The repository is available at https://github.com/frankaemika/franka_ros.

The source code of this documentation is also `available online
<https://github.com/frankaemika/docs>`_.

.. important::
    Before you start using the FCI, please read through the documents shipped with the robot and
    the :doc:`requirements` chapter.

.. note::
    Using a robot with system version 4.2.0 or higher requires to enable the FCI mode. To do that
    open Desk -> expand the menu in the sidebar -> press `'Activate FCI'`. Further information
    about Single Point of Control (SPoC) can be found in the manual shipped with the robot.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   overview
   requirements
   installation_linux
   installation_windows
   getting_started
   libfranka
   libfranka_changelog
   franka_ros
   franka_ros_changelog
   control_parameters
   troubleshooting
   faq
