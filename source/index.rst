
FRANKA Control Interface Documentation
======================================

.. todolist::

The FRANKA Control Interface (FCI) consists of several open source components which we provide on
`GitHub <https://github.com/frankaemika>`_. We welcome contributions and suggestions for
improvements.

These components are:

 * ``libfranka``, a C++ library that provides low-level control of a `FRANKA EMIKA
   <https://franka.de>`_ research robot. Its source code is available at
   https://github.com/frankaemika/libfranka.
 * ``franka_ros``, our `ROS integration <https://wiki.ros.org/franka_ros>`_, including support for
   ROS Control and MoveIt!. The source code is available at
   https://github.com/frankaemika/franka_ros.

The source code of this documentation is also `available online
<https://github.com/frankaemika/docs>`_.


.. important::
    Before you start using the FCI, please read through the documents shipped with the robot and
    the :doc:`minimum system and network requirements <requirements>` chapter.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   overview
   requirements
   installation
   tutorial
   libfranka
   ros_introduction
   troubleshooting
