
FRANKA Control Interface Documentation
======================================

.. todolist::

FRANKA Control Interface (FCI) consists of several open source components which we provide on
`GitHub <https://github.com/frankaemika>`_. We welcome contributions and suggestions for
improvements.

These components are:

 * ``libfranka``, a C++ library that provides low-level control of the `Arm and Hand
   <https://franka.de>`_. Its source code is available at https://github.com/frankaemika/libfranka.
 * ``franka_ros``, our `ROS integration <http://wiki.ros.org/franka>`_, including ROS Control and
   MoveIt!. The source code is available at https://github.com/frankaemika/franka_ros.

The source code of this documentation is also `available online
<https://github.com/frankaemika/docs>`_.


.. important::
    Before you start using the research interface, please read through the documents shipped with
    the robot and the :doc:`minimum system and network requirements <requirements>` chapter.

.. important::
    In this documentation, the terms robot and gripper simply refer to the **Arm** and **Hand**
    that are shipped as part of your **Panda**.

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

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
