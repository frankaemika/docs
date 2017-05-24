Introduction
============

``libfranka`` provides a C++ interface for low-level control of FRANKA. With this library, it is possible to:

* Retrieve information about the current state of FRANKA, e.g. current end effector pose or joint angles.
* Execute motions by giving joint positions, joint velocities, Cartesian poses, or Cartesian velocities.
* Perform torque control.
* Send commands to FRANKA, e.g. to set collision sensitivity.
* Calculate forward kinematics and other model properties from the current robot state.

.. important::

    Data is sent to FRANKA over the network with a frequency of :math:`1 KHz`. Therefore, a good network connection is vital!
