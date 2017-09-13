Overview
========

.. figure:: _static/overview.png
    :align: center
    :alt: alternate text
    :figclass: align-center

    Schematic overview of the components.

The FRANKA Control Interface (FCI) allows a fast and direct low-level bidirectional connection
to the Arm and Hand. It provides the current status of the robot and enables its direct control.
You can execute custom trajectories by sending real-time control values:

 * Joint torque control.
 * Desired joint position or velocity command.
 * Desired Cartesian position or velocity command.

Furthermore, you get access to the following feedback data:

 * Measured joint data, such as the position, velocity, and torque.
 * Low-level desired joint goals.
 * Estimation of externally applied torques and wrenches.
 * Various collision and contact information.

.. important::

    While the FRANKA Control Interface is active you have full control of the robot.
    However, you `cannot` use Desk and FRANKA Control Interface simultaneously.
    This means that you `cannot` utilize Apps and FRANKA Control Interface at the same time.


It consists of two components:

* ``libfranka``
* ``franka_ros`` (ROS support)


``libfranka`` provides a **C++ interface** which is run remotely on a workstation PC. The
connection to Control is established via a standard Ethernet connection. The interface
provides high-speed measurements, internal data of the robot and the gripper. Further, it accepts
parameters and control values at an update frequency of up to 1 kHz. With this library, it is
possible to:

* Retrieve information about the current state of the robot, e.g. current end effector pose, joint
  angles or the gripper status.
* Execute motions by giving joint positions, joint velocities, Cartesian poses, or Cartesian
  velocities.
* Perform torque control.
* Send commands to the robot, e.g. to set collision sensitivity, set additional loads or the
  joint/Cartesian stiffness.
* Calculate forward kinematics and other model properties from the current robot state.
* Control the gripper.
* Use the robot model library which provides the following:

  - Forward kinematics of all robot joints.
  - Jacobian matrix of all robot joints.
  - Dynamics: inertia matrix, Coriolis and centrifugal vector and gravity vector.

.. important::

    Data is sent over the network with a frequency of 1 kHz. Therefore, a good network
    connection is vital!

``franka_ros`` connects Panda with the entire ROS ecosystem. It provides functionality equivalent
to ``libfranka`` inside of ROS. Additionally, it includes a URDF model of the Arm and
Hand, which allows for visualization (e.g. RViz) and kinematic simulations. **MoveIt!**
integration makes it easy to move the robot and control the gripper, and the provided examples show
you how to control your Panda from ROS.
