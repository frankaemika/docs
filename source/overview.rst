Overview
========

.. figure:: _static/overview.png
    :align: center
    :alt: alternate text
    :figclass: align-center

    Schematic overview of the components.

The FRANKA research interface allows a fast and direct low-level bidirectional connection to
the FRANKA ARM and gripper. It provides feedback-data and enables controlling the robot.
By sending real-time control values you can execute a custom robot behavior:

 * Joint torque control.
 * Desired joint position or velocity command.
 * Desired Cartesian position or velocity command.

Further, you get access to the following feedback data:

 * Measured joint data, like the position, velocity and torque.
 * Low-level desired joint goals.
 * Estimation of externally applied torques and wrenches.
 * Various collision and contact information.

.. important::

    While the research interface is active you have full control of the robot, but you cannot use
    FRANKA DESK. This means that you cannot use APPS and the RI at the same time.


It consists of two components::

* libfranka
* franka_ros (ROS support)


``libfranka`` provides a **C++ interface** which is run remotely on a workstation PC. The
connection to FRANKA CONTROL is established via a standard Ethernet connection. The interface
provides high-speed measurements, internal data of the robot and the gripper. Further, it accepts
parameters and control values at an update frequency of up to 1 kHz. With this library, it is
possible to:

* Retrieve information about the current state of FRANKA, e.g. current end effector pose, joint
  angles or the gripper status.
* Execute motions by giving joint positions, joint velocities, Cartesian poses, or Cartesian
  velocities.
* Perform torque control.
* Send commands to FRANKA, e.g. to set collision sensitivity, set additional loads or the
  joint/Cartesian stiffness.
* Calculate forward kinematics and other model properties from the current robot state.
* Control the FRANKA gripper.
* Use the robot model library which provides the following:

  - Forward kinematics of all joints.
  - Jacobian matrix of all joints.
  - Dynamics: inertia matrix, Coriolis and centrifugal vector and gravity vector.

.. important::

    Data is sent over the network with a frequency of 1 kHz. Therefore, a good network
    connection is vital!

``franka_ros`` connects FRANKA with the entire ROS ecosystem. It provides functionality equivalent to 
``libfranka`` inside of ROS. Additionally, it includes a URDF model of the FRANKA ARM and gripper, 
which allows for visualization (e.g. RViz) and kinematic simulations. **MoveIt!** integration makes it 
easy to move the robot, and examples show how to control FRANKA from ROS.
