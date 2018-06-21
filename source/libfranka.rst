libfranka
=========

Before continuing with this chapter, please :doc:`install or compile libfranka <installation>`.
API documentation for the latest version of ``libfranka`` is available at
https://frankaemika.github.io/libfranka.

.. figure:: _static/libfranka-architecture.png
    :align: center
    :figclass: align-center

    Schematic overview

``libfranka`` is the C++ implementation of the client side of the FCI. It handles the network
communication with Control and provides interfaces to easily:

 * execute **non-realtime commands** to control the Hand and configure Arm parameters.
 * execute **realtime commands** to run  your own 1 kHz control loops.
 * read the **robot state** to get sensor data at 1 kHz.
 * access the **model library** to compute your desired kinematic and dynamic parameters.


Non-realtime commands
---------------------

Non-realtime commands are blocking, TCP/IP-based and always executed `outside` of any realtime
control loop. They encompass all of the Hand commands and some configuration-related commands
for the Arm.

.. figure:: _static/fci-architecture-non-realtime.png
    :align: center
    :figclass: align-center

    Non-realtime commands for both Arm and Hand.

The most relevant ones for the Hand are

 * ``homing`` which calibrates the maximum grasping width of the Hand.
 * ``move``, ``grasp`` and ``stop``, to move or grasp with the Hand.
 * ``readOnce``, which reads the Hand state.

Concerning the Arm, some useful non-realtime commands are:

 * ``setCollisionBehavior`` which sets the contact and collision detection thresholds.
 * ``setCartesianImpedance`` and ``setJointImpedance`` which set the impedance parameters
   for the internal Cartesian impedance and internal joint impendace controllers.
 * ``setEE``, ``setK`` and ``setLoad`` which set end effector and load parameters.
 * ``automaticErrorRecovery`` that clears any command or control exception that previously
   happened in the robot.
 * ``setFilters`` which sets the cutoff frequency of the lowpass filter for the realtime
   interfaces.

For a complete and fully specified list check the API documentation for the
`Arm <https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html>`__
or the `Hand <https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html>`__.

All operations (non-realtime or realtime) on the Arm or the Hand are performed through the
``franka::Robot`` and ``franka::Gripper`` objects respectively. A connection to the Arm/Hand
will be established when the object is created:

.. code-block:: c++

    #include <franka/robot.h>
    #include <franka/gripper.h>

    ...

    franka::Gripper gripper("<fci-ip>");
    franka::Robot robot("<fci-ip>");

The address can be passed either as a hostname or an IP address. In case of any error, either due
to networking or conflicting library version, an exception of type ``franka::Exception`` will
be thrown. When using several robots at the same time, simply create several objects with
appropriate IP addresses.

To run a specific command, simply call the corresponding method, e.g.

.. code-block:: c++

    gripper.homing();
    robot.automaticErrorRecovery();


Realtime commands
-----------------

Realtime commands are UDP based and require a 1 kHz connection to Control.
There are two types of realtime interfaces:

 * **Motion generators**, which define a robot motion in joint or Cartesian space.
 * **Controllers**, which define the torques to be sent to the robot joints.

There are 4 different types of external motion generators and 3 different types of controllers
(one external and 2 internal) as depicted in the following figure:

.. figure:: _static/rt-interfaces.png
    :align: center
    :figclass: align-center

    Realtime interfaces: motion generators and controllers.

You can either use a single interface or combine two different types. Specifically, you can
command:

 * *only a motion generator* and therefore use one of the two internal controllers to follow
   the commanded motion.
 * *only an external controller* and ignore any motion generator signals, i.e. torque control only.
 * *a motion generator and an external controller* to use the inverse kinematics of Control in
   your external controller.

All realtime loops (motion generator or controller) are defined by a callback function that
receives the robot state and the duration of the last cycle (1 ms unless packet losses occur)
and returns the specific type of the interface. The ``control`` method of the ``franka::Robot``
class will then run the control loop by executing the callback function at a 1 kHz frequency,
as shown in this example

.. code-block:: c++

  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
     my_external_controller_callback;
  // Define my_external_controller_callback
  ...

  std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
      my_external_motion_generator_callback;
  // Define my_external_motion_generator_callback
  ...

  try {
    franka::Robot robot("<fci-ip>");
    // only a motion generator
    robot.control(my_external_motion_generator_callback);
    // only an external controller
    robot.control(my_external_controller_callback);
    // a motion generator and an external controller
    robot.control(my_external_motion_generator_callback, my_external_controller_callback);
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
    return 0;
  }

All control loops are finished once the ``motion_finished`` flag of a realtime command is set
to ``true``. An excerpt of the ``generate_joint_velocity_motion`` example included
in the `libfranka examples <https://frankaemika.github.io/libfranka/examples.html>`__ is shown here

.. code-block:: c++

   robot.control(
        [=, &time](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
          time += period.toSec();

          double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

          franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};

          if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });

In this case, the callback function is defined directly in the call of the
``robot.control( ... )`` function. It uses the joint velocity motion generator interface,
as it returns a ``franka::JointVelocities`` object. It commands joint velocities to the last four
joints and move them by approx. +/-12 degrees. After ``2 * time_max`` seconds it will return a
``motion_finished`` flag by setting it to true with the ``franka::MotionFinished`` method and
the control loop will stop.

Note that if you use only a motion generator, the default controller is the internal joint
impedance controller. You can however use the internal Cartesian impedance controller by
setting the optional argument of the control function, e.g.

.. code-block:: c++

    // Set joint impedance (optional)
    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
    // Runs my_external_motion_generator_callback with the default joint impedance controller
    robot.control(my_external_motion_generator_callback);
    // Identical to the previous line (default franka::ControllerMode::kJointImpedance)
    robot.control(my_external_motion_generator_callback, franka::ControllerMode::kJointImpedance);

    // Set Cartesian impedance (optional)
    robot.setCartesianImpedance({{2000, 2000, 2000, 100, 100, 100}});
    // Runs my_external_motion_generator_callback with the Cartesian impedance controller
    robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance);

For writing a controller, the ``franka::Robot::control`` function is used as well. The following
example shows a simple controller commanding zero torque for each joint. Gravity is
compensated by the robot.

.. code-block:: c++

    robot.control([&](const franka::RobotState&, franka::Duration) -> franka::Torques {
          return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        });

You can find examples for all interfaces and combinations of control loops in the
`libfranka examples <https://frankaemika.github.io/libfranka/examples.html>`__. Prior to running
the examples, verify that the robot has enough free space to move without colliding. Then, for
instance for the ``generate_joint_velocity_motion`` example execute the following command from
the ``libfranka`` build directory:

.. code-block:: shell

    ./examples/generate_joint_velocity_motion <fci-ip>


.. warning::

    For writing your own motion generators or controllers it is crucial to deliver a smooth
    signal to the robot. Nonsmooth signals can easily generate discontinuity errors or even
    make the robot unstable. Check the :ref:`interface specifications
    <control_parameters_specifications>` before starting.


Under the hood
********************
Until now we have covered details of the interface running on the client side, i.e your own
workstation PC. The behavior of the Control side of the realtime interface is shown in the
following figure

.. figure:: _static/rt-loop.png
    :align: center
    :figclass: align-center

    Realtime loop: from control commands to the robot desired joint torques.

**Motion generators**: all motion generator commands sent by the user have the subscript `c`
which stands for 'commanded'. When a motion generator is sent, the `Robot Kinematics completion`
block will compute the forward/inverse kinematics of the user-commanded signal yielding the
'desired' signals,  subscript `d`. If an internal controller is used, it will generate the
necessary torques :math:`\tau_{d}` to track the corresponding computed `d` signals (the internal
joint impedance controller will follow the joint signals :math:`q_{d}, \dot{q}_{d}` and the
internal Cartesian impedance controller the Cartesian ones
:math:`{}^OT_{EE,d}, {}^O\dot{P}_{EE,d}`) and send them to the robot joints.
All the variables in the figure, i.e. the last received `c` values, the computed `d` values
and their time derivatives are sent back to the user in the robot state. This way you can
take advantage of the inverse kinematics in your own external controller and, at the same time,
it will offer you `full transparency`: you will always know the exact values
and derivatives that the robot received and tracked in the last sample.

**External controller**: if an external controller is sent, the desired joint torques commanded
by the user :math:`\tau_{d}` are directly fed to the robot joints.

Note that, on the Control side, there are two things that could modify your signals:

* An optional `low pass filter`. You can set the cutoff frequency with the non-realtime command
  ``setFilters``. Set it to ``1000`` to deactivate it. Since version ``0.4.0`` it is
  deactivated by default.
* `Packet losses`, which may occur if you:

   * don't have a very good connection due to the performance of your PC + network card.  
   * your control loop is taking too long to compute (you have, depending on you network card and
     PC configuration, approx. < 300 :math:`\mu s` for your own control loop).

  In this case, Control assumes a constant acceleration model or a constant torque to extrapolate
  your signals. If ``>=20`` packets are lost in a row the control loop is stopped with the
  ``communication_constraints_violation`` exception.

.. hint::

    If you are not sure if your signals are being filtered or extrapolated, you can always check the
    last commanded values that you sent and compare them with the values you receive on the robot
    state in the next sample.

Rate limiters
*******************
As of version ``0.4.0``, libfranka includes rate limiters for all realtime interfaces running by
default. `Rate limiters`, also called `safe controllers`, will limit the rate of change of the
signals sent by the user to prevent the violation of the
:ref:`limits of the interface<control_parameters_specifications>`. For motion generators, it
will limit the acceleration and jerk, while, for an external controller, it will limit the
torque derivative. Rate limiters are part of libfranka so you can have a look at the code or even
change the limits to more conservative values for a less abrupt behavior.

You can deactivate the rate limiters of motion generators by specifying the second optional
parameter of the ``control`` function:

.. code-block:: c++

    // Set Cartesian impedance (optional)
    robot.setCartesianImpedance({{2000, 2000, 2000, 100, 100, 100}});
    // Turn off lowpass filter (off by default)
    robot.setFilters(1000, 1000, 1000, 1000, 1000);
    // Runs my_external_motion_generator_callback with the Cartesian impedance controller
    // and rate limiters on
    robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance);
    // Identical to the previous line (default true)
    robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance, true);
    // Runs my_external_motion_generator_callback with the Cartesian impedance controller
    // and rate limiters off
    robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance, false);

Or similarly for an external controller

.. code-block:: c++

    // Turn off lowpass filter
    robot.setFilters(1000, 1000, 1000, 1000, 1000);
    // With rate limiting
    robot.control(my_external_controller);
    // Without rate limiting
    robot.control(my_external_controller, false);

.. important::

    Rate limiters require the lowpass filter to be deactivated. You can easily do that by
    executing the non realtime command ``robot.setFilters(1000, 1000, 1000, 1000, 1000);`` before
    your control loop.

.. danger::

    Rate limiters are a robustness feature against packet losses to be used **after** you have
    already designed a smooth motion generator or controller. For the first tests of a new control
    loop we strongly recommend to deactivate this feature. Limiting the rate of a nonsmooth
    signal can easily yield instabilities or unexpected behavior. Too many packet losses can
    also generate unstable behavior.

Robot state
-----------------------
The robot state delivers the robot sensor readings and estimated values at a 1 kHz rate.
It provides:

 * *Joint level signals*: motor and extimated joint angles and their derivatives,
   joint torque and derivatives, estimated external torque, joint collision/contacts.
 * *Cartesian level signals*: Cartesian pose, configured endeffector and load parameters,
   external wrench acting on the endeffector, Cartesian collision
 * *Interface signals*: the last commanded and desired values and their derivatives,
   as explained in the previous subsection.

For a complete list check the API of the ``franka::RobotState``
`here <https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html>`__.
As shown in the the previous subsection, the robot state is always an input of all callback
functions for control loops. However, if you wish to only read the robot state without controlling
it, the functions ``read`` or ``readOnce`` can be used to gather it, e.g. for
logging or visualization purposes.

With a valid connection, *a single sample of the robot state* can be read using the ``readOnce``
function:

.. code-block:: c++

    franka::RobotState state = robot.readOnce();

The next example shows how to continuously read the robot state using the ``read`` function and a
callback. Returning ``false`` in the callback stops the loop. In the following, an excerpt of the
``echo_robot_state`` example is shown:

.. code-block:: c++

    size_t count = 0;
    robot.read([&count](const franka::RobotState& robot_state) {
      // Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
      // but should not be done in a control loop.
      std::cout << robot_state << std::endl;
      return count++ < 100;
    });


Model library
--------------------
The robot model library provides

  - The forward kinematics of all robot joints.
  - The body and zero jacobian matrices of all robot joints.
  - Dynamic parameters: inertia matrix, Coriolis and centrifugal vector and gravity vector.

Note that after you load the model library, you can compute kinematic and dynamic parameters for
an arbitrary robot state, not just the current one. You can also use the model library in a non
realtime fashion, e.g. in an optimzation loop. The libfranka examples include exemplary code
`printing joint poses
<https://frankaemika.github.io/libfranka/print_joint_poses_8cpp-example.html>`_
or `computing jacobians and dynamic parameters
<https://frankaemika.github.io/libfranka/cartesian_impedance_control_8cpp-example.html>`_.
