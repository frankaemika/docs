Getting started
===============

After :doc:`setting up the required software <installation>`, you can get started with your FRANKA!

Setting up the robot
--------------------

Before we get started with controlling FRANKA, here are a few important points to consider when
working with FRANKA.

1. Make sure that FRANKA has been mounted on a stable base and cannot topple over, even when
   performing fast motions or abrupt stops.

.. caution::

    FRANKA's built-in controllers only support tabletop mounting, i.e. assumes that FRANKA is
    mounted perpendicular to the ground! Other mountings will **void your warranty**, decrease the
    performance of the robot and **might damage the robot**!


2. Ensure that the cable connecting FRANKA and FRANKA CONTROL is firmly attached on both sides.
3. Connect a user stop to FRANKA's base to be able to stop movement of the robot at any time.

.. hint::

    Pressing the user stop will switch the joint motor controllers to hold their position.
    **The user stop is not an emergency stop!**

Plug in a network cable to the Ethernet connection at the base of FRANKA ARM. You can now point a
web browser to https://robot.franka.de to connect to the FRANKA DESK interface.


Verifying the connection
------------------------

You can run the ``echo_robot_state`` example from ``libfranka`` to see if you can successfully
connect to your FRANKA using the research interface. Ensure your network cable is plugged in to the
Ethernet connection at the base of FRANKA (*not* to the master controller!).

Change to the build directory of ``libfranka`` and execute the example:

.. code-block:: shell

    ./examples/echo_robot_state robot.franka.de

The program will print the current state of the robot to the console and terminate after a few
iterations.

.. hint::

    If you get an error at this point, ensure that FRANKA's brakes are opened. You can open the
    brakes from FRANKA DESK at https://robot.franka.de.


Getting started with ``libfranka``
----------------------------------

Connecting libfranka to the robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All operations on the robot are performed through the ``franka::Robot`` object. A connection to the
robot will be established when the object is created:

.. code-block:: c++

    #include <franka/robot.h>

    ...

    franka::Robot robot("robot.franka.de");

The address can be passed either as a hostname or an IP address. In case of any error, either due to
networking, or conflicting library version, an exception of the ``franka::Exception`` type will be
thrown. When using several FRANKAs at the same time, simply create several objects with appropriate
addresses.


Reading the robot state
^^^^^^^^^^^^^^^^^^^^^^^

Use the ``read`` or ``readOnce`` functions if you want to read the robot state, e.g. for logging or
visualization, but do not need to execute any motions. With a valid connection, a single sample of
the robot state can be read using the ``readOnce`` function:

.. code-block:: c++

    franka::RobotState state = robot.readOnce();


The state can be read continuously using the ``read`` function and a callback. Return ``false`` in
the callback when you want to stop, like in the ``echo_robot_state`` example:

.. code-block:: c++

    size_t count = 0;
    robot.read([&count](const franka::RobotState& robot_state) {
        std::cout << robot_state << std::endl;
        return count++ < 100;
    });


Moving the robot
^^^^^^^^^^^^^^^^

To try moving the robot, execute the ``generate_joint_velocity_motion`` example. Brakes and the user
stop must be released before moving. The example will move the last four joints. After verifying
that the robot has enough free space to move without colliding, execute the following from the build
directory:

.. code-block:: shell

    ./examples/generate_joint_velocity_motion robot.franka.de

The robot is moved by a `controller` which specifies the desired torque on each joint. You can
choose from one of the four `controllers`, or provide your own. Additionally, the `controllers` can
be fed with desired joint values :math:`q_d` by `motion generators`. Your motion generator can use
one of the four interfaces:

* Joint position
* Joint velocity
* Cartesian position
* Cartesian velocity

You execute the motions by providing a callback to the ``franka::Robot::control`` function.
An example usage can be seen in ``examples/generate_joint_velocity_motion.cpp``:

.. code-block:: c++

    double time_max = 4.0;
    double omega_max = 0.2;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&) -> franka::JointVelocities {
      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      time += 0.001;
      if (time > 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::Stop;
      }
      return {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
    });

The callback provided to the ``robot.control`` will be executed for each robot state received from
FRANKA, at 1 KHz frequency. In the above example. the desired velocity is returned
``{{0.0, 0.0, 0.0, omega, omega, omega, omega}}`` during motion. When the motion is finished
``franka::Stop`` is returned instead. This example uses the default `Joint Impedance` controller,
which offers the best performance (and can be used for Cartesian motions as well).

.. caution::

    Do not call any ``franka::Robot`` functions inside of the callbacks, as this would negatively
    influence the timings.

To provide your own controller, you would also use ``franka::Robot::control`` function. Here's a
simple controller commanding zero torque for each joint:

.. code-block:: c++

    robot.control([&](const franka::RobotState& robot_state) -> franka::Torques {
          return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        });

``motion_with_control.cpp`` shows how to provide both external motion generation and control.

When creating motions, make sure they have smooth velocity and acceleration profiles. Big
discontinuities can trigger robot's safety features and abort the motion.


Getting started with ROS
------------------------

.. todo::
 Add description of ROS packages, example launchfiles, ...
