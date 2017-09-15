libfranka
=========

Connecting libfranka to the robot
---------------------------------

All operations on the robot are performed through the ``franka::Robot`` object. A connection to the
robot will be established when the object is created:

.. code-block:: c++

    #include <franka/robot.h>

    ...

    franka::Robot robot("<control-ip>");

The address can be passed either as a hostname or an IP address. In case of any error, either due
to networking or conflicting library version, an exception of type ``franka::Exception`` will
be thrown. When using several robots at the same time, simply create several objects with
appropriate IP addresses.


Reading the robot state
-----------------------

The functions ``read`` or ``readOnce`` can be used to gather the current robot state, e.g. for
logging or visualization.


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


Moving the robot
----------------

 .. warning::

    Please read the manual before you start using the robot.

The robot can be moved by executing one of the many examples provided with ``libfranka``, such as
the ``generate_joint_velocity_motion`` example.
This example will move the last four joints by +/-12 degrees. Prior to running the example,
verify that the robot has enough free space to move without colliding. Then, execute the following
command from the ``libfranka`` build directory:

.. code-block:: shell

    ./examples/generate_joint_velocity_motion <control-ip>

The robot is moved by a `controller` which specifies the desired joint level torque. It
is possible to use a built in `controller`. Alternatively, a self written controller can be
provided. Additionally, the `controller` can be fed with desired values by `motion generators`.


Currently the following internal controllers are available:

* Joint impedance


For building a motion generator, one of the following four interfaces can be used:

* Joint position
* Joint velocity
* Cartesian position
* Cartesian velocity


The motions are executed by providing a callback to the ``franka::Robot::control`` function.
An excerpt from ``examples/generate_joint_velocity_motion.cpp`` is shown in the following:

.. code-block:: c++

    double time_max = 4.0;
    double omega_max = 0.2;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&,
                             franka::Duration time_step) -> franka::JointVelocities {
      time += time_step.s();

      if (time > 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::Stop;
      }

      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      return {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
    });


The callback provided to the ``robot.control`` will be executed for each robot state received from
the robot by the control interface, at 1 kHz frequency. In the callback, read() and readOnce() are
not needed as the robot state is provided as an input argument to the callback. In the above
example, the desired velocity is returned as ``{{0.0, 0.0, 0.0, omega, omega, omega, omega}}``
during motion. When the motion is finished ``franka::Stop`` is returned instead. This example uses
robot's internal joint impedance controller.

.. caution::

    Do not call any ``franka::Robot`` functions inside of the callbacks, as this would negatively
    influence the timings.

For writing a controller, the ``franka::Robot::control`` function is used as well. The following
example shows a simple controller commanding zero torque for each joint. The gravity is
compensated by the robot.

.. code-block:: c++

    robot.control([&](const franka::RobotState&, franka::Duration) -> franka::Torques {
          return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        });


The combination of both, external motion generation and control is shown in the example file
``motion_with_control.cpp``.

When creating motions, make sure they have smooth velocity and acceleration profiles. Big
discontinuities can trigger robot's safety features and abort the motion.
