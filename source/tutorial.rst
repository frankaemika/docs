Getting started
===============

.. todo::

  Add information about the emergency stop


After :doc:`setting up the required software <installation>`, the robot needs to be installed and
tested. **Please read - in beforehand - through the documents shipped with the robot and follow
the set-up instructions!**


Turning on the robot
--------------------

Always check the following things before turning on the robot.

1. Make sure that the FRANKA ARM has been mounted on a stable base and cannot topple over, even
   when performing fast motions or abrupt stops.

.. caution::
   FRANKA's built-in controllers only support tabletop mounting, i.e. assumes that FRANKA is
   mounted perpendicular to the ground! Other mountings will **void your warranty**, decrease the
   performance of the robot and **might damage the robot**!

2. Ensure that the cable connecting FRANKA ARM and FRANKA CONTROL is firmly attached on both sides.
3. Connect a user stop to FRANKA's base and keep it next to you in order to be able to stop
   the robot at any time.

.. hint::
   Pressing the user stop will switch from FRANKA CONTROL to the joint motor controllers. They
   will hold their current position. **The user stop is not an emergency stop!**


In the setup process, described in the manual which is shipped with Franka, the IP of the FRANKA
CONTROL LAN port is set. In the following, the **FRANAK CONTROL IP is referred as
<franka-control-ip>.** The workstation PC, which commands the robot using the research interface,
must always be connected to the LAN port of FRANKA CONTROL, not to the LAN port at the robot base.


Verifying the connection
------------------------

In order to verify, that everything is correctly set up and it is possible to use the research
interface, run the ``echo_robot_state`` example from ``libfranka``.

Change to the build directory of ``libfranka`` and execute the example:

.. code-block:: shell

    ./examples/echo_robot_state <franka-control-ip>

The program will print the current state of the robot to the console and terminate after a few
iterations.

Example output:

.. code-block:: json

    {
      "O_T_EE": [0.99507,-0.087825,0.0458691,0,-0.0932286,-0.98667,0.133307,0,0.0335506,-0.136928,
                 -0.990013,0,0.466187,-0.0437226,0.310521,1],
      "elbow": [-0.0658817,-1],
      "O_T_EE_d": [0.7071,0.7071,0,0,0.7071,-0.7071,0,0,0,0,-1,0,0.088,0,0.769,1],
      "elbow_d": [0,1], "EE_T_K": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
      "tau_J": [0.159435,-16.7615,0.0478516,18.8524,0.751037,1.43036,0.0289359],
      "dtau_J": [0,0,0,0,0,0,0],
      "q": [0.0412189,-0.294853,-0.0658817,-2.30709,-0.175077,2.04355,0.931813],
      "dq": [-0.000408694,-0.000135271,8.87298e-05,0.000983035,-0.00110876,-0.00250781,0.00128548],
      "q_d": [0,0,0,0,0,0,0],
      "joint_contact": [0,0,0,0,0,0,0], "cartesian_contact": [0,0,0,0,0,0],
      "joint_collision": [0,0,0,0,0,0,0], "cartesian_collision": [0,0,0,0,0,0],
      "tau_ext_hat_filtered": [0.172327,-0.230889,-0.130005,-0.189423,0.067974,-0.237919,0.0259882],
      "O_F_ext_hat_K": [-1.28182,0.136979,0.436185,0.0400712,-0.651955,0.0273414],
      "K_F_ext_hat_K": [-1.26755,0.0424965,-0.493591,0.106515,0.0430354,-0.00899586],
      "current_errors": {}, "last_motion_errors": {}, "time": 12444112
    }


.. hint::

    If an error occurs at this point, do the :ref:`ping test <troubleshooting_robot_not_reachable>`
    and ensure that FRANKA's brakes are opened. The brakes can be opened the from FRANKA DESK at
    https://<franka-control-ip>.


Getting started with libfranka
------------------------------

Connecting libfranka to the robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All operations on the robot are performed through the ``franka::Robot`` object. A connection to the
robot will be established when the object is created:

.. code-block:: c++

    #include <franka/robot.h>

    ...

    franka::Robot robot("<franka-control-ip>");

The address can be passed either as a hostname or an IP address. In case of any error, either due
to networking or conflicting library version, an exception of the ``franka::Exception`` type will
be thrown. When using several FRANKAs at the same time, simply create several objects with
appropriate IP addresses.


Reading the robot state
^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^

The robot can be moved, by executing one of many examples provided with ``libfranka``, like the
``generate_joint_velocity_motion`` example. As already mentioned before, the
:ref:`brakes <troubleshooting_open_brakes>` and the user stop must be released before moving,
otherwise an error is printed. This example will move the
last four joints.

After verifying, that the robot has enough free space to move without colliding, execute the
following command from the build directory:

.. code-block:: shell

    ./examples/generate_joint_velocity_motion <franka-control-ip>

The robot is moved by a `controller` which specifies the desired torque on each joint. It is
possible to choose between four built in `controllers`. Further, a self written controller
can be provided. Additionally, the `controllers` can be fed with desired joint values :math:`q_d`
by `motion generators`. For building a motion generator, one of the four interfaces can be used:

* Joint position
* Joint velocity
* Cartesian position
* Cartesian velocity


The motions are executed by providing a callback to the ``franka::Robot::control`` function.
An excerpt from ``examples/generate_joint_velocity_motion.cpp`` of an example usage is shown in
the following:

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
FRANKA, at 1 kHz frequency. In the callback, read() and readOnce() is not needed, as the robot
state is provided. In the above example. the desired velocity is returned
``{{0.0, 0.0, 0.0, omega, omega, omega, omega}}`` during motion. When the motion is finished
``franka::Stop`` is returned instead. This example uses the default `Joint Impedance` controller,
which offers the best performance (and can be used for Cartesian motions as well).

.. caution::

    Do not call any ``franka::Robot`` functions inside of the callbacks, as this would negatively
    influence the timings.

For writing a controller, the ``franka::Robot::control`` function is used as well. The following
example shows a **simple controller** commanding zero torque for each joint. The gravity is
compensated by the robot.

.. code-block:: c++

    robot.control([&](const franka::RobotState&, franka::Duration) -> franka::Torques {
          return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        });


The combination of both, **external motion generation and control** is shown in the example file
``motion_with_control.cpp``.

When creating motions, make sure they have smooth velocity and acceleration profiles. Big
discontinuities can trigger robot's safety features and abort the motion.



Getting started with ROS
------------------------

.. todo::
 Add description of ROS packages, example launchfiles, ...
