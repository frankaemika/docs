Getting started
===============

After :doc:`setting up the required software <installation>`, your robot needs to be installed and
tested. **Please read through the documents shipped with your robot and follow the setup
instructions before continuing any further!**


Powering on the robot
---------------------

Always check the following things before powering on the robot.

1. Make sure that the Arm has been mounted on a stable base and cannot topple over, even
   when performing fast motions or abrupt stops.

.. caution::
   Only tabletop mounting is supported, i.e. the Arm must be mounted perpendicular to the
   ground! Other mountings will **void your warranty** and **might cause damage
   to the robot**!

2. Ensure that the cable connecting Arm and Control is firmly attached on both sides.
3. Connect the external activation device to Arm's base and keep it next to you in order to be
   able to stop the robot at any time.

.. hint::
   Activating the external activation device will disconnect the Arm from Control.
   The joint motor controllers will then hold their current position.
   The *external activation device is not an emergency stop!*

The manual shipped with your robot describes how to specify the IP address of the Control's
LAN port during the setup process. In the following sections that address is referred to
as <fci-ip>.

The workstation PC, which commands your robot using the FCI,
must always be connected to the LAN port of Control and `not` to the LAN port of Arm.


Verifying the connection
------------------------

In order to verify that everything is correctly set up, run the ``echo_robot_state``
example from ``libfranka``.

Change to the build directory of ``libfranka`` and execute the example:

.. code-block:: shell

    ./examples/echo_robot_state <fci-ip>

The program will print the current state of the robot to the console and terminate after a few
iterations.

Example output:

.. code-block:: json

    {
      "O_T_EE": [0.99507,-0.087825,0.0458691,0,-0.0932286,-0.98667,0.133307,0,0.0335506,-0.136928,
                 -0.990013,0,0.466187,-0.0437226,0.310521,1],
      "O_T_EE_d": [0.7071,0.7071,0,0,0.7071,-0.7071,0,0,0,0,-1,0,0.088,0,0.769,1],
      "F_T_EE": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], "EE_T_K": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
      "m_load": 0, "F_x_Cload": [0,0,0], "I_load": [0,0,0,0,0,0,0,0,0],
      "elbow": [-0.0658817,-1], "elbow_d": [0,1],
      "tau_J": [0.159435,-16.7615,0.0478516,18.8524,0.751037,1.43036,0.0289359],
      "dtau_J": [0,0,0,0,0,0,0],
      "q": [0.0412189,-0.294853,-0.0658817,-2.30709,-0.175077,2.04355,0.931813],
      "dq": [-0.000408694,-0.000135271,8.87298e-05,0.000983035,-0.00110876,-0.00250781,0.00128548],
      "q_d": [0,0,0,0,0,0,0], "dq_d": [0,0,0,0,0,0,0],
      "joint_contact": [0,0,0,0,0,0,0], "cartesian_contact": [0,0,0,0,0,0],
      "joint_collision": [0,0,0,0,0,0,0], "cartesian_collision": [0,0,0,0,0,0],
      "tau_ext_hat_filtered": [0.172327,-0.230889,-0.130005,-0.189423,0.067974,-0.237919,0.0259882],
      "O_F_ext_hat_K": [-1.28182,0.136979,0.436185,0.0400712,-0.651955,0.0273414],
      "K_F_ext_hat_K": [-1.26755,0.0424965,-0.493591,0.106515,0.0430354,-0.00899586],
      "current_errors": [], "last_motion_errors": [], "robot_mode": "Idle", "time": 12444112
    }


.. hint::

    If an error occurs at this point, perform the
    :ref:`ping test <troubleshooting_robot_not_reachable>` and ensure that the robot's fail-safe
    safety locking system is opened. Further information are provided in the manual shipped with
    the robot.
