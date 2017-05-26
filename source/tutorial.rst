Tutorial
========

After :doc:`building the library <build-instructions>`, you can get started with your FRANKA!

Setup
-----

Before we get started with controlling FRANKA, here are a few important points to consider when working with FRANKA.

1. Make sure that FRANKA has been mounted on a stable base and cannot topple over, even when performing fast motions or abrupt stops.

.. caution::

    FRANKA's built-in controllers only support tabletop mounting, i.e. assumes that FRANKA is mounted perpendicular to the ground! You can implement your own controller to work around this limitation, but this might **damage the robot** and void your warranty!


2. Ensure that the cable connecting FRANKA and FRANKA CONTROL is firmly attached on both sides.
3. Connect a user stop to FRANKA's base to be able to stop movement of the robot at any time.

.. hint::

    Pressing the user stop will switch the joint motor controllers to hold their position. **The user stop is not an emergency stop!**

Plug in a network cable to the ethernet connection at the base of FRANKA ARM. You can now point a web browser to https://robot.franka.de to connect to the FRANKA DESK interface.

Getting started with the research interface
-------------------------------------------

You can run the ``echo_robot_state`` example to see if you can successfully connect to your FRANKA. Plug in a network cable to the ethernet connection at the base of FRANKA (*not* to the master controller!).

Change to the build directory in your ``libfranka`` source directory and execute the example:

.. code-block:: shell

    ./examples/echo_robot_state robot.franka.de

The program will print the current state of the robot to the console and terminate after a few iterations.

.. hint::

    If you get an error at this point, ensure that FRANKA's brakes are opened. You can open the brakes from FRANKA DESK at https://robot.franka.de.
