Minimum system and network requirements
=======================================

This page only specifies the requirements for running the FRANKA Control Interface
Additional requirements are specified in the documents that you have received with your Panda.

Workstation PC
--------------

+------------------------------------------------------------+
| Minimum System Requirements                                |
+===================+========================================+
| Operating System  | Linux with  PREEMPT_RT patched kernel  |
+-------------------+----------------------------------------+
| Network card      | 100BASE-TX                             |
+-------------------+----------------------------------------+

.. _requirement-network:

Network
-------
If possible, directly connect your workstation PC to the LAN port of Control.
Having relays in between could lead to delay, jitter or packet loss. This will decrease the
performance of your controller or make it unusable.

.. hint::
    To minimize the delay, use the LAN port of Control, and not the LAN port in the base of the
    Arm.

To control the robot, it must be guaranteed that the sum of the following time
measurements is less than 1 ms:

 * Round-trip time between the workstation PC and Control.
 * Execution time of the motion generator or control loop.

.. caution::
    If the **<1 ms RRT constraint** is violated for a cycle, the received packet is dropped by the
    FRANKA Control Interface. After 20 dropped packets, the robot stops.

If a **motion generator command packet is dropped**, the robot takes the previous waypoints and
performs a linear extrapolation (keep acceleration constant and integrate) for the missed
timestep. If more than 20 packets are lost or dropped in a row, the robot `will stops`.

If a **controller command packet is dropped**, the FRANKA Control interface will reuse the
torques of the last successful received packet. Again, more than 20 consecutive lost or dropped
packets will cause your robot to `stop`.

.. hint::
    Measure the performance of your network (see :ref:`network-bandwidth-delay-test`) and the
    control or motion generator loop beforehand.
