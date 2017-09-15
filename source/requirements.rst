Minimum system and network requirements
=======================================

This page only specifies the requirements for running the Franka Control Interface (FCI)
Additional requirements are specified in the documents that you have received with your robot.

Workstation PC
--------------

+------------------------------------------------------------+
| Minimum System Requirements                                |
+===================+========================================+
| Operating System  | Linux with PREEMPT_RT patched kernel   |
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
    The best performance of your robot can be achieved when you directly connect to the LAN port
    of Control.

To control the robot, it must be guaranteed that the sum of the following time measurements is
less than 1 ms:

 * Round trip time (RTT) between the workstation PC and FCI.
 * Execution time of your motion generator or control loop.

.. caution::
    If the **<1 ms RTT constraint** is violated for a cycle, the received packet is dropped by
    FCI. After 20 dropped packets, your robot `will stop`.

If a **motion generator command packet is dropped**, the robot takes the previous waypoints and
performs a linear extrapolation (keep acceleration constant and integrate) for the missed
timestep. If more than 20 packets are lost or dropped in a row, your robot `will stop`.

If a **controller command packet is dropped**, FCI will reuse the torques of the last successful
received packet. Again, more than 20 consecutive lost or dropped packets will cause your robot to
`stop`.

.. hint::
    Measure the performance of your network (see :ref:`network-bandwidth-delay-test`) and the
    control or motion generator loop beforehand.
