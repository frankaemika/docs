Minimum system and network requirements
=======================================

This page only specifies the requirements for running the research interface. Additional
requirements are specified in the documents that you received with the robot.

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
If possible, directly connect your workstation / control PC to the **LAN port of FRANKA CONTROL**.
Having relays in between could lead to delay, jitter or packet loss. This will decrease the
performance of your controller or make it unusable.

.. hint::
    To minimize the delay, use the LAN port of FRANKA CONTROL, not the LAN port in the base of the
    FRANKA ARM.

To control the robot, it must be guaranteed that the sum of the following time
measurements is less than 1 ms:

 * Round-trip time between the workstation PC and FRANKA CONTROL.
 * Execution time of the motion generator or control loop.

.. caution::
    If the **<1 ms RRT constraint** is violated for a cycle, the received packet is dropped by the
    research interface. After 20 dropped packets, the robot stops.

If a **motion generator command packet is dropped**, the robot takes the previous waypoints and
performs a linear extrapolation (keep acceleration constant and integrate) for the missed
timestep. If more than 20 packets are lost or dropped in a row, the robot stops.

If a **controller command packet is dropped**, the research interface will reuse the torques of
the last successful received packet. Again, more than 20 in a row lost or dropped packets will
stop the robot.

.. hint::
    Measure the performance of your network (see :ref:`network-bandwidth-delay-test`) and the
    control or motion generator loop beforehand.
