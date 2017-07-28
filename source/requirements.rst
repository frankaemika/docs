Minimum system and network requirements
=======================================

This page only specifies the requirements for running the Research Interface. Additional
requirements are specified in the documents that you received with your robot.

Workstation PC
--------------

+------------------------------------------------------------+
| Minimum System Requirements                                |
+===================+========================================+
| Operating System  | Linux with  PREEMPT_RT patched kernel  |
+-------------------+----------------------------------------+
| Network card      | 100BASE-TX                             |
+-------------------+----------------------------------------+


Network
-------
If possible, directly connect your workstation / control PC to the **LAN port of FRANKA CONTROL**.
Having relays in between could lead to delay, jitter or packet loss. This will decrease the
performance of your controller or make it unusable.

.. hint::
	To minimize the delay, use the LAN port of FRANKA CONTROL, not the LAN port in the base of the
	FRANKA ARM.

In order to get a good performance of the robot, it must be guaranteed, that the sum of the
following time measurements is less than 1 ms.

The time,
 * it takes to send the current measurements from FRANKA CONTROL to the workstation PC.
 * the motion generator or control loop takes.
 * it takes to send the command back to FRANKA CONTROL.

.. caution::
	If the **<1 ms RRT constraint** is violated for a cycle, the received packet is dropped by the
	Research Interface. After 50 dropped packets, the robot stops.

If a **motion generator command packet is dropped**, the robot takes the previous waypoints and
does a linear extrapolation (keep acceleration constant and integrate) for the missed timestep. If
more than 50 packets are lost or dropped in a row, the robot stops.

If a **controller command packet is dropped**, the research interface will reuse the torques of
the last successful received packet. Again, more than 50 in a row lost or dropped packets will
stop the robot.

.. hint::
	Measure the performance of your network (see :ref:`network-bandwidth-delay-test`) and the
	algorithm beforehand.
