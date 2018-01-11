Troubleshooting
===============
This section lists solutions to a set of possible errors which can happen when using the FCI.

 .. hint::

    Further help is provided in the troubleshooting page of the manual shipped with your robot.


Motion stopped due to discontinuities
-------------------------------------

If the difference between commanded values in subsequent time steps is too large, then the motion
is stopped with a discontinuity error such as ``joint_motion_generator_velocity_discontinuity``.

Discontinuities can occur if your code commands actual jumps to the robot, but also because of
network packet losses. If the issue occurs even when using the provided examples, the problem is
most likely related to overall communication quality. To ensure best performance, please check the
following:

 * All source code is compiled with optimizations (``-DCMAKE_BUILD_TYPE=Release``). If you installed
   ``libfranka`` and ``franka_ros`` from the ROS repositories, this is already the case for these
   projects. However, your own source code needs to be compiled with optimizations as well.
 * Verify your network connection by executing the `network bandwidth, delay and jitter test`_.
 * ``franka::Robot`` is instantiated with ``RealtimeConfig::kEnforce``. This is the default if no
   ``RealtimeConfig`` is explicitly specified in the constructor.
 * Power saving features are disabled (cpu frequency scaling, power saving mode,
   laptop on battery, BIOS power saving features, etc.)
 * Try to lower the cut-off frequency for commands by running ``franka::Robot::setFilters`` with
   e.g. 50 Hz. This will mitigate network packet losses, but also decrease the accuracy with which
   the robot follows the commanded trajectory.

.. _troubleshooting_robot_not_reachable:

Robot is not reachable
----------------------

Try to ping the robot using the following command:

.. code-block:: shell

    ping <fci-ip>

If this command fails, the robot is not properly connected to the network, or the IP address
is not correctly assigned during the setup phase. Please set up the network according to the
documents sent with your robot.


.. _network-bandwidth-delay-test:

Network bandwidth, delay and jitter test
----------------------------------------

The following command will simulate a network load which is equivalent to a scenario where the
robot is controlled by the FCI:

.. code-block:: shell

    sudo ping <fci-ip> -i 0.001 -D -c 10000 -s 1200

Example output:

.. code-block:: shell

    PING <fci-ip> 1200(1228) bytes of data.
    [1500982522.977579] 1208 bytes from <fci-ip>: icmp_seq=1 ttl=64 time=0.279 ms
    [1500982522.978423] 1208 bytes from <fci-ip>: icmp_seq=2 ttl=64 time=0.224 ms
    [1500982522.979434] 1208 bytes from <fci-ip>: icmp_seq=3 ttl=64 time=0.196 ms
    [1500982522.980482] 1208 bytes from <fci-ip>: icmp_seq=4 ttl=64 time=0.243 ms
    ....
    [1500982533.034267] 1208 bytes from <fci-ip>: icmp_seq=9999 ttl=64 time=0.236 ms
    [1500982533.035211] 1208 bytes from <fci-ip>: icmp_seq=10000 ttl=64 time=0.203 ms

    --- <fci-ip> ping statistics ---
    10000 packets transmitted, 10000 received, 0% packet loss, time 10057ms
    rtt min/avg/max/mdev = 0.147/0.240/0.502/0.038 ms


The example result shows an average round-trip time of 0.24 ms and a maximum round-trip time of 0.5
ms. The standard deviation `mdev` is around 0.04 ms. As explained in the
:ref:`network requirements section<requirement-network>` it must be guaranteed that the sum of the
round-trip time and the execution time of the motion generator or control loop is
**less than 1 ms**. If this constraint is violated for a cycle, the received packet is dropped by
the FCI.

If the round-trip time is long even with a direct connection, consider
purchasing a separate, high performance PCI-Express network card for your
workstation PC. See if there are dedicated drivers for your network card,
these usually offer better performance. Lastly, the CPU can also be a limiting
factor for network performance.
