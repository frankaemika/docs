Troubleshooting
===============


Jumps in robot movement
-----------------------

Please check the following things:

* RT kernel is booted
* RT permissions for the process is set
* Check `Network bandwidth, delay and jitter test`_



.. _troubleshooting_robot_not_reachable:

Robot is not reachable
----------------------

Try to ping the robot using the following command:

	ping <franka-control-ip>

If this command fails, the robot is not properly connected to the network, or the IP was not
correctly assigned during the setup phase. Please set up the network according to documents, sent
with the robot.


.. _troubleshooting_open_brake:

Open the brakes of the robot
----------------------------

The brakes can be opened from the from FRANKA DESK at https://<franka-control-ip>. Further
information are provided by the documents, shipped with the robot.


.. _network-bandwidth-delay-test:

Network bandwidth, delay and jitter test
----------------------------------------

The following command will simulate an equivalent network load::

	sudo ping <franka-control-ip> -i 0.001 -D -c 10000 -s 1200

Example output::

	PING <franka-control-ip> (XXX.XXX.XXX.XXX) 1200(1228) bytes of data.
	[1500982522.977579] 1208 bytes from <franka-control-ip> (XXX.XXX.XXX.XXX): icmp_seq=1 ttl=64 time=0.279 ms
	[1500982522.978423] 1208 bytes from <franka-control-ip> (XXX.XXX.XXX.XXX): icmp_seq=2 ttl=64 time=0.224 ms
	[1500982522.979434] 1208 bytes from <franka-control-ip> (XXX.XXX.XXX.XXX): icmp_seq=3 ttl=64 time=0.196 ms
	[1500982522.980482] 1208 bytes from <franka-control-ip> (XXX.XXX.XXX.XXX): icmp_seq=4 ttl=64 time=0.243 ms
	....
	[1500982533.034267] 1208 bytes from <franka-control-ip> (XXX.XXX.XXX.XXX): icmp_seq=9999 ttl=64 time=0.236 ms
	[1500982533.035211] 1208 bytes from <franka-control-ip> (XXX.XXX.XXX.XXX): icmp_seq=10000 ttl=64 time=0.203 ms

	--- <franka-control-ip> ping statistics ---
	10000 packets transmitted, 10000 received, 0% packet loss, time 10057ms
	rtt min/avg/max/mdev = 0.147/0.240/0.304/0.041 ms
