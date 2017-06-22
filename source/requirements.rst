Minimum system and network requirements
=============================================

This only specifies the requirements for running the Research Interface. Additional requirements are
specified in the documents that you received with your robot.

Workstation PC
-----------------------------

+--------------------------------------------------------------------+
| Minimum System Requirements                                        |
+===================+================================================+
| Operating System  | Linux with  PREEMPT_RT patched kernel          |
+-------------------+------------------------------------------------+
| Network card      | 100BASE-TX                                     |
+-------------------+------------------------------------------------+


Network
-----------------------------

If possible, directly connect your workstation / control PC to the robot. Having relays in between
could lead to delay, jitter or packet loss. This will decrease the performance of your controller or
even make it unusable.
