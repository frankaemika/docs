Control parameters specifications
=================================

Control parameters fed into the robot should fulfill *recommended* and *necessary* conditions.
Recommended conditions should be fulfilled to ensure optimal operation of the robot. If necessary
conditions are not met then the motion will be aborted.

The final robot trajectory is the result of processing the user-specified trajectory ensuring that
recommended conditions are fulfilled. As long as necessary conditions conditions are met, the robot
will try to follow the user-provided trajectory but it will only match the final trajectory if it
also fulfills recommended conditions.
If the necessary conditions are violated, an error will abort the motion: if, for instance, the
first point of the user defined joint trajectory is very different from :math:`q_{start}` a velocity
limits violation error will abort the motion.

.. _limit_table:

Limits table
------------

The constants used throughout this chapter have the following values:

===== ================== ================== ====================== ====================== ===================== ===================== =============== ===============
Joint :math:`\tau_{min}` :math:`\tau_{max}` :math:`\ddot{q}_{min}` :math:`\ddot{q}_{max}` :math:`\dot{q}_{min}` :math:`\dot{q}_{max}` :math:`q_{min}` :math:`q_{max}`
===== ================== ================== ====================== ====================== ===================== ===================== =============== ===============
1     -82.65             82.65              -14.25                 14.25                  -2.375                2.375                 -2.818745       -2.818745
2     -82.65             82.65              -7.125                 7.125                  -2.375                2.375                 -1.74097        -1.74097
3     -82.65             82.65              -11.875                11.875                 -2.375                2.375                 -2.818745       -2.818745
4     -82.65             82.65              -11.875                11.875                 -2.375                2.375                 -2.98452        -2.98452
5     -11.4              11.4               -14.25                 14.25                  -2.85                 2.85                  -2.818745       -2.818745
6     -11.4              11.4               -19.0                  19.0                   -2.85                 2.85                  -0.082935       -0.082935
7     -11.4              11.4               -19.0                  19.0                   -2.85                 2.85                  -2.818745       -2.818745
===== ================== ================== ====================== ====================== ===================== ===================== =============== ===============

Joint trajectory requirements
-----------------------------

Recommended conditions
**********************

1. :math:`q_{min} < q < q_{max}`
2. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
3. :math:`\tau_{j, min} < \tau_j < \tau_{j, max}`
4. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`

beginning of trajectory:

5. :math:`q = q_{start}` (The most recent q_d)
6. :math:`\dot{q} = 0`
7. :math:`\ddot{q} = 0`

end of trajectory:

8. :math:`\dot{q} = 0`
9. :math:`\ddot{q} = 0`

Necessary conditions
*********************

1. :math:`q_{min} < q < q_{max}`
2. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
3. :math:`\ddot{q}_{min} < \ddot{q} < \ddot{q}_{max}`
4. :math:`\dddot{q}_{min} < \dot{q} < \dddot{q}_{max}`

Cartesian trajectory requirements
---------------------------------

Recommended conditions
**********************

1. :math:`q_{min} < q < q_{max}`
2. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
3. :math:`\tau_{j, min} < \tau_j < \tau_{j, max}`
4. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{tau}_{j, max}`

beginning of trajectory:

5. :math:`{}^OT_{EE} = {}^OT_{EE, start}` (The most recent O_T_EE_d)
6. :math:`\dot{p} = 0`
7. :math:`\ddot{p} = 0`

end of trajectory:

8. :math:`\dot{p} = 0`
9. :math:`\ddot{p} = 0`

Necessary conditions
********************

1. :math:`T` is proper transformation matrix
2. :math:`\dot{p}_{min} < \dot{p} < \dot{p}_{max}`
3. :math:`\ddot{p}_{min} < \ddot{p} < \ddot{p}_{max}`

derived from inverse kinematics:

4. :math:`q_{min} < q < q_{max}`
5. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
6. :math:`\ddot{q}_{min} < \ddot{q} < \ddot{q}_{max}`

Controller requirements
-----------------------

Recommended conditions
**********************

1. :math:`\tau_{j, min} < \tau_j < \tau_{j, max}`
2. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`

beginning of trajectory:

3. :math:`\tau_j = 0`

Necessary conditions
********************

1. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`
