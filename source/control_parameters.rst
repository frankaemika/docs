Control parameters specifications
=================================

Control parameters fed into the robot should fulfill *necessary* and *recommended* conditions. If
necessary conditions are not met then the motion will be aborted. Recommended conditions should be
fulfilled to ensure optimal operation of the robot.

The final robot trajectory is the result of processing the user-specified trajectory ensuring that
recommended conditions are fulfilled. As long as necessary conditions are met, the robot
will try to follow the user-provided trajectory but it will only match the desired trajectory if it
also fulfills recommended conditions.
If the necessary conditions are violated, an error will abort the motion: if, for instance, the
first point of the user defined joint trajectory is very different from :math:`q_{start}` a velocity
limits violation error will abort the motion.

Values for the constants used in the equations below are shown in the `Constants`_ section.

Joint trajectory requirements
-----------------------------

Necessary conditions
********************

1. :math:`q_{min} < q_d < q_{max}`
2. :math:`-\dot{q}_{max} < \dot{q_d} < \dot{q}_{max}`
3. :math:`-\ddot{q}_{max} < \ddot{q_d} < \ddot{q}_{max}`
4. :math:`-\dddot{q}_{max} < \dddot{q_d} < \dddot{q}_{max}`

Recommended conditions
**********************

1. :math:`-\dot{q}_{max} < \dot{q_d} < \dot{q}_{max}`
2. :math:`-\tau_{j, max} < {\tau_j}_d < \tau_{j, max}`
3. :math:`-\dot{\tau}_{j, max} < \dot{\tau_j}_d < \dot{\tau}_{j, max}`

beginning of trajectory:

4. :math:`q = q_d`
5. :math:`\dot{q} = 0`
6. :math:`\ddot{q} = 0`

end of trajectory:

8. :math:`\dot{q} = 0`
9. :math:`\ddot{q} = 0`

Cartesian trajectory requirements
---------------------------------

Necessary conditions
********************

1. :math:`T` is proper transformation matrix
2. :math:`-\dot{p}_{max} < \dot{p_d} < \dot{p}_{max}` (Cartesian velocity)
3. :math:`-\ddot{p}_{max} < \ddot{p_d} < \ddot{p}_{max}` (Cartesian acceleration)

derived from inverse kinematics:

4. :math:`q_{min} < q_d < q_{max}`
5. :math:`-\dot{q}_{max} < \dot{q_d} < \dot{q}_{max}`
6. :math:`-\ddot{q}_{max} < \ddot{q_d} < \ddot{q}_{max}`

Recommended conditions
**********************

1. :math:`q_{min} < q_d < q_{max}`
2. :math:`-\dot{q}_{max} < \dot{q_d} < \dot{q}_{max}`
3. :math:`-\tau_{j, max} < {\tau_j}_d < \tau_{j, max}`
4. :math:`-\dot{\tau}_{j, max} < \dot{{\tau_j}_d} < \dot{\tau}_{j, max}`

beginning of trajectory:

5. :math:`{}^OT_{EE} = {{}^OT_{EE}}_d`
6. :math:`\dot{p} = 0` (Cartesian velocity)
7. :math:`\ddot{p} = 0` (Cartesian acceleration)

end of trajectory:

8. :math:`\dot{p} = 0` (Cartesian velocity)
9. :math:`\ddot{p} = 0` (Cartesian acceleration)

Controller requirements
-----------------------

Necessary conditions
********************

1. :math:`-\dot{\tau}_{j, max} < \dot{{\tau_j}_d} < \dot{\tau}_{j, max}`

Recommended conditions
**********************

1. :math:`-\tau_{j, max} < {\tau_j}_d < \tau_{j, max}`

beginning of trajectory:

2. :math:`\tau_j = 0`

.. _limit_table:

Constants
---------

Limits in the Cartesian space are as follows:

+-------------------------+-------------+-----------------------+------------+-----------+-------------------------+
|          Name           | Translation |         Unit          |  Rotation  |   Elbow   |          Unit           |
+=========================+=============+=======================+============+===========+=========================+
| :math:`\dot{p}_{max}`   | 1.8700      | :math:`\frac{m}{s}`   | 2.7500     | 2.3925    | :math:`\frac{rad}{s}`   |
+-------------------------+-------------+-----------------------+------------+-----------+-------------------------+
| :math:`\ddot{p}_{max}`  | 14.3000     | :math:`\frac{m}{s^2}` | 27.5000    | 11.0000   | :math:`\frac{rad}{s^2}` |
+-------------------------+-------------+-----------------------+------------+-----------+-------------------------+

Joint space limits are:

.. csv-table::
   :header-rows: 1
   :file: control-parameters-joint.csv

Denavit–Hartenberg parameters
-----------------------------

The Denavit–Hartenberg parameters for the Panda's kinematic chain are as follows:

.. figure:: _static/dh-diagram.png
    :align: center
    :figclass: align-center

    Panda's kinematic chain.

+-------------+----------------+----------------+------------------------+-----------------------+
|  Joint      | :math:`a\;(m)` | :math:`d\;(m)` | :math:`\alpha\;(rad)`  | :math:`\theta\;(rad)` |
+=============+================+================+========================+=======================+
| Joint 1     | 0              | 0.333          | 0                      | :math:`\theta_1`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| Joint 2     | 0              | 0              | :math:`-\frac{\pi}{2}` | :math:`\theta_2`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| Joint 3     | 0              | 0.316          | :math:`\frac{\pi}{2}`  | :math:`\theta_3`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| Joint 4     | 0.0825         | 0              | :math:`\frac{\pi}{2}`  | :math:`\theta_4`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| Joint 5     | -0.0825        | 0.384          | :math:`-\frac{\pi}{2}` | :math:`\theta_5`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| Joint 6     | 0              | 0              | :math:`\frac{\pi}{2}`  | :math:`\theta_6`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| Joint 7     | 0.088          | 0              | :math:`\frac{\pi}{2}`  | :math:`\theta_7`      |
+-------------+----------------+----------------+------------------------+-----------------------+
| flange      | 0              | 0.107          | 0                      | 0                     |
+-------------+----------------+----------------+------------------------+-----------------------+

