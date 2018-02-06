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

Values for the constants used in the equations below are shown in the :ref:`following section <limit_table>`.

Joint trajectory requirements
-----------------------------

Recommended conditions
**********************

1. :math:`q_{min} < q < q_{max}`
2. :math:`-\dot{q}_{max} < \dot{q} < \dot{q}_{max}`
3. :math:`-\tau_{j, max} < \tau_j < \tau_{j, max}`
4. :math:`-\dot{\tau}_{j, max} < \dot{\tau}_j < \dot{\tau}_{j, max}`

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
2. :math:`-\dot{q}_{max} < \dot{q} < \dot{q}_{max}`
3. :math:`-\ddot{q}_{max} < \ddot{q} < \ddot{q}_{max}`
4. :math:`-\dddot{q}_{max} < \dot{q} < \dddot{q}_{max}`

Cartesian trajectory requirements
---------------------------------

Recommended conditions
**********************

1. :math:`q_{min} < q < q_{max}`
2. :math:`-\dot{q}_{max} < \dot{q} < \dot{q}_{max}`
3. :math:`-\tau_{j, max} < \tau_j < \tau_{j, max}`
4. :math:`-\dot{\tau}_{j, max} < \dot{\tau}_j < \dot{\tau}_{j, max}`

beginning of trajectory:

5. :math:`{}^OT_{EE} = {}^OT_{EE, start}` (The most recent O_T_EE_d)
6. :math:`\dot{p} = 0` (Cartesian velocity)
7. :math:`\ddot{p} = 0` (Cartesian acceleration)

end of trajectory:

8. :math:`\dot{p} = 0`
9. :math:`\ddot{p} = 0`

Necessary conditions
********************

1. :math:`T` is proper transformation matrix
2. :math:`-\dot{p}_{max} < \dot{p} < \dot{p}_{max}`
3. :math:`-\ddot{p}_{max} < \ddot{p} < \ddot{p}_{max}`

derived from inverse kinematics:

4. :math:`q_{min} < q < q_{max}`
5. :math:`-\dot{q}_{max} < \dot{q} < \dot{q}_{max}`
6. :math:`-\ddot{q}_{max} < \ddot{q} < \ddot{q}_{max}`

Controller requirements
-----------------------

Recommended conditions
**********************

1. :math:`-\tau_{j, max} < \tau_j < \tau_{j, max}`
2. :math:`-\dot{\tau}_{j, max} < \dot{\tau}_j < \dot{\tau}_{j, max}`

beginning of trajectory:

3. :math:`\tau_j = 0`

Necessary conditions
********************

1. :math:`-\dot{\tau}_{j, max} < \dot{\tau}_j < \dot{\tau}_{j, max}`

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
| :math:`\dddot{p}_{max}` | 6500.0000   | :math:`\frac{m}{s^3}` | 12500.0000 | 5000.0000 | :math:`\frac{rad}{s^3}` |
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

