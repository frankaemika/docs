.. _control_parameters_specifications:

Robot and interface specifications
===================================
Realtime control commands sent to the robot should fulfill *recommended* and *necessary*
conditions. Recommended conditions should be fulfilled to ensure optimal operation of the
robot. If necessary conditions are not met then the motion will be aborted.

The final robot trajectory is the result of processing the user-specified trajectory ensuring
that recommended conditions are fulfilled. As long as necessary conditions are met, the robot
will try to follow the user-provided trajectory but it will only match the final trajectory
if it also fulfills recommended conditions. If the necessary conditions are violated, an error
will abort the motion: if, for instance, the first point of the user defined joint trajectory
is very different from robot start position (:math:`q(t=0) \neq q_c(t=0)`) a ``start_pose_invalid`` error
will abort the motion.

Values for the constants used in the equations below are shown in the `Constants`_ section.

Joint trajectory requirements
-----------------------------

Necessary conditions
********************

1. :math:`q_{min} < q_c < q_{max}`
2. :math:`-\dot{q}_{max} < \dot{q}_c < \dot{q}_{max}`
3. :math:`-\ddot{q}_{max} < \ddot{q}_c < \ddot{q}_{max}`
4. :math:`-\dddot{q}_{max} < \dddot{q}_c < \dddot{q}_{max}`

Recommended conditions
**********************

1. :math:`-{\tau_j}_{max} < {\tau_j}_d < {\tau_j}_{max}`
2. :math:`-\dot{\tau_j}_{max} < \dot{\tau_j}_d < \dot{\tau_j}_{max}`

At the beginning of the trajectory, the following conditions should be fulfilled:

3. :math:`q = q_c`
4. :math:`\dot{q}_{c} = 0`
5. :math:`\ddot{q}_{c} = 0`

At the end of the trajectory, the following conditions should be fulfilled:

6. :math:`\dot{q}_{c} = 0`
7. :math:`\ddot{q}_{c} = 0`

Cartesian trajectory requirements
---------------------------------

Necessary conditions
********************

1. :math:`T` is proper transformation matrix
2. :math:`-\dot{p}_{max} < \dot{p_c} < \dot{p}_{max}` (Cartesian velocity)
3. :math:`-\ddot{p}_{max} < \ddot{p_c} < \ddot{p}_{max}` (Cartesian acceleration)
4. :math:`-\dddot{p}_{max} < \dddot{p_c} < \dddot{p}_{max}` (Cartesian jerk)

Conditions derived from inverse kinematics:

5. :math:`q_{min} < q_c < q_{max}`
6. :math:`-\dot{q}_{max} < \dot{q_c} < \dot{q}_{max}`
7. :math:`-\ddot{q}_{max} < \ddot{q_c} < \ddot{q}_{max}`

Recommended conditions
**********************

Conditions derived from inverse kinematics:

1. :math:`-{\tau_j}_{max} < {\tau_j}_d < {\tau_j}_{max}`
2. :math:`-\dot{\tau_j}_{max} < \dot{{\tau_j}_d} < \dot{\tau_j}_{max}`

At the beginning of the trajectory, the following conditions should be fulfilled:

3. :math:`{}^OT_{EE} = {{}^OT_{EE}}_c`
4. :math:`\dot{p}_{c} = 0` (Cartesian velocity)
5. :math:`\ddot{p}_{c} = 0` (Cartesian acceleration)

At the end of the trajectory, the following conditions should be fulfilled:

6. :math:`\dot{p}_{c} = 0` (Cartesian velocity)
7. :math:`\ddot{p}_{c} = 0` (Cartesian acceleration)

Controller requirements
-----------------------

Necessary conditions
********************

1. :math:`-\dot{\tau_j}_{max} < \dot{{\tau_j}_d} < \dot{\tau_j}_{max}`

Recommended conditions
**********************

1. :math:`-{\tau_j}_{max} < {\tau_j}_d < {\tau_j}_{max}`

At the beginning of the trajectory, the following conditions should be fulfilled:

2. :math:`{\tau_j}_{d} = 0`

.. _limit_table:

Constants
---------

Limits in the Cartesian space are as follows:\

+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+
|          Name          |                 Translation                   |                   Rotation                       |                  Elbow                     |
+========================+===============================================+==================================================+============================================+
| :math:`\dot{p}_{max}`  | 1.7000 :math:`\frac{\text{m}}{\text{s}}`      | 2.5000 :math:`\frac{\text{rad}}{\text{s}}`       | 2.1750 :math:`\frac{rad}{\text{s}}`        |
+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+
| :math:`\ddot{p}_{max}` | 13.0000 :math:`\frac{\text{m}}{\text{s}^2}`   | 25.0000 :math:`\frac{\text{rad}}{\text{s}^2}`    | 10.0000 :math:`\;\frac{rad}{\text{s}^2}`   |
+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+
| :math:`\dddot{p}_{max}`| 6500.0000 :math:`\frac{\text{m}}{\text{s}^3}` | 12500.0000 :math:`\frac{\text{rad}}{\text{s}^3}` | 5000.0000 :math:`\;\frac{rad}{\text{s}^3}` |
+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+

Joint space limits are:

.. csv-table::
   :header-rows: 1
   :file: control-parameters-joint.csv

Denavit–Hartenberg parameters
-----------------------------

The Denavit–Hartenberg parameters for the Panda's kinematic chain are derived following Craig's convention and are as follows:

.. figure:: _static/dh-diagram.png
    :align: center
    :figclass: align-center

    Panda's kinematic chain.

+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
|    Joint    | :math:`a\;(\text{m})` | :math:`d\;(\text{m})` | :math:`\alpha\;(\text{rad})` | :math:`\theta\;(\text{rad})` |
+=============+=======================+=======================+==============================+==============================+
| Joint 1     | 0                     | 0.333                 | 0                            | :math:`\theta_1`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Joint 2     | 0                     | 0                     | :math:`-\frac{\pi}{2}`       | :math:`\theta_2`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Joint 3     | 0                     | 0.316                 | :math:`\frac{\pi}{2}`        | :math:`\theta_3`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Joint 4     | 0.0825                | 0                     | :math:`\frac{\pi}{2}`        | :math:`\theta_4`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Joint 5     | -0.0825               | 0.384                 | :math:`-\frac{\pi}{2}`       | :math:`\theta_5`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Joint 6     | 0                     | 0                     | :math:`\frac{\pi}{2}`        | :math:`\theta_6`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Joint 7     | 0.088                 | 0                     | :math:`\frac{\pi}{2}`        | :math:`\theta_7`             |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+
| Flange      | 0                     | 0.107                 | 0                            | 0                            |
+-------------+-----------------------+-----------------------+------------------------------+------------------------------+


.. note::

    :math:`{}^0T_{1}` is the transformation matrix which describes the position and orientation of
    `frame 1` in `frame 0`. A kinematic chain can be calculated like the following:
    :math:`{}^0T_{2} = {}^0T_{1} * {}^1T_{2}`
