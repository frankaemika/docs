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

Values for the constants used in the equations below are shown in the `Limits for Panda`_  and `Limits for Franka Research 3`_ section.

Joint trajectory requirements
-----------------------------

Necessary conditions
********************

- :math:`q_{min} < q_c < q_{max}`
- :math:`-\dot{q}_{max} < \dot{q}_c < \dot{q}_{max}`
- :math:`-\ddot{q}_{max} < \ddot{q}_c < \ddot{q}_{max}`
- :math:`-\dddot{q}_{max} < \dddot{q}_c < \dddot{q}_{max}`

Recommended conditions
**********************

- :math:`-{\tau_j}_{max} < {\tau_j}_d < {\tau_j}_{max}`
- :math:`-\dot{\tau_j}_{max} < \dot{\tau_j}_d < \dot{\tau_j}_{max}`

At the beginning of the trajectory, the following conditions should be fulfilled:

- :math:`q = q_c`
- :math:`\dot{q}_{c} = 0`
- :math:`\ddot{q}_{c} = 0`

At the end of the trajectory, the following conditions should be fulfilled:

- :math:`\dot{q}_{c} = 0`
- :math:`\ddot{q}_{c} = 0`

Cartesian trajectory requirements
---------------------------------

Necessary conditions
********************

- :math:`T` is proper transformation matrix
- :math:`-\dot{p}_{max} < \dot{p_c} < \dot{p}_{max}` (Cartesian velocity)
- :math:`-\ddot{p}_{max} < \ddot{p_c} < \ddot{p}_{max}` (Cartesian acceleration)
- :math:`-\dddot{p}_{max} < \dddot{p_c} < \dddot{p}_{max}` (Cartesian jerk)

Conditions derived from inverse kinematics:

- :math:`q_{min} < q_c < q_{max}`
- :math:`-\dot{q}_{max} < \dot{q_c} < \dot{q}_{max}`
- :math:`-\ddot{q}_{max} < \ddot{q_c} < \ddot{q}_{max}`

Recommended conditions
**********************

Conditions derived from inverse kinematics:

- :math:`-{\tau_j}_{max} < {\tau_j}_d < {\tau_j}_{max}`
- :math:`-\dot{\tau_j}_{max} < \dot{{\tau_j}_d} < \dot{\tau_j}_{max}`

At the beginning of the trajectory, the following conditions should be fulfilled:

- :math:`{}^OT_{EE} = {{}^OT_{EE}}_c`
- :math:`\dot{p}_{c} = 0` (Cartesian velocity)
- :math:`\ddot{p}_{c} = 0` (Cartesian acceleration)

At the end of the trajectory, the following conditions should be fulfilled:

- :math:`\dot{p}_{c} = 0` (Cartesian velocity)
- :math:`\ddot{p}_{c} = 0` (Cartesian acceleration)

Controller requirements
-----------------------

Necessary conditions
********************

- :math:`-\dot{\tau_j}_{max} < \dot{{\tau_j}_d} < \dot{\tau_j}_{max}`

Recommended conditions
**********************

- :math:`-{\tau_j}_{max} < {\tau_j}_d < {\tau_j}_{max}`

At the beginning of the trajectory, the following conditions should be fulfilled:

- :math:`{\tau_j}_{d} = 0`

.. _limit_table:

Limits for Panda
----------------

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
   :file: control-parameters-joint-panda.csv

The arm can reach its maximum extension when joint 4 has angle :math:`q_{elbow-flip}`, where :math:`q_{elbow-flip} = -0.467002423653011\:rad`.
This parameter is used to determine the flip direction of the elbow.

Limits for Franka Research 3
----------------------------

Limits in the Cartesian space are as follows:\

+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+
|          Name          |                 Translation                   |                   Rotation                       |                  Elbow                     |
+========================+===============================================+==================================================+============================================+
| :math:`\dot{p}_{max}`  | 1.7000 :math:`\frac{\text{m}}{\text{s}}`      | 2.5000 :math:`\frac{\text{rad}}{\text{s}}`       | 2.6200 :math:`\frac{rad}{\text{s}}`        |
+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+
| :math:`\ddot{p}_{max}` |  9.0000 :math:`\frac{\text{m}}{\text{s}^2}`   | 17.0000 :math:`\frac{\text{rad}}{\text{s}^2}`    | 10.0000 :math:`\;\frac{rad}{\text{s}^2}`   |
+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+
| :math:`\dddot{p}_{max}`| 4500.0000 :math:`\frac{\text{m}}{\text{s}^3}` |  8500.0000 :math:`\frac{\text{rad}}{\text{s}^3}` | 5000.0000 :math:`\;\frac{rad}{\text{s}^3}` |
+------------------------+-----------------------------------------------+--------------------------------------------------+--------------------------------------------+

Joint space limits are:

.. csv-table::
   :header-rows: 1
   :file: control-parameters-joint-fr3.csv

The arm can reach its maximum extension when joint 4 has angle :math:`q_{elbow-flip}`, where :math:`q_{elbow-flip} = -0.467002423653011\:rad`.
This parameter is used to determine the flip direction of the elbow.


.. important::

    Note that the maximum joint velocity depends on the joint position. The maximum and minimum joint velocities at a certain joint position are calculated as:

    :math:`\dot{q}_1(q_1)_{max} = min( 2.62, \max(0, -0.3  + \sqrt{\max(0, 12   * ( 2.7501  - q_1))}))`
    :math:`\dot{q}_2(q_2)_{max} = min( 2.62, \max(0, -0.2  + \sqrt{\max(0, 5.17 * ( 1.7918  - q_2))}))`
    :math:`\dot{q}_3(q_3)_{max} = min( 2.62, \max(0, -0.2  + \sqrt{\max(0, 7    * ( 2.9065  - q_3))}))`
    :math:`\dot{q}_4(q_4)_{max} = min( 2.62, \max(0, -0.3  + \sqrt{\max(0, 8    * (-0.1458  - q_4))}))`
    :math:`\dot{q}_5(q_5)_{max} = min( 5.26, \max(0, -0.35 + \sqrt{\max(0, 34   * ( 2.8101  - q_5))}))`
    :math:`\dot{q}_6(q_6)_{max} = min( 4.18, \max(0, -0.35 + \sqrt{\max(0, 11   * ( 4.5205  - q_6))}))`
    :math:`\dot{q}_7(q_7)_{max} = min( 5.26, \max(0, -0.35 + \sqrt{\max(0, 34   * ( 3.0196  - q_7))}))`

    :math:`\dot{q}_1(q_1)_{min} = max(-2.62, \min(0,  0.3  - \sqrt{\max(0, 12   * ( 2.7501  + q_1))}))`
    :math:`\dot{q}_2(q_2)_{min} = max(-2.62, \min(0,  0.2  - \sqrt{\max(0, 5.17 * ( 1.7918  + q_2))}))`
    :math:`\dot{q}_3(q_3)_{min} = max(-2.62, \min(0,  0.2  - \sqrt{\max(0, 7    * ( 2.9065  + q_3))}))`
    :math:`\dot{q}_4(q_4)_{min} = max(-2.62, \min(0,  0.3  - \sqrt{\max(0, 8    * ( 3.0481  + q_4))}))`
    :math:`\dot{q}_5(q_5)_{min} = max(-5.26, \min(0,  0.35 - \sqrt{\max(0, 34   * ( 2.8101  + q_5))}))`
    :math:`\dot{q}_6(q_6)_{min} = max(-4.18, \min(0,  0.35 - \sqrt{\max(0, 11   * (-0.54092 + q_6))}))`
    :math:`\dot{q}_7(q_7)_{min} = max(-5.26, \min(0,  0.35 - \sqrt{\max(0, 34   * ( 3.0196  + q_7))}))`

.. list-table:: Visualization of the joint limits of FR3
   :class: borderless

   * - .. figure:: _static/pbv_limits_j1.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 1

     - .. figure:: _static/pbv_limits_j2.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 2

   * - .. figure:: _static/pbv_limits_j3.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 3

     - .. figure:: _static/pbv_limits_j4.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 4

   * - .. figure:: _static/pbv_limits_j5.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 5

     - .. figure:: _static/pbv_limits_j6.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 6

   * - .. figure:: _static/pbv_limits_j7.svg
            :align: center
            :figclass: align-center

            Velocity limits of Joint 7
     -





As most motion planners can only deal with fixed velocity limits (rectangular limits), we are providing here a suggestion on which values to use for them.

.. csv-table::
   :header-rows: 1
   :file: control-parameters-joint-fr3-rectangular.csv

These limits are only a suggestion, you are free to define your own rectangles within the specification. However, these are the values that are
used in the rate limiter and in the URDF inside :doc:`franka_ros`.

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
