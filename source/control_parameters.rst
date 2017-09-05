Control Parameters Specifications
=================================

Control parameters fed into the robot should fulfill *recommended* and *necessary* conditions.
Recommended conditions should be fulfilled to ensure optimal operation of the robot. If necessary
conditions are not met then the motion will be aborted.

Robot's trajectory before and after user specified trajectory complies with
recommended conditions. Robot puts the user specified trajectory in between and makes an
overall trajectory out of it. Therefore, for example, if the first point of the user defined
joint trajectory is very different from :math:`q_{start}` a velocity limits violation error will
abort the motion.

.. hint::

  All the variables used in this documentation are robot dependant and can be found in the
  robot URDF file.

Joint Trajectory Requirements
-----------------------------

Recommended Conditions
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

Necessary Conditions
*********************

1. :math:`q_{min} < q < q_{max}`
2. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
3. :math:`\ddot{q}_{min} < \ddot{q} < \ddot{q}_{max}`
4. :math:`\dddot{q}_{min} < \dot{q} < \dddot{q}_{max}`

Cartesian Trajectory Requirements
---------------------------------

Recommended Conditions
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

Necessary Conditions
********************

1. :math:`T` is proper transformation matrix
2. :math:`\dot{p}_{min} < \dot{p} < \dot{p}_{max}`
3. :math:`\ddot{p}_{min} < \ddot{p} < \ddot{p}_{max}`

derived from inverse kinematics:

4. :math:`q_{min} < q < q_{max}`
5. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
6. :math:`\ddot{q}_{min} < \ddot{q} < \ddot{q}_{max}`

Controller Requirements
-----------------------

Recommended Conditions
**********************

1. :math:`\tau_{j, min} < \tau_j < \tau_{j, max}`
2. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`

beginning of trajectory:

3. :math:`\tau_j = 0`

Necessary Conditions
********************

1. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`


