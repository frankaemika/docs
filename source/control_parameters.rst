Control Parameters Specifications
=================================

The control parameters fed into the robot should met some certain specifications. There are two
categories of specification namely recommended and necessary. 

.. hint::

  One should notice that even if the recommended condtions are not met still the robot can
  perform actions under necessary conditions.

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

.. hint::

  The user specified trajectory is plugged into a trajectory which meets the recommended
  conditions for start and end of trajectory. So for example if the first point fo the user defined
  trajectory is very different from q_start an velocity limits violation error would stop the
  operation.


Cartesian Trajectory requirements
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

1. :math:`T` is proper transformationmatrix
2. :math:`\dot{p}_{min} < \dot{p} < \dot{p}_{max}`
3. :math:`\ddot{p}_{min} < \ddot{p} < \ddot{p}_{max}`

Derived from inverse kinematics:

4. :math:`q_{min} < q < q_{max}`
5. :math:`\dot{q}_{min} < \dot{q} < \dot{q}_{max}`
6. :math:`\ddot{q}_{min} < \ddot{q} < \ddot{q}_{max}` 

.. hint::

  The user specified trajectory is plugged into a trajectory which meets the recommended
  conditions for start and end of trajectory. So for example if the first point for the user defined
  trajectory is very different from O_T_EE _start an velocity limits violation error would stop the
  operation.


Controller requirements
-----------------------

Recommended Conditions
**********************

1. :math:`\tau_{j, min} < \tau_j < \tau_{j_max}`
2. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`

beginning of trajectory:

3. :math:`\tau_j = 0`

Necessary Conditions
********************

1. :math:`\dot{\tau}_{j, min} < \dot{\tau}_j < \dot{\tau}_{j, max}`

.. hint::

  The user specified torque trajectory is plugged into a trajectory which meets the
  recommended conditions for start and end of trajectory. So for example if the first point for the
  user defined trajectory is very different from 0 a torque discontinuity error would stop the
  operation.


Finishing the Control
---------------------

Finishing Properly
******************

To properly finish the control one should set the motion finished flag to 'true' and keep sending
the same control parameters till the status corresponding to move command is showing that motion
is finished. Motion finsihed should be sent along with the last sample of control parameters.
This flag basically mean that this sample of control parameters is the last sample. The flag is
important for making sure that the motion has completely and sucessfully taken place.

Cancelling the Contol
*********************

Another way of finishing the control is via stop command. One has two options either sending control
data till he makes sure that stop command state has changed to active or sucessful or he can right
away stop sending the control parameters (putting message id to zero) right after sending the stop
command.
