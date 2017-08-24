Control Parameters Specifications
=================================

The control parameters fed into the robot should met some certain specifications. There are two categories of specification namely recommended and nececcary. 

.. hint::
  One should notice that even if the recommended condtions are not met still the robot can perform actions under necessary conditions

.. hint::
  All the variables used in this documentation are robot dependant and can be found in the robot URDF file.

Joint Trajectory Requirements
-----------------------------

Recommended Conditions
**********************

1. q_min < q < q_max
2. dq_min < dq < dq_max
3. tau_j_min < tau_j < tau_j_max
4. dtau_j_min < dtau_j < dtau_j_maxi

beginning of trajectory:

5. q = q_start (The most recent q_d)
6. dq = 0
7. ddq = 0

end of trajectory:

8. dq = 0
9. ddq = 0

Necessary Conditions
*********************
1. q_min < q < q_max
2. dq_min < dq < dq_max
3. ddq_min < ddq < ddq_max
4. dddq_min < dq < dddq_max

.. hint::
  The user specified trajectory is plugged into a trajectory which meets the recommended conditions for start and end of trajectory. So for example if the first point fo the user defined trajectory is very different from q_start an velocity limits violation error would stop the operation.


Cartesian Trajectory requirements
---------------------------------

Recommended Conditions
**********************

1. q_min < q < q_max
2. dq_min < dq < dq_max
3. tau_j_min < tau_j < tau_j_max
4. dtau_j_min < dtau_j < dtau_j_max

beginning of trajectory:

5. O_T_EE = O_T_EE_start (The most recent O_T_EE_d)
6. dp = 0
7. ddp = 0

end of trajectory: 

8. dp = 0
9. ddp = 0

Necessary Conditions
********************

1. T is proper transformationmatrix
2. dp_min < dp < dp_max
3. ddp_min < ddp < ddp_max

Derived from inverse kinematics:

4. q_min < q < q_max
5. dq_min < dq < dq_max
6. ddq_min < ddq < ddq_max 

.. hint::
  The user specified trajectory is plugged into a trajectory which meets the recommended conditions for start and end of trajectory. So for example if the first point for the user defined trajectory is very different from O_T_EE
  _start an velocity limits violation error would stop the operation.


Controller requirements
-----------------------

Recommended Conditions
**********************

1. tau_j_min < tau_j < tau_j_max
2. dtau_j_min < dtau_j < dtau_j_max

beginning of trajectory:

3. tau_j = 0

Necessary Conditions
********************

1. dtau_j_min < dtau_j < dtau_j_max

.. hint::
  The user specified torque trajectory is plugged into a trajectory which meets the recommended conditions for start and end of trajectory. So for example if the first point for the user defined trajectory is very different from 0 a torque discontinuity error would stop the operation.


