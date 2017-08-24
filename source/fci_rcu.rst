Control Specifications
===============

Joint Trajectory requirements
---------------------
Every joint trajectory should fulfill the following requirements for an optimal result:

1. q_min < q < q_max
2. dq_min < dq < dq_max
3. tau_j_min <tau_j < tau_j_max
4. dtau_j_min < dtau_j < dtau_j_maxi

One should notice that even if the above mentioned condtions are not met still the robot can perform actions under following condtions:

1. q_min < q < q_max
2. dq_min < dq < dq_max
3. ddq_min < ddq < ddq_max
4. dddq_min < dq < dddq_max

One should notice that every joint trajectory is assumed to start from :

1. q = q_start (The most recent q_d)
2. dq = 0
3. ddq = 0

One should notice that every joint trajectory is assumed to end at: 

1. dq = 0
2. ddq = 0


Cartesian Trajectory requirements
---------------------

Every joint trajectory should fulfill the following requirements for an optimal result:

1. q_min < q < q_max
2. dq_min < dq < dq_max
3. tau_j_min <tau_j < tau_j_max
4. dtau_j_min < dtau_j < dtau_j_max

One should notice that even if the above mentioned condtions are not met still the robot can perform actions under following condtions

1. T is proper transformationmatrix
2. dp_min < dp < dp_max
3. ddp_min < ddp < ddp_max
Derived from inverse kinematics
4. q_min < q < q_max
5. dq_min < dq < dq_max
6. ddq_min < ddq < ddq_max 

One should notice that every cartesian trajectory is assumed to start from : 

1. O_T_EE = O_T_EE_start (The most recent O_T_EE_d)
2. dp = 0
3. ddp = 0

One should notice that every cartesian trajectory is assumed to end: 

1. dp = 0
2. ddp = 0

Controller requirements
---------------------
Every controller should fulfill the following requirements for an optimal result:
1. tau_j_min <tau_j < tau_j_max
2. dtau_j_min < dtau_j < dtau_j_max
One should notice that even if the above mentioned condtions are not met still the robot can perform actions under following condtions
1. dtau_j_min < dtau_j < dtau_j_max

Table of Erros
---------------------
