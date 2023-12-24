# Changelog:

## 0.3.1 (23-03-2023)

   - Bugfix. Properly setting the collision threshold values in Simulink.

## 0.3.0 (20-09-2022)

   - **BREAKING** New Matlab object oriented API expressed based on the new `FrankaRobot()` class. Incorporation of existing Franka Matlab functions as methods of the new API Class.
   - Expansion of the Matlab API with the new methods `gripper_state()`, `gripper_homing()`, `gripper_grasp()`, `gripper_move()` and `gripper_stop()` for controlling the Franka Gripper.
   - Expansion of the Matlab API with the new method `joint_trajectory_motion()` for following precomputed joint trajectories.
   - Creation of the new demo `pick_and_place_with_RRT.mlx` showcasing a workflow approach for the new Matlab API.

## 0.3.0 (20-09-2022)

   - Windows 10 support (Experimental mainly due to the non-Real-Time nature of the default Windows system).
   - Project now relies on the leaner "Generic Real-Time" .tlc (grt.tlc) target framework.
   - Support for XCP communication protocol (vehicle network communication). Data inspector is now enabled!
   - Support for "Run on custom Hardware" Simulink App for controlling the "Build-deploy-execute-connect" workflow. 
   - Project back-end overal simplification with modified rt_main.cpp for handling the external mode as a seperate thread.
   - **BREAKING** all the Franka Matlab functions are starting with the "`franka_`" prefix.
   - Expansion of Matlab library with the functions `franka_communication_test()`, `franka_joint_poses()`, `franka_robot_state()` and `franka_joint_point_to_point_motion()`.
   - Addition of the Simulink demo, "joint_impedance_control.slx".
   - Fixing the bug when utilizing the Control Modes "Torque Control - X`.

## 0.2.1 (29-04-2022)

   - Adding supoort for all versions from Matlab2019a till Matlab2021a with libfranka 0.9.0.

## 0.2.0 (31-05-2021)

   - franka_matlab upgrade, supports Matlab2019a, Matlab2019b, Matlab2020a, Matlab2020b, Matlab2021a & libfranka 0.8.0.

## 0.1.1 (01-07-2020)

   - Any dependences that lead to source code linking for the Simulink Franka Library during mexing removed. That fixes the memory corruption
     bug when shutting down Matlab.
   - Simulink Franka Library sFunctions C++ and tlc implementations decoupled from src code necessary for automatic code gen. SRC Folder can be treated seperately as a C++ project.

## 0.1.0 (21-01-2020)
  
  - Features:
    - **Simulink Library** for **Panda Robot**, includes the following blocks:
        - **Franka Simulink Iterface** for applying the desired control, plus additional parameters.
        - **Read Initial Robot State** for reading initial values during the first execution step for any desirable signal. The set of the desired signals can be set through the mask in free text form.
        - **Read Robot State** for reading the values of any desirable signal during execution. The set of the desired signals can be set through the mask in free text form.
        - **Panda Model** for reading the values of all Model parameters of the Panda Robot during execution.
        - **Duration Period** for reading the current step, sample time. If communication is not secured during the 1ms, the block will return the value of 2ms or 3ms etc.
        - **Gripper Read State** for reading the current values out of the Panda Gripper.
    - **franka_emika_panda.tlc** & **franka_emika_panda_shrlib.tlc** custom linux targets, based on ert, that offer ext mode that is real time capable(package drop in case of main step frame loss).
    - **Matlab Library**(Experimental, limited support), includes the following command:
        - **automatic_error_recovery(robot_ip)**. Execute through a matlab command line for getting automatically out of an error state.
<<<<<<< HEAD
    - **Simulink Library misc**(Experimental, limited support) that includes a set of UI buttons with callback scripts with the potential to automate some of the dev. workflow.
=======
    - **Simulink Library misc**(Experimental, limited support) that includes a set of UI buttons with callback scripts with the potential to automate some of the dev. workflow. 
>>>>>>> b769b4d... updated franka matlab section due to new matlab api
