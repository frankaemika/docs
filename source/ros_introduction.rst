ROS getting started
===================

In this section we assume that the ``franka_ros`` repository was cloned and successfully integrated
into a catkin workspace, built and the ``setup.sh`` or ``setup.bash`` of the workspace was sourced.
.. code-block:: shell

   source /path/to/catkin_ws/devel/setup.sh


``franka_description``
----------------------
This package contains the description of the franka ARM and a franka HAND in terms of kinematics,
limits, visible surfaces and a collisioń space. The latter is a simplified version of the more
detailed visual description to decrease the computational weight of collision detection. The
descriptions are based on the urdf format (according to http://wiki.ros.org/urdf/XML (TODO link)).
The files do NOT contain inertial terms, as they are considered confidential, and can therefore
not be used for dynamics simulations (e.g. gazebo). The package contains to launch files, allowing
to display the robot descriptions of the arm and the gripper. To visualize e.g. the franka ARM
with a franka HAND mounted run

.. code-block:: shell

    roslaunch franka_description visualize_franka.launch                      # with gripper
    roslaunch franka_description visualize_franka.launch load_gripper:=false  # without gripper


franka_example_controllers
--------------------------
    This package implements a set of exemplary controllers for use on a franka ARM via the
    ROS control framework. The controllers depict the variety in interfaces offered by the
    ``FrankaHW`` class and the according usage. Each example comes with a separate stand-alone
    launch file by the according name that launches everything required to run the controller on the
    robot and visualize it. As arguments to these launchfiles, a ``robot_ip`` and a bool
    ``load_gripper`` are added. E.g. the joint impedance example can be run by

.. code-block:: shell

    roslaunch franka_example_controllers joint_impedance_example_controller.launch
    robot_ip:=<your_IP> load_gripper:=true


   Writing  your own controller:
   All example controllers are implemented inheriting from the class
   ``controller_interface::MultiInterfaceController`` which allows to claim up to four interfaces
   for your controller and which we recommend to use. To write your own controller, your controller
   class must be exported correctly with ``pluginlib`` which requires adding

.. code-block:: c++

    #include <pluginlib/class_list_macros.h>
    // Implementation ..
    PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
    controller_interface::ControllerBase)


to your implementation. In addition you need to define a plugin.xml file with the content
.. code-block:: xml

    <library path="lib/lib<name_of_your_controller_library>">
    <class name="name_of_your_controller_package/NameOfYourControllerClass"
    type="name_of_your_controller_package::NameOfYourControllerClass"
    base_class_type="controller_interface::ControllerBase">
    <description>
    Some text to describe what your controller is doing
    </description>
    </class>
    </library>


which is exported by adding
.. code-block:: xml

    <export>
    <controller_interface plugin="${prefix}/plugin.xml"/>
    </export>


to your package.xml. To run your controller you need to load at least a controller name in
in combination with a controller type to the parameter server. Additionally you can include all
other parameters you need to configure. An exemplary configuration.yaml file can look like:
.. code-block:: yaml

    your_custom_controller_name:
      type: name_of_your_controller_package/NameOfYourControllerClass
      additional_example_parameter: 0.0
      # ..

You can now start your using the ``controller_spawner`` node from ROS control or via the service
calls offered by the ``hardare_manager``. Just make sure both the ``controller_spawner`` and the
hardware node are run in the same namespace. For more details see the example controllers
or the tutorials under <http://wiki.ros.org/ros_control/Tutorials>.


franka_gripper
--------------
This package contains the ``franka_gripper_node`` node that allows to interface a gripper of type
Franka HAND from ROS. The node publishes the joint states of the gripper and offer action servers
for the following actions:
* ``franka_gripper::MoveAction`` (width, speed)  :  moves to a target width with the defined speed
* ``franka_gripper::GraspAction`` (width, speed, max_current)  :  tries to grasp at the desired
width with the maximum current while closing with the given speed
* ``franka_gripper::HomingAction`` ()  :  homes the gripper and updates the maximum width given the
mounted fingers
* ``franka_gripper::StopAction`` ()  :  aborts a running action. This can be used to stop applying
forces after grasping
* ``control_msgs::GripperCommandAction`` (width, max_effort)  :  A standard gripper action
recognized by moveit

You can launch the ``franka_gripper_node`` with
.. code-block:: shell

    roslaunch franka_gripper franka_gripper.launch robot_ip:=<your_robot_ip>
    arm_id:=<your_robot_namespace>


franka_hw
---------
This package contains the hardware abstraction of the franka ARM for the ROS control framework
based on the ``libfranka`` API. The hardware class ``franka_hw::FrankaHW`` is defined that
offers the following interfaces for use in in user-defined controllers:
* ``hardware_interface::JointStateInterface``    ..reads joint states
* ``hardware_interface::PositionJointInterface`` ..commands joint positions and reads joint states
* ``hardware_interface::VelocityJointInterface`` ..commands joint velociteis and reads joint states
* ``hardware_interface::EffortJointInterface``   ..commands joint torques and reads joint states
* ``franka_hw::FrankaStateInterface``   ..reads the complete franka robot state
* ``franka_hw::FrankaPoseCartesianInterface``   ..commands cartesian poses and reads the robot state
* ``franka_hw::FrankaVelocityCartesianInterface``   ..commands cartesian velocities and reads the
robot state
* ``franka_hw::FrankaModelInterface``  ..reads the dynamic and kinematic model of the robot

The class also implements the starting, stopping and switching of controllers and the required
configuration changes with libfranka. In addition a variety of ROS services are offered to expose
the full ``libfranka`` API in the ROS ecosystem. The following services are offered:
* ``franka_hw::SetJointImpedance``  ..sets a joint stiffness for the internal controller
(damping is automatically derived from the stiffness)
* ``franka_hw::SetCartesianImpedance``  ..sets a Cartesian stiffness for the internal controller
(damping is automatically derived from the stiffness)
* ``franka_hw::SetEEFrame``  ..sets the transformation from EE to F frame
* ``franka_hw::SetKFrame``  ..sets the transformation from K to EE frame
* ``franka_hw::SetForceTorqueCollisionBehavior``
* ``franka_hw::SetFullCollisionBehavior``
* ``franka_hw::SetLoad``
* ``franka_hw::SetTimeScalingFactor``

.. important::

    The F frame is equivalent the frame <arm_id>_link8 from the urdf in the
    ``franka_description`` package. Its childframe is the <arm_id>_EE frame which denotes the
    end-effector frame which can be configured. The K frame is a child frame of EE and denotes
    the center of the internal Cartesian impedance (if used). It also serves as reference frame
    for external wrenches.


To recover from errors and triggered reflexes the ``franka_hw::ErrorRecoveryAction`` action can
be called. All of this functionality is contained in the ``franka_hw_node`` which can be launched
with the command
.. code-block:: shell

    roslaunch franka_hw franka_hw.launch robot_ip:=<your_robot_ip> arm_id:=<your_robot_namespace>
    load_gripper:=<true/false>


This launch file loads besides the ``franka_hw_node`` a ``franka_hw::FrankaStateController``
reading and publishing the robot states,including external wrenches, EE and F frame transforms
and the joint states required for visualization with rivz. For the latter, a
``robot_state_publisher`` is started together with rviz.


franka_moveit_config
--------------------
This package contains partly autogenerated files that connect Franka to the moveit motion
planning framework. It contains three movegroups:
* franka  (arm without gripper)
* franka_with_gripper  (default, arm with gripper)
* franka_gripper  (gripper end-effector only)
To move a Franka arm with moveit launch the following
.. code-block:: shell

    roslaunch franka_hw franka_hw.launch robot_ip:=<your_robot_ip> arm_id:=<your_robot_namespace>
    load_gripper:=<true/false>     # bring up hardware
    roslaunch franka_moveit_config franka_moveit.launch  arm_id:=<your_robot_namespace>
    controller:=<effort/position>  # start a joint_trajectory_controller of type controller
    roslaunch franka_moveit_config moveit_rviz.launch  # for visualization and GUI-based
    motion planning and execution


franka_visualization
--------------------
This package contains the ``franka_joint_state_publisher`` which connects to a Franka ARM
using ``libfranka`` and visualizes the robot in rviz. To run it launch
.. code-block:: shell

    roslaunch franka_visualization franka_visualization.launch robot_ip:=<your_robot_ip>
    load_gripper:=<true/false>


This is pure visualization and does not communicate any commands to the robot. It rather serves
to as a check for the communication with the robot.

.. important::

    Only one instance of a ``franka::Robot`` can connect to the robot meaning the
    ``franka_joint_state_publisher`` cannot run in parallel to e.g. the control frame work from
    ``franka_hw``

    
