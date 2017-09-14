franka_ros
==========

Before continuing with this chapter, please follow the steps from the
:ref:`building the ROS packages <installing_ros>` section.

The ``franka_ros`` repository contains a variety of packages that are briefly introduced here.
We also give a short how-to for :ref:`writing controllers <write_own_controller>`.

All parameters passed to launch files in this section come with default values, so they
can be omitted if using the default network addresses and ROS namespaces.
Make sure the ``source`` command was called with the setup script from your workspace:

.. code-block:: shell

   source /path/to/catkin_ws/devel/setup.sh


franka_description
------------------
This package contains the description of our robots and end effectors in terms of kinematics,
joint limits, visual surfaces and collision space. The collision space is a simplified version of
the visual description used to improve performance of collision checks. The descriptions are based
on the URDF format according to the `URDF XML documentation <http://wiki.ros.org/urdf/XML>`_ . The
files do NOT contain inertial terms and therefore cannot be used for dynamics simulations (e.g.
gazebo). The package offers launch files which visualize the robot descriptions of the robot and the
gripper.

.. code-block:: shell

    roslaunch franka_description visualize_franka.launch                      # with gripper
    roslaunch franka_description visualize_franka.launch load_gripper:=false  # without gripper
    roslaunch franka_description display_gripper.launch                       # gripper only


franka_gripper
--------------
This package implements the ``franka_gripper_node`` for interfacing a gripper from ROS.
The node publishes the state of the gripper and offers the following `actions servers`:

 * ``franka_gripper::MoveAction(width, speed)``: moves to a target `width` with the defined
   `speed`.
 * ``franka_gripper::GraspAction(width, speed, max_current)``: tries to grasp at the desired
   `width` with the `max_current` while closing with the given `speed`.
 * ``franka_gripper::HomingAction()``: homes the gripper and updates the maximum width given the
   mounted fingers.
 * ``franka_gripper::StopAction()``: aborts a running action. This can be used to stop applying
   forces after grasping.
 * ``control_msgs::GripperCommandAction(width, max_effort)``: A standard gripper action
   recognized by MoveIt!.


You can launch the ``franka_gripper_node`` with:

.. code-block:: shell

    roslaunch franka_gripper franka_gripper.launch robot_ip:=<franka-control-ip>
      arm_id:=<your_robot_namespace>


.. _franka_hw:

franka_hw
---------
This package contains the hardware abstraction of the robot for the ROS control framework
based on the ``libfranka`` API. The hardware class ``franka_hw::FrankaHW`` is implemented in this
package offering the following interfaces to controllers:

+-------------------------------------------------+----------------------------------------------+
|                    Interface                    |                   Function                   |
+=================================================+==============================================+
| ``hardware_interface::JointStateInterface``     | reads joint states.                          |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::VelocityJointInterface``  | commands joint positions and reads joint     |
|                                                 | states.                                      |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::PositionJointInterface``  | commands joint velocities and reads joint    |
|                                                 | states.                                      |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::EffortJointInterface``    | commands joint torques and reads joint       |
|                                                 | states.                                      |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaStateInterface``             | reads the full robot state.                  |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaPoseCartesianInterface``     | commands Cartesian poses and reads the full  |
|                                                 | robot state.                                 |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaVelocityCartesianInterface`` | commands Cartesian velocities and reads the  |
|                                                 | full robot state.                            |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaModelInterface``             | reads the dynamic and kinematic model of the |
|                                                 | robot.                                       |
+-------------------------------------------------+----------------------------------------------+

To use ROS control interfaces, you have to retrieve resource handles by name:

+-------------------------------------------------+----------------------------------------+
|                    Interface                    |          Resource handle name          |
+=================================================+========================================+
| ``hardware_interface::JointStateInterface``     | "<arm_id>_joint1" to "<arm_id>_joint7" |
+-------------------------------------------------+----------------------------------------+
| ``hardware_interface::VelocityJointInterface``  | "<arm_id>_joint1" to "<arm_id>_joint7" |
+-------------------------------------------------+----------------------------------------+
| ``hardware_interface::PositionJointInterface``  | "<arm_id>_joint1" to "<arm_id>_joint7" |
+-------------------------------------------------+----------------------------------------+
| ``hardware_interface::EffortJointInterface``    | "<arm_id>_joint1" to "<arm_id>_joint7" |
+-------------------------------------------------+----------------------------------------+
| ``franka_hw::FrankaStateInterface``             | "<arm_id>_robot"                       |
+-------------------------------------------------+----------------------------------------+
| ``franka_hw::FrankaPoseCartesianInterface``     | "<arm_id>_robot"                       |
+-------------------------------------------------+----------------------------------------+
| ``franka_hw::FrankaVelocityCartesianInterface`` | "<arm_id>_robot"                       |
+-------------------------------------------------+----------------------------------------+
| ``franka_hw::FrankaModelInterface``             | "<arm_id>_robot"                       |
+-------------------------------------------------+----------------------------------------+

The ``franka_hw::FrankaHW`` class also implements the starting, stopping and switching of
controllers. In addition a variety of ROS services are offered to expose the full ``libfranka``
API in the ROS ecosystem. The following services are provided:

 * ``franka_hw::SetJointImpedance`` specifies joint stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_hw::SetCartesianImpedance`` specifies Cartesian stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_hw::SetEEFrame`` specifies the transformation from <arm_id>_EE to <arm_id>_link8 frame.
 * ``franka_hw::SetKFrame`` specifies the transformation from <arm_id>_K to <arm_id>_EE frame.
 * ``franka_hw::SetForceTorqueCollisionBehavior`` sets thresholds for external Cartesian wrenches
   to configure the collision reflex.
 * ``franka_hw::SetFullCollisionBehavior`` sets thresholds for external forces on Cartesian and
   joint level to configure the collision reflex.
 * ``franka_hw::SetLoad`` sets an external load to compensate (e.g. of a grasped object).

.. important::

    The <arm_id>_EE frame is a child of the <arm_id>_link8 frame and denotes the configurable
    end effector frame. The <arm_id>_K frame is a child frame of <arm_id>_EE and marks the center
    of the internal Cartesian impedance. It also serves as a reference frame for external
    wrenches. *Neither the <arm_id>_EE nor the <arm_id>_K are contained in the URDF as they can be
    changed at run time*.

To recover from errors and reflexes the ``franka_hw::ErrorRecoveryAction`` can be called.
That can be done from an action client or by simply publishing on the action goal topic:

.. code-block:: shell

   rostopic pub /<your_robot_namespace>/error_recovery/goal franka_hw/ErrorRecoveryActionGoal "{}"


After recovery, the ``franka_hw_node`` restarts the controllers that were running. That is
possible as the node does not die when robot reflexes are triggered or errors are occurred.
All of these functionalities are provided by the ``franka_hw_node`` which can be launched with the
following command:

.. code-block:: shell

    roslaunch franka_hw franka_hw.launch robot_ip:=<franka-control-ip>
      arm_id:=<your_robot_namespace> load_gripper:=<true/false>


Besides loading the ``franka_hw_node``, the launch file also starts a
``franka_hw::FrankaStateController`` for reading and publishing the robot states, including
external wrenches, configurable transforms and the joint states required for visualization with
rivz. For visualization purposes, a ``robot_state_publisher`` is started together with RViz.


franka_visualization
--------------------
This package contains the ``franka_joint_state_publisher`` which connects to the robot using
the ``libfranka`` API and visualizes it in RViz. To run this package launch:

.. code-block:: shell

    roslaunch franka_visualization franka_visualization.launch robot_ip:=<franka-control-ip>
      load_gripper:=<true/false>


This is pure visualization - no commands are sent to the robot. It can be useful to check the
connection to the robot.

.. important::

    Only one instance of a ``franka::Robot`` can connect to the robot. This means, that for example
    the ``franka_joint_state_publisher`` cannot run in parallel to the ``franka_hw_node``. This
    also implies that you cannot execute the visualization example alongside a separate program
    running a controller.


.. _example_controllers:

franka_example_controllers
--------------------------
In this package a set of example controllers for controlling the robot via ROS are implemented.
The controllers depict the variety of interfaces offered by the ``franka_hw::FrankaHW`` class and
the according usage. Each example comes with a separate stand-alone launch file that starts the
controller on the robot and visualizes it.

To launch the joint impedance example, execute the following command:

.. code-block:: shell

    roslaunch franka_example_controllers joint_impedance_example_controller.launch
      robot_ip:=<franka-control-ip> load_gripper:=<true/false> arm_id:=<your_robot_namespace>

Other examples are started in the same way.


franka_moveit_config
--------------------
This package contains partly auto generated files that connect the FRANKA ARM to the MoveIt! motion
planning framework. It contains three move-groups:

 * franka_with_gripper (default, robot and gripper)
 * franka (robot only)
 * franka_gripper (gripper only)

To control the robot with MoveIt! launch the following three files:

.. code-block:: shell

    # bring up hardware
    roslaunch franka_hw franka_hw.launch robot_ip:=<franka-control-ip>
    arm_id:=<your_robot_namespace>  load_gripper:=<true/false>

    # start a joint_trajectory_controller of type <controller>
    roslaunch franka_moveit_config franka_moveit.launch  arm_id:=<your_robot_namespace>
      controller:=<effort/position>

    # for visualization and GUI-based motion planning and execution
    roslaunch franka_moveit_config moveit_rviz.launch


For more details, documentation and tutorials, please have a look at the
`MoveIt! tutorials website <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>`_.


.. _write_own_controller:

Writing  your own controller
----------------------------
All example controllers from the :ref:`example controllers package<example_controllers>` are
derived from the ``controller_interface::MultiInterfaceController`` class, which allows to claim
up to four interfaces in one controller instance. The declaration of your class then looks like:

.. code-block:: c++

    class NameOfYourControllerClass : controller_interface::MultiInterfaceController <
                                  my_mandatory_first_interface,
                                  my_possible_second_interface,
                                  my_possible_third_interface,
                                  my_possible_fourth_interface> {
      bool init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh);  // mandatory
      void update (const ros::Time& time, const ros::Duration& period);  // mandatory
      void starting (const ros::Time& time)   // optional
      void stopping (const ros::Time& time);  // optional
      ...
    }


The available interfaces are described in Section :ref:`franka_hw <franka_hw>`.

.. important::

    Note that the claimable combinations of commanding interfaces are restricted as it does not
    make sense to e.g. command joint positions and Cartesian poses simultaneously. Read-only
    interfaces like the *JointStateInterface*, the *FrankaStateInterface* or the
    *FrankaModelInterface* can always be claimed and are not subject to restrictions.


Possible claims are:

 * all possible single interface claims
 * *EffortJointInterface* + *PositionJointInterface*
 * *EffortJointInterface* + *VelocityJointInterface*
 * *EffortJointInterface* + *FrankaCartesianPoseInterface*
 * *EffortJointInterface* + *FrankaCartesianVelocityInterface*

The idea behind offering the *EffortJointInterface* in combination with a motion generator
interface is to expose the internal motion generators to the user. The calculated desired joint
pose corresponding to a motion generator command is available in the robot state one time step
later. One use case for this combination would be following a Cartesian trajectory using your own
joint torque controller. In this case you would claim the combination *EffortJointInterface* +
*FrankaCartesianPoseInterface*, stream your trajectory into the *FrankaCartesianPoseInterface*, and
compute your torque commands based on the resulting desired joint pose (q_d) from the robot state.
This allows to use the robot's built-in inverse kinematics instead of having to solve it on
your own.

To implement a fully functional controller you have to implement at least the inherited virtual
functions ``init`` and ``update``. Initializing - e.g. start poses - should be done in the
``starting`` function as ``starting`` is called when restarting the controller, while ``init`` is
called only once when loading the controller. The ``stopping`` method should contain shutdown
related functionality (if needed).

.. important::

    Always command a gentle slowdown before shutting down the controller. When using velocity
    interfaces, do not simply command zero velocity in ``stopping``. Since it might be called
    while the robot is still moving, it would be equivalent to commanding a jump in velocity
    leading to very high resulting torques. In this case it would be better to keep the
    same velocity and stop the controller than sending zeros and let the robot handle
    the slowdown.

Your controller class must be exported correctly with ``pluginlib`` which requires adding:

.. code-block:: c++

    #include <pluginlib/class_list_macros.h>
    // Implementation ..
    PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
                           controller_interface::ControllerBase)


at the end of the ``.cpp`` file. In addition you need to define a ``plugin.xml`` file with the
following content:

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


which is exported by adding:

.. code-block:: xml

    <export>
      <controller_interface plugin="${prefix}/plugin.xml"/>
    </export>


to your package.xml. Further, you need to load at least a controller name in combination with a
controller type to the ROS parameter server. Additionally, you can include other parameters you
need. An exemplary configuration.yaml file can look like:

.. code-block:: yaml

    your_custom_controller_name:
      type: name_of_your_controller_package/NameOfYourControllerClass
      additional_example_parameter: 0.0
      # ..

Now you can start your controller using the ``controller_spawner`` node from ROS control or via the
service calls offered by the ``hardware_manager``. Just make sure that both the
``controller_spawner`` and the ``franka_hw_node`` run in the same namespace. For more details have
a look at the controllers from the :ref:`franka_example_controllers package<example_controllers>`
or the `ROS control tutorials <http://wiki.ros.org/ros_control/Tutorials>`_.
