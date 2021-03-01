franka_ros
==========
.. note::

 ``franka_ros`` is not supported on Windows.

Before continuing with this chapter, please :doc:`install or compile franka_ros <installation_linux>`.

.. figure:: _static/ros-architecture.png
    :align: center
    :figclass: align-center

    Schematic overview of the ``franka_ros`` packages.

The ``franka_ros`` metapackage integrates ``libfranka`` into ROS and ROS control.
Here, we introduce its packages and
we also give a short how-to for :ref:`writing controllers <write_own_controller>`.

All parameters passed to launch files in this section come with default values, so they
can be omitted if using the default network addresses and ROS namespaces.
Make sure the ``source`` command was called with the setup script from your workspace:

.. code-block:: shell

   source /path/to/catkin_ws/devel/setup.sh


franka_description
------------------

This package contains the description of our robots and end effectors in terms of kinematics, joint
limits, visual surfaces and collision space. The collision space is a simplified version of the
visual description used to improve performance of collision checks. The descriptions are based on
the URDF format according to the `URDF XML documentation <http://wiki.ros.org/urdf/XML>`_ .


franka_gripper
--------------
This package implements the ``franka_gripper_node`` for interfacing a gripper from ROS.
The node publishes the state of the gripper and offers the following `actions servers`:

 * ``franka_gripper::MoveAction(width, speed)``: moves to a target `width` with the defined
   `speed`.
 * ``franka_gripper::GraspAction(width, epsilon_inner, epsilon_outer, speed, force)``: tries to
   grasp at the desired `width` with a desired `force` while closing with the given `speed`. The
   operation is successful if the distance :math:`d` between the gripper fingers is:
   :math:`\text{width} - \epsilon_\text{inner} < d < \text{width} + \epsilon_\text{outer}`.
 * ``franka_gripper::HomingAction()``: homes the gripper and updates the maximum width given the
   mounted fingers.
 * ``franka_gripper::StopAction()``: aborts a running action. This can be used to stop applying
   forces after grasping.
 * ``control_msgs::GripperCommandAction(width, max_effort)``: A standard gripper action
   recognized by MoveIt!.


You can launch the ``franka_gripper_node`` with:

.. code-block:: shell

    roslaunch franka_gripper franka_gripper.launch robot_ip:=<fci-ip>

.. hint::

    Starting with ``franka_ros`` 0.6.0, specifying ``load_gripper:=true`` for
    ``roslaunch franka_control franka_control.launch`` will start a ``franka_gripper_node`` as well.


.. _franka_hw:

franka_hw
---------
This package contains the hardware abstraction of the robot for the ROS control framework
based on the ``libfranka`` API. The hardware class ``franka_hw::FrankaHW`` is implemented in this
package offering the following interfaces to controllers:

+-------------------------------------------------+----------------------------------------------+
|                    Interface                    |                   Function                   |
+=================================================+==============================================+
| ``hardware_interface::JointStateInterface``     | Reads joint states.                          |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::VelocityJointInterface``  | Commands joint velocities and reads joint    |
|                                                 | states.                                      |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::PositionJointInterface``  | Commands joint positions and reads joint     |
|                                                 | states.                                      |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::EffortJointInterface``    | Commands joint-level torques and reads       |
|                                                 | joint states.                                |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaStateInterface``             | Reads the full robot state.                  |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaPoseCartesianInterface``     | Commands Cartesian poses and reads the full  |
|                                                 | robot state.                                 |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaVelocityCartesianInterface`` | Commands Cartesian velocities and reads the  |
|                                                 | full robot state.                            |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaModelInterface``             | Reads the dynamic and kinematic model of the |
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

.. hint::

    By default, <arm_id> is set to "panda".

The ``franka_hw::FrankaHW`` class also implements the starting, stopping and switching of
controllers.

The ``FrankaHW`` class also serves as base class for ``FrankaCombinableHW``, a hardware class that
can be combined with others to control multiple robots from a single controller. The combination of
an arbitrary number of Panda robots (number configured by parameters) based on ``FrankaCombinableHW``
for the ROS control framework `<https://github.com/ros-controls/ros_control>`_ is implemented
in ``FrankaCombinedHW``. The key-difference between ``FrankaHW`` and ``FrankaCombinedHW`` is
that the latter supports torque control only.

.. important::

  The ``FrankaCombinableHW`` class is available from version 0.7.0 and allows torque/effort control only.

The ROS parameter server is used to determine at runtime which robots are loaded in the combined
class. For an example on how to configure the ``FrankaCombinedHW`` in the according hardware node,
see :ref:`franka_control <franka_control>`.

.. note::

   The approach of ``FrankaHW`` is optimal for controlling single robots. Thus we recommend using
   the ``FrankaCombinableHW``/``FrankaCombinedHW`` classes only for controlling multiple robots.

The interfaces offered by the ``FrankaCombinableHW``/``FrankaCombinedHW`` classes are the following:

+-------------------------------------------------+----------------------------------------------+
|                    Interface                    |                   Function                   |
+=================================================+==============================================+
| ``hardware_interface::EffortJointInterface``    | Commands joint-level torques and reads       |
|                                                 | joint states.                                |
+-------------------------------------------------+----------------------------------------------+
| ``hardware_interface::JointStateInterface``     | Reads joint states.                          |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaStateInterface``             | Reads the full robot state.                  |
+-------------------------------------------------+----------------------------------------------+
| ``franka_hw::FrankaModelInterface``             | Reads the dynamic and kinematic model of the |
|                                                 | robot.                                       |
+-------------------------------------------------+----------------------------------------------+

The only admissible command interface claim is the ``EffortJointInterface`` which can be combined
with any set of read-only-interfaces (``FrankaModelInterface``, ``JointStateInterface``,
``FrankaStateInterface``). The resource handles offered by all interfaces are claimed by name and
follow the same naming conventions as described for `FrankaHW`. Every instance of
``FrankaCombinableHW`` offers the complete set of service and action interfaces
(see :ref:`franka_control <franka_control>`).

.. note::

   The ``FrankaCombinedHW`` class offers an additional action server in the control node namespace
   to recover all robots. If a reflex or error occurs on any of the robots, the control loop of all
   robots stops until they are recovered.

.. important::

    ``FrankaHW`` makes use of the ROS `joint_limits_interface <http://wiki.ros.org/ros_control#Joint_limits_interface>`_
    to `enforce position, velocity and effort safety limits
    <http://wiki.ros.org/pr2_controller_manager/safety_limits>`_.
    The utilized interfaces are listed below:

     * joint_limits_interface::PositionJointSoftLimitsInterface
     * joint_limits_interface::VelocityJointSoftLimitsInterface
     * joint_limits_interface::EffortJointSoftLimitsInterface

    Approaching the limits will result in the (unannounced) modification of the commands.

.. _franka_control:

franka_control
--------------

The ROS nodes ``franka_control_node`` and ``franka_combined_control_node`` are hardware nodes
for ROS control that use according hardware classes from ``franka_hw``. They provide a variety
of ROS services to expose the full ``libfranka`` API in the ROS ecosystem. The following services
are provided:

 * ``franka_msgs::SetJointImpedance`` specifies joint stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_msgs::SetCartesianImpedance`` specifies Cartesian stiffness for the internal
   controller (damping is automatically derived from the stiffness).
 * ``franka_msgs::SetEEFrame`` specifies the transformation from <arm_id>_EE (end effector) to
   <arm_id>_NE (nominal end effector) frame. The transformation from flange to end effector frame
   is split into two transformations: <arm_id>_EE to <arm_id>_NE frame and <arm_id>_NE to
   <arm_id>_link8 frame. The transformation from <arm_id>_NE to <arm_id>_link8 frame can only be
   set through the administrator's interface.
 * ``franka_msgs::SetKFrame`` specifies the transformation from <arm_id>_K to <arm_id>_EE frame.
 * ``franka_msgs::SetForceTorqueCollisionBehavior`` sets thresholds for external Cartesian
   wrenches to configure the collision reflex.
 * ``franka_msgs::SetFullCollisionBehavior`` sets thresholds for external forces on Cartesian
   and joint level to configure the collision reflex.
 * ``franka_msgs::SetLoad`` sets an external load to compensate (e.g. of a grasped object).

.. important::

    The <arm_id>_EE frame is a child of the <arm_id>_NE frame and denotes the part of the
    configurable end effector frame which can be adjusted during run time through `franka_ros`. The
    <arm_id>_K frame is a child frame of <arm_id>_EE and marks the center of the internal
    Cartesian impedance. It also serves as a reference frame for external wrenches. *Neither the
    <arm_id>_EE nor the <arm_id>_K are contained in the URDF as they can be changed at run time*.
    By default, <arm_id> is set to "panda".

To recover from errors and reflexes the ``franka_msgs::ErrorRecoveryAction`` can be called.
That can be done from an action client or by simply publishing on the action goal topic:

.. code-block:: shell

   rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"


After recovery, the ``franka_control_node`` restarts the controllers that were running. That is
possible as the node does not die when robot reflexes are triggered or errors are occurred.
All of these functionalities are provided by the ``franka_control_node`` which can be launched
with the following command:

.. code-block:: shell

    roslaunch franka_control franka_control.launch robot_ip:=<fci-ip> load_gripper:=<true|false>


Besides loading the ``franka_control_node``, the launch file also starts a
``franka_control::FrankaStateController`` for reading and publishing the robot states, including
external wrenches, configurable transforms and the joint states required for visualization with
rivz. For visualization purposes, a ``robot_state_publisher`` is started.

This package also implements the ``franka_combined_control_node``, a hardware node for ``ros_control`` based
on the ``franka_hw::FrankaCombinedHW`` class. The set of robots loaded are configured via the ROS parameter
server. These parameters have to be in the hardware node's namespace (see `franka_combined_control_node.yaml
<https://github.com/frankaemika/franka_ros/tree/develop/franka_control/config/franka_combined_control_node.yaml>`__
as a reference) and look like this:

.. code-block:: yaml

    robot_hardware:
      - panda_1
      - panda_2
      # (...)

    panda_1:
      type: franka_hw/FrankaCombinableHW
      arm_id: panda_1
      joint_names:
        - panda_1_joint1
        - panda_1_joint2
        - panda_1_joint3
        - panda_1_joint4
        - panda_1_joint5
        - panda_1_joint6
        - panda_1_joint7
      # Configure the threshold angle for printing joint limit warnings.
      joint_limit_warning_threshold: 0.1 # [rad]
      # Activate rate limiter? [true|false]
      rate_limiting: true
      # Cutoff frequency of the low-pass filter. Set to >= 1000 to deactivate.
      cutoff_frequency: 1000
      # Internal controller for motion generators [joint_impedance|cartesian_impedance]
      internal_controller: joint_impedance
      # Configure the initial defaults for the collision behavior reflexes.
      collision_config:
        lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        upper_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]
        upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]
        lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]
        upper_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]

    panda_2:
      type: franka_hw/FrankaCombinableHW
      arm_id: panda_2
      joint_names:
        - panda_2_joint1
        - panda_2_joint2
        - panda_2_joint3
        - panda_2_joint4
        - panda_2_joint5
        - panda_2_joint6
        - panda_2_joint7
      # Configure the threshold angle for printing joint limit warnings.
      joint_limit_warning_threshold: 0.1 # [rad]
      # Activate rate limiter? [true|false]
      rate_limiting: true
      # Cutoff frequency of the low-pass filter. Set to >= 1000 to deactivate.
      cutoff_frequency: 1000
      # Internal controller for motion generators [joint_impedance|cartesian_impedance]
      internal_controller: joint_impedance
      # Configure the initial defaults for the collision behavior reflexes.
      collision_config:
        lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        upper_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  # [Nm]
        lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]
        upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]
        lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]
        upper_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]  # [N, N, N, Nm, Nm, Nm]

    # (+ more robots ...)

.. note::

    Be sure to choose unique and consistent ``arm_id`` parameters. The IDs must match the prefixes
    in the joint names and should be according to the robot description loaded to the control
    node's namespace.

For more information on the parameter based loading of hardware classes, please refer to the
official documentation of ``combined_robot_hw::CombinedRobotHW`` from
`<https://github.com/ros-controls/ros_control>`_.

A second important parameter file
(see franka_ros/franka_control/config/default_combined_controllers.yaml as a reference) configures
a set of default controllers that can be started with the hardware node. The controllers have to match
the launched hardware. The provided default parameterization (here for 2 robots) looks like:

.. code-block:: yaml

    panda_1_state_controller:
      type: franka_control/FrankaStateController
      arm_id: panda_1
      joint_names:
        - panda_1_joint1
        - panda_1_joint2
        - panda_1_joint3
        - panda_1_joint4
        - panda_1_joint5
        - panda_1_joint6
        - panda_1_joint7
      publish_rate: 30  # [Hz]

    panda_2_state_controller:
      type: franka_control/FrankaStateController
      arm_id: panda_2
      joint_names:
        - panda_2_joint1
        - panda_2_joint2
        - panda_2_joint3
        - panda_2_joint4
        - panda_2_joint5
        - panda_2_joint6
        - panda_2_joint7
      publish_rate: 30  # [Hz]

    # (+ more controllers ...)

We provide a launch file to run the ``franka_combined_control_node`` with user specified configuration
files for hardware and controllers which default to a configuration with 2 robots. Launch it with:

.. code-block:: shell

    roslaunch franka_control franka_combined_control.launch \
        robot_ips:=<your_robot_ips_as_a_map>                 # mandatory
        robot:=<path_to_your_robot_description> \
        args:=<xacro_args_passed_to_the_robot_description> \ # if needed
        robot_id:=<name_of_your_multi_robot_setup> \
        hw_config_file:=<path_to_your_hw_config_file>\       # includes the robot ips!
        controllers_file:=<path_to_your_default_controller_parameterization>\
        controllers_to_start:=<list_of_default_controllers_to_start>\
        joint_states_source_list:=<list_of_sources_to_fuse_a_complete_joint_states_topic>

This launch file can be parameterized to run an arbitrary number of robots.
To do so just write your own configuration files in the style of
franka_control/config/franka_combined_control_node.yaml and
franka_ros/franka_control/config/default_combined_controllers.yaml.

.. important::

    Be sure to pass the correct IPs of your robots to `franka_combined_control.launch` as a map.
    This looks like: `{<arm_id_1>/robot_ip: <my_ip_1>, <arm_id_2>/robot_ip: <my_ip_2>, ...}`



.. _ros_visualization:

franka_visualization
--------------------
This package contains publishers that connect to a robot and publish the robot and
gripper joint states for visualization in RViz. To run this package launch:

.. code-block:: shell

    roslaunch franka_visualization franka_visualization.launch robot_ip:=<fci-ip> \
      load_gripper:=<true|false>


This is purely for visualization - no commands are sent to the robot. It can be useful to check the
connection to the robot.

.. important::

    Only one instance of a ``franka::Robot`` can connect to the robot. This means, that for example
    the ``franka_joint_state_publisher`` cannot run in parallel to the ``franka_control_node``.
    This also implies that you cannot execute the visualization example alongside a separate
    program running a controller.


.. _example_controllers:

franka_example_controllers
--------------------------
In this package a set of example controllers for controlling the robot via ROS are implemented.
The controllers depict the variety of interfaces offered by the ``franka_hw::FrankaHW`` class and
the according usage. Each example comes with a separate stand-alone launch file that starts the
controller on the robot and visualizes it.

To launch the joint impedance example, execute the following command:

.. code-block:: shell

    roslaunch franka_example_controllers joint_impedance_example_controller.launch \
      robot_ip:=<fci-ip> load_gripper:=<true|false>

Other single Panda examples are started in the same way.

The ``dual_arm_cartesian_impedance_example_controller`` showcases the control of two Panda robots
based on ``FrankaCombinedHW`` using one realtime controller for fulfilling Cartesian tasks with
an impedance-based control approach. The example controller can be launched with

.. code-block:: shell

  roslaunch franka_example_controllers \
      dual_arm_cartesian_impedance_example_controller.launch \
      robot_id:=<name_of_the_2_arm_setup> \
      robot_ips:=<your_robot_ips_as_a_map> \
      rviz:=<true/false> rqt:=<true/false>

The example assumes a robot configuration according to `dual_panda_example.urdf.xacro` where two
Pandas are mounted at 1 meter distance on top of a box. Feel free to replace this robot description
with one that matches your setup.
The option `rviz` allows to choose whether a visualization should be launched. With `rqt` the user
can choose to launch an rqt-gui which allows an online adaption of the rendered end-effector
impedances at runtime via dynamic reconfigure.

franka_msgs
-----------
This package contains message, service and action types that are primarily used the packages
``franka_hw`` and ``franka_control`` to publish robot states or to expose the libfranka API
in the ROS ecosystem. For more information about the services and actions offered in this
package, please refer to :ref:`franka_control <franka_control>`.


panda_moveit_config
--------------------

.. note::

    This package was moved to the `ros_planning repos <https://github.com/ros-planning/panda_moveit_config>`_.

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


Possible claims to command interfaces are:

+-------------------------------------------------+----------------------------------------------+
|          ``franka_hw::FrankaHW``                | ``franka_combinable_hw::FrankaCombinableHW`` |
+=================================================+==============================================+
|     - all possible single interface claims      |     - ``EffortJointInterface``               |
|     - ``EffortJointInterface`` +                |     - ``EffortJointInterface`` +             |
|       ``PositionJointInterface``                |       ``FrankaCartesianPoseInterface``       |
|     - ``EffortJointInterface`` +                |     - ``EffortJointInterface`` +             |
|       ``VelocityJointInterface``                |       ``FrankaCartesianVelocityInterface``   |
|     - ``EffortJointInterface`` +                |                                              |
|       ``FrankaCartesianPoseInterface``          |                                              |
|     - ``EffortJointInterface`` +                |                                              |
|       ``FrankaCartesianVelocityInterface``      |                                              |
+-------------------------------------------------+----------------------------------------------+

The idea behind offering the *EffortJointInterface* in combination with a motion generator
interface is to expose the internal motion generators to the user. The calculated desired joint
pose corresponding to a motion generator command is available in the robot state one time step
later. One use case for this combination would be following a Cartesian trajectory using your own
joint-level torque controller. In this case you would claim the combination *EffortJointInterface*
+ *FrankaCartesianPoseInterface*, stream your trajectory into the *FrankaCartesianPoseInterface*,
and compute your joint-level torque commands based on the resulting desired joint pose (q_d) from
the robot state. This allows to use the robot's built-in inverse kinematics instead of having to
solve it on your own.

To implement a fully functional controller you have to implement at least the inherited virtual
functions ``init`` and ``update``. Initializing - e.g. start poses - should be done in the
``starting`` function as ``starting`` is called when restarting the controller, while ``init`` is
called only once when loading the controller. The ``stopping`` method should contain shutdown
related functionality (if needed).

.. important::

    Always command a gentle slowdown before shutting down the controller. When using velocity
    interfaces, do not simply command zero velocity in ``stopping``. Since it might be called
    while the robot is still moving, it would be equivalent to commanding a jump in velocity
    leading to very high resulting joint-level torques. In this case it would be better to keep the
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
``controller_spawner`` and the ``franka_control_node`` run in the same namespace. For more details
have a look at the controllers from the
:ref:`franka_example_controllers package<example_controllers>` or the
`ROS control tutorials <http://wiki.ros.org/ros_control/Tutorials>`_.
