ROS getting started
===================

In this section we assume that the steps :ref:`from here <installing_ros>` were conducted meaning
the ``franka_ros`` repository was cloned, successfully integrated into a catkin workspace, built and
the ``setup.sh`` or ``setup.bash`` of the workspace was sourced.

.. code-block:: shell

   source /path/to/catkin_ws/devel/setup.sh


The ``franka_ros`` repository contains a variety of packages that are briefly introduced in the
following before giving a short how-to for :ref:`writing own controllers <write_own_controller>`. In
general all parameters passed to launch files in this section come with default values, so
specifying them explicitely can be omitted in case the defaults are correct.


franka_description
------------------
This package contains the description of the FRANKA ARM and the FRANKA HAND in terms of kinematics,
joint limits, visual surfaces and collision space. The latter is a simplified version of the
more detailed visual description to decrease the computational weight of collision checking. The
descriptions are based on the urdf format according to the `urdf xml documentation
<http://wiki.ros.org/urdf/XML>`_ . The files do NOT contain inertial terms and can therefore not be
used for dynamics simulations (e.g. gazebo). The package offers launch files, allowing to display
the robot descriptions of the arm and the gripper.

.. code-block:: shell

    roslaunch franka_description visualize_franka.launch                      # with gripper
    roslaunch franka_description visualize_franka.launch load_gripper:=false  # without gripper
    roslaunch franka_description display_gripper.launch                       # gripper only


franka_gripper
--------------
This package implements the ``franka_gripper_node`` for interfacing a gripper of type FRANKA HAND
from ROS. The node publishes the joint states of the gripper and offers action servers for the
following actions:

 * ``franka_gripper::MoveAction`` (width, speed) : moves to a target width with the defined speed.
 * ``franka_gripper::GraspAction`` (width, speed, max_current) : tries to grasp at the desired
   width with the maximum current while closing with the given speed.
 * ``franka_gripper::HomingAction`` () : homes the gripper and updates the maximum width given the
   mounted fingers.
 * ``franka_gripper::StopAction`` () : aborts a running action. This can be used to stop applying
   forces after grasping.
 * ``control_msgs::GripperCommandAction`` (width, max_effort) : A standard gripper action
   recognized by moveit.


You can launch the ``franka_gripper_node`` with

.. code-block:: shell

    roslaunch franka_gripper franka_gripper.launch robot_ip:=<your_robot_ip>
      arm_id:=<your_robot_namespace>

.. _franka_hw:

franka_hw
---------
This package contains the hardware abstraction of the FRANKA ARM for the ROS control framework
based on the ``libfranka`` API. The hardware class ``franka_hw::FrankaHW`` is implemented in this
package offering the following interfaces to controllers:

 * ``hardware_interface::JointStateInterface``:  reads joint states.
 * ``hardware_interface::PositionJointInterface``:  commands joint positions and reads joint states
 * ``hardware_interface::VelocityJointInterface``:  commands joint velociteis and reads joint
   states.
 * ``hardware_interface::EffortJointInterface``:  commands joint torques and reads joint states.
 * ``franka_hw::FrankaStateInterface``:  reads the FRANKA robot state.
 * ``franka_hw::FrankaPoseCartesianInterface``:  commands cartesian poses and reads the FRANKA robot
   state.
 * ``franka_hw::FrankaVelocityCartesianInterface``:  commands cartesian velocities and reads the
   FRANKA robot state.
 * ``franka_hw::FrankaModelInterface``:  reads the dynamic and kinematic model of the robot.

.. important::

    In ROS control using interfaces works via resource handles that are retrieved by name. For the
    joint interfaces the handle names are "<arm_id>_joint1" to "<arm_id>_joint7". Cartesian pose and
    velocity interfaces as well as the FrankaStateInterface offer a handle by the name 
    "<arm_id>_robot" while the handle of the model interface is named "<arm_id>_model". 


The ``franka_hw::FrankaHW`` class also implements the starting, stopping and switching of
controllers including the required configuration changes with libfranka. In addition a variety of 
ROS services are offered to expose the full ``libfranka`` API in the ROS ecosystem. The following
services are offered:

 * ``franka_hw::SetJointImpedance`` : sets a joint stiffness for the internal controller
   (damping is automatically derived from the stiffness)
 * ``franka_hw::SetCartesianImpedance`` : sets a Cartesian stiffness for the internal controller
   (damping is automatically derived from the stiffness)
 * ``franka_hw::SetEEFrame`` : sets the transformation from <arm_id>_EE to <arm_id>_link8 frame
 * ``franka_hw::SetKFrame`` : sets the transformation from <arm_id>_K to <arm_id>_EE frame
 * ``franka_hw::SetForceTorqueCollisionBehavior`` : sets thresholds for external Cartesian wrenches
   to configure the collision reflex.
 * ``franka_hw::SetFullCollisionBehavior`` : sets thresholds for external forces on Cartesian and
   joint level to configure the collision reflex.
 * ``franka_hw::SetLoad`` : sets an external load to compensate (e.g. of a grasped object)

.. important::

    The <arm_id>_EE frame is a child of the <arm_id>_link8 frame and denotes the configurable
    end-effector frame. The <arm_id>_K frame is a child frame of <arm_id>_EE and marks the center of
    the internal Cartesian impedance. It also serves as a reference frame for external
    wrenches. Note: Both <arm_id>_EE and <arm_id>_K are not contained in the urdf as they can be 
    changed at run time.


To recover from errors and reflexes the ``franka_hw::ErrorRecoveryAction`` can be called.
That can be done from an action client or by simply publishing on the action goal topic like

.. code-block:: shell

   rostopic pub /<your_robot_namespace>/error_recovery/goal franka_hw/ErrorRecoveryActionGoal "{}"
   

After recovering the ``franka_hw_node`` restarts the controllers that were running. That is possible
as the node does not die when robot reflexes trigger or errors occur. All of this functionality is
run by the ``franka_hw_node`` which can be launched with this command:

.. code-block:: shell

    roslaunch franka_hw franka_hw.launch robot_ip:=<your_robot_ip> arm_id:=<your_robot_namespace>
      load_gripper:=<true/false>


This launch file loads besides the ``franka_hw_node`` a ``franka_hw::FrankaStateController``
reading and publishing the robot states, including external wrenches, configurable transforms and
the joint states required for visualization with rivz. For the latter, a ``robot_state_publisher``
is started together with rviz.


.. _example_controllers:

franka_example_controllers
--------------------------
This package implements a set of exemplary controllers for the use on a FRANKA ARM via ROS control.
The controllers depict the variety of interfaces offered by the ``franka_hw::FrankaHW`` class and
the according usage. Each example comes with a separate stand-alone launch file that launches
everything required to run the controller on the robot and visualize it. A ``robot_ip`` an
``arm_id`` as the robots namespace and a bool ``load_gripper`` are passed as arguments to these
launch files. The joint impedance example can be run by:

.. code-block:: shell

    roslaunch franka_example_controllers joint_impedance_example_controller.launch
      robot_ip:=<your_IP> load_gripper:=<true/false> arm_id:=<your_robot_namespace>



franka_moveit_config
--------------------
This package contains partly auto generated files that connect the FRANKA ARM to the moveit motion
planning framework. It contains three move-groups:

 * franka_with_gripper  (default, arm with gripper)
 * franka  (arm without gripper)
 * franka_gripper  (gripper end-effector only)

To move the FRANKA ARM with moveit launch the following three files:

.. code-block:: shell

    roslaunch franka_hw franka_hw.launch robot_ip:=<your_robot_ip> arm_id:=<your_robot_namespace>
      load_gripper:=<true/false>     # bring up hardware
    
    roslaunch franka_moveit_config franka_moveit.launch  arm_id:=<your_robot_namespace>
      controller:=<effort/position>  # start a joint_trajectory_controller of type <controller>
    
    roslaunch franka_moveit_config moveit_rviz.launch  # for visualization and GUI-based
                                                       # motion planning and execution


For more details, documentation and tutorials on the moveit API please refere to
`the tutorials website <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>`_.


franka_visualization
--------------------
This package contains the ``franka_joint_state_publisher`` which connects to a FRANKA ARM
using the ``libfranka`` API and visualizes the robot in rviz. To run it launch

.. code-block:: shell

    roslaunch franka_visualization franka_visualization.launch robot_ip:=<your_robot_ip>
      load_gripper:=<true/false>


This is pure visualization and does not send any commands to the robot. It rather serves to check
the communication with the robot.

.. important::

    Only one instance of a ``franka::Robot`` can connect to the robot meaning the
    ``franka_joint_state_publisher`` cannot run in parallel to e.g. the ``franka_hw_node``.

.. _write_own_controller:

Writing  your own controller
----------------------------
All controllers from  :ref:`the example controllers package<example_controllers>` inherit from the
class ``controller_interface::MultiInterfaceController`` which allows claiming up to four interfaces
for your controller. The declaration of you class then looks like

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
    Note that the claimable combinations of commanding interfaces are restricted as it does not make
    sense to e.g. command joint positions and Cartesian poses simultaneously. Read-only interfaces
    like the JointStateInterface, the FrankaStateInterface or the FrankaModelInterface can always be
    claimed and are not subject to restrictions.
    
    
Possible claims are

 * all possible single interface claims
 * EffortJointInterface + PositionJointInterface
 * EffortJointInterface + VelocityJointInterface 
 * EffortJointInterface + FrankaCartesianPoseInterface
 * EffortJointInterface + FrankaCartesianVelocityInterface

The idea behind offering the EffortJointInterface in combination with a motion generator interface
is to expose the internal motion generators to the user. The answer to the motion generator commands
can allways be read in the robot state one time step later. This can make sense e.g. in the case
where you want to follow a Cartesian trajectory using your own joint torque controller. You would
claim the combination EffortJointInterface + FrankaCartesianPoseInterface, stream your trajectory
into in the FrankaCartesianPoseInterface and compute your torque commands based on the resulting
desired joint pose (q_d) from the robot state. Doing that you would basically use the FRANKA
built-in inverse kinematics instead of having to solve that on your own.

To implement a fully functional controller you have to implement at least the inherited virtual
functions ``init`` and ``update``. Initializing e.g. start poses should be done in ``starting`` as
``starting`` is called when restarting the controller while ``init`` is called only once when
loading the controller. The ``stopping`` method should contain shutdown related functionality
if needed.

.. important::

    When commanding velocities do NOT command zeros in ``stopping`` as it might be called during
    motion which is equivalent to commanding a jump in velocity leading to very high resulting
    torques which can damage your hardware. Rather let the built-in stopping beahvior bring the
    robot to rest in that case. 


To run your own controller, your controller class must be exported correctly with ``pluginlib``
which requires adding

.. code-block:: c++

    #include <pluginlib/class_list_macros.h>
    // Implementation ..
    PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
                           controller_interface::ControllerBase)


to the bottom of your controllers .cpp file. In addition you need to define a plugin.xml file with
the content

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
combination with a controller type to the ROS parameter server. Additionally you can include all
other parameters you need. An exemplary configuration.yaml file can look like:

.. code-block:: yaml

    your_custom_controller_name:
      type: name_of_your_controller_package/NameOfYourControllerClass
      additional_example_parameter: 0.0
      # ..

You can now start your controller using the ``controller_spawner`` node from ROS control or via the
service calls offered by the ``hardware_manager``. Just make sure both the ``controller_spawner``
and the ``franka_hw_node`` run in the same namespace. For more details see the controllers from the
:ref:`franka_example_controllers package<example_controllers>` or the tutorials under
`wiki.ros.org/ros_control/Tutorials <http://wiki.ros.org/ros_control/Tutorials>`_.

