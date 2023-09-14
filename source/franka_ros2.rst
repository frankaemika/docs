franka_ros2
===========
.. note::

 ``franka_ros2`` is not supported on Windows.

The `franka_ros2 repo <https://github.com/frankaemika/franka_ros2>`_ contains a ROS 2 integration of
:doc:`libfranka <libfranka>`.

.. caution::
    franka_ros2 is in rapid development. Anticipate breaking changes. Report bugs on
    `GitHub <https://github.com/frankaemika/franka_ros2/issues>`_.


Prerequisites
-------------

* A `ROS 2 Humble installation <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`_
  (ros-humble-desktop) or a VSCode IDE with DevContainer.
* A :ref:`PREEMPT_RT kernel <preempt>` (optional, but strongly recommended).
* A system-wide :ref:`libfranka installation <build-libfranka>`. Minimum supported version of libfranka is 0.11.0.
  Here is a minimal example:

.. code-block:: shell

   sudo apt install -y libpoco-dev libeigen3-dev
   git clone https://github.com/frankaemika/libfranka.git --recursive
   cd libfranka
   git switch 0.11.0
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  ..
   cmake --build . -j$(nproc)
   cpack -G DEB
   sudo dpkg -i libfranka-*.deb

Optional .bashrc Settings
^^^^^^^^^^^^^^^^^^^^^^^^^

* To get colorized warn and error messages you can put
  ``export RCUTILS_COLORIZED_OUTPUT=1`` into your ``.bashrc``.

* If your system language is not set to American English you should put
  ``export LC_NUMERIC=en_US.UTF-8`` into your ``.bashrc`` to avoid issues in RViz.

Setup
------

Install From Source
^^^^^^^^^^^^^^^^^^^

1. Install requirements::

    sudo apt install -y \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-control-msgs \
    ros-humble-xacro \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-moveit \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    python3-colcon-common-extensions

2. Create a ROS 2 workspace::

    mkdir -p ~/franka_ros2_ws/src

3. Clone repo and build packages::

    source /opt/ros/humble/setup.bash
    cd ~/franka_ros2_ws
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.sh

Use VSCode DevContainer
^^^^^^^^^^^^^^^^^^^^^^^
FrankaROS2 package comes with .devcontainer folder which enables developer to use FrankaROS2 packages without manually installing ROS2 or libfranka.
VSCode DevContainer working schematic is shown in the below image:

  .. figure:: _static/vscode.png
    :align: center
    :figclass: align-center

1. Follow the setup guide from VSCode `devcontainer_setup <https://code.visualstudio.com/docs/devcontainers/tutorial>`_.

2. Create a ROS 2 workspace::

    mkdir franka_ros2_ws
    cd franka_ros2_ws

3. Clone repo::

    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2

4. Move the .devcontainer folder to the frnaka_ros2_ws parent folder::

    mv franka_ros2/.devcontainer .

5. Open VSCode::

    code .

6. Open the current folder in DevContainer::

    ctrl + shift + p

   Write in the command prompt bar::

    open folder in container

   and click this option in the search results

7. Open up the terminal in VScode::

    ctrl + `

8. Source the environment::

    source /ros_entrypoint.sh

9. Install the Franka ROS 2 packages::

    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.sh

MoveIt
------

To see if everything works, you can try to run the MoveIt example on the robot::

    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<fci-ip>

Then activate the ``MotionPlanning`` display in RViz.

If you do not have a robot you can still test your setup by running on a dummy hardware::

    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true


Wait until you can see the green ``You can start planning now!`` message from MoveIt inside the
terminal. Then turn off the ``PlanningScene`` and turn it on again. After that turn on the ``MotionPlanning``.


Example Controllers
-------------------

This repo comes with a few example controllers located in the ``franka_example_controllers`` package.

The following launch files are executed with the gripper by default. If you
do not have the gripper attached you can disable the gripper in the launch file with ``load_gripper:=false``.

Move-to-start
^^^^^^^^^^^^^

This controller moves the robot to its home configuration.

.. code-block:: shell

    ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=<fci-ip>

.. _gravity_example:

Gravity Compensation
^^^^^^^^^^^^^^^^^^^^

This is the simplest controller that we have and is a good starting point to write your own.
It sends zero as torque command to all joints, which means that the robot only compensates its own weight.

.. code-block:: shell

    ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=<fci-ip>

Joint Impedance Example
^^^^^^^^^^^^^^^^^^^^^^^

The example moves joints 4 and 5 in a periodic movement that is very compliant. You can try to move the
joints while it is running.

.. code-block:: shell

    ros2 launch franka_bringup joint_impedance_example_controller.launch.py robot_ip:=<fci-ip>


Model Example Controller
^^^^^^^^^^^^^^^^^^^^^^^^
This is a read-only controller which prints the coriolis force vector, gravity force vector, pose matrix of Joint4,
Joint4 body jacobian and end-effector jacobian with respect to the base frame.

.. code-block:: shell

    ros2 launch franka_bringup model_example_controller.launch.py robot_ip:=<fci-ip>


Package Descriptions
--------------------

This section contains more detailed descriptions of what each package does. In general the package structure tries to
adhere to the structure that is proposed
`here <https://stoglrobotics.github.io/ros_team_workspace/master/guidelines/robot_package_structure.html>`_.


franka_bringup
^^^^^^^^^^^^^^

This package contains the launch files for the examples as well as the basic ``franka.launch.py`` launch file, that
can be used to start the robot without any controllers.

When you start the robot with::

    ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip> use_rviz:=true

There is no controller running apart from the ``joint_state_broadcaster``. However, a connection with the robot is still
established and the current robot pose is visualized in RViz. In this mode the robot can be guided when the user stop
button is pressed. However, once a controller that uses the ``effort_command_interface`` is started, the robot will be
using the torque interface from libfranka. For example it is possible to launch the
``gravity_compensation_example_controller`` by running::

    ros2 control load_controller --set-state active  gravity_compensation_example_controller

This is the equivalent of running the ``gravity_compensation_example_controller.launch.py`` launch file mentioned in
:ref:`Gravity Compensation <gravity_example>`.

When the controller is stopped with::

    ros2 control set_controller_state gravity_compensation_example_controller inactive

the robot will stop the torque control and will only send its current state over the FCI.

You can now choose to start the same controller again with::

    ros2 control set_controller_state gravity_compensation_example_controller active

or load and start a different one::

    ros2 control load_controller --set-state active joint_impedance_example_controller


.. _franka_description:

franka_description
^^^^^^^^^^^^^^^^^^

This package contains the xacro files and meshes that are used to visualize the robot.
Further, it contains a launch file that visualizes the robot model without access to a real robot::

    ros2 launch franka_description visualize_franka.launch.py load_gripper:=<true|false>


.. _franka_example_controllers:

franka_example_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^

This package contains a few controllers that can be seen as example of how to write controllers in ROS 2. Currently,
a controller only has access to measured joint positions and joint velocities. Based on this information the controller
can send torque commands. It is currently not possible to use other interfaces like the joint position interface.

franka_gripper
^^^^^^^^^^^^^^

This package contains the ``franka_gripper_node`` for interfacing with the ``Franka Hand``.

The ``franka_gripper_node`` provides the following actions:

* ``homing`` - homes the gripper and updates the maximum width given the mounted fingers.
* ``move`` - moves to a target width with the defined speed.
* ``grasp`` - tries to grasp at the desired width with the desired force while closing with the given speed. The operation is successful if the
  distance ``d`` between the gripper fingers is ``width - epsilon.inner < d < width + epsilon.outer``
* ``gripper_action`` - a special grasping action for MoveIt.

Also, there is a ``stop`` service that aborts gripper actions and stops grasping.


Use the following launch file to start the gripper::

    ros2 launch franka_gripper gripper.launch.py robot_ip:=<fci-ip>


In a different tab you can now perform the homing and send a grasp command.::


    ros2 action send_goal /panda_gripper/homing franka_msgs/action/Homing {}
    ros2 action send_goal -f /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50}"

The inner and outer epsilon are 0.005 meter per default. You can also explicitly set the epsilon::

    ros2 action send_goal -f /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50, epsilon: {inner: 0.01, outer: 0.01}}"

To stop the grasping, you can use ``stop`` service.::

    ros2 service call /panda_gripper/stop std_srvs/srv/Trigger {}

.. _franka_hardware:

franka_hardware
^^^^^^^^^^^^^^^

This package contains the ``franka_hardware`` plugin needed for `ros2_control <https://control.ros.org/humble/index.html>`_.
The plugin is loaded from the URDF of the robot and passed to the controller manger via the robot description.
It provides for each joint:

* a ``position state interface`` that contains the measured joint position.
* a ``velocity state interface`` that contains the measured joint velocity.
* an ``effort state interface`` that contains the measured link-side joint torques including gravity.
* an ``effort command interface`` that contains the desired joint torques without gravity.

In addition

* a ``franka_robot_state`` that contains the robot state information, `franka_robot_state <https://t.ly/0GfIM>`_.
* a ``franka_robot_model_interface`` that contains the pointer to the model object.

.. important::
    ``franka_robot_state`` and ``franka_robot_model_interface`` state interfaces should not be used directly from hardware state interface.
    Rather, they should be utilized by the :ref:`franka_semantic_components` interface.

The IP of the robot is read over a parameter from the URDF.

.. hint::
    Joint Position and Velocity controllers will be soon available

.. _franka_semantic_components:

franka_semantic_components
^^^^^^^^^^^^^^^^^^^^^^^^^^

This package contains franka_robot_model and franka_robot_state classes.
These classes are used to convert franka_robot_model object and franka_robot_state objects,
which are stored in the hardware_state_interface as a double pointer.

For further reference on how to use these classes:
`Franka Robot State Broadcaster <https://github.com/frankaemika/franka_ros2/tree/humble/franka_robot_state_broadcaster>`_
and
`Franka Example Controllers(model_example_controller)
<https://github.com/frankaemika/franka_ros2/blob/humble/franka_example_controllers/src/model_example_controller.cpp>`_

.. _franka_robot_state_broadcaster:

franka_robot_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This package contains read-only franka_robot_state_broadcaster controller.
It publishes franka_robot_state topic to the topic named `/franka_robot_state_broadcaster/robot_state`.
This controller node is spawned by franka_launch.py in the franka_bringup.
Therefore, all the examples that include the franka_launch.py publishes the robot_state topic.

.. _franka_moveit_config:

franka_moveit_config
^^^^^^^^^^^^^^^^^^^^

This package contains the configuration for MoveIt2. There is a new move group called
``panda_manipulator`` that has its tip between the fingers of the gripper and has its Z-axis rotated by -45 degrees, so
that the X-axis is now facing forward, making it easier to use. The ``panda_arm`` move group is still available
for backward compatibility. New applications should use the new ``panda_manipulator`` move group instead.

.. figure:: _static/move-groups.png
    :align: center
    :figclass: align-center

    Visualization of the old and the new move group

franka_msgs
^^^^^^^^^^^

This package contains the definitions for the different gripper actions and robot state message.


joint_effort_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This package contains a modified joint_trajectory_controller that can use the effort interface of the
``franka_hardware::FrankaHardwareInterface``. It is based on this
`Pull request <https://github.com/ros-controls/ros2_controllers/pull/225>`_.

.. note::
    This package will be soon deleted as the fix is available in
    `ros2_controllers <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_ master branch.
    As soon as, it's backported to Humble, it will be deleted from franka_ros2 repository.

Differences between franka_ros and franka_ros2
----------------------------------------------

This section gives an overview of the fundamental changes between ``franka_ros`` and ``franka_ros2``.

franka_gripper
^^^^^^^^^^^^^^

* All topics and actions were previously prefixed with ``franka_gripper``. This prefix was renamed to ``panda_gripper``
  to enable, in the future, a workflow where all prefixes are based on the ``arm_id``
  to effortlessly enable multi arm setups.

* The ``stop`` action is now a service action as it is not preemptable.

* All actions (apart from the ``gripper_action``) have the current gripper width as feedback.

franka_gazebo
^^^^^^^^^^^^^

Currently, we do not offer a Gazebo integration with ``franka_ros2``. However, we provide one
with :doc:`franka_ros`.

franka_visualization
^^^^^^^^^^^^^^^^^^^^

This package does not exist anymore. However, :ref:`franka_description` provides a launch file to visualize the robot
model without a connection to a robot.

franka_control
^^^^^^^^^^^^^^

This package does not exist anymore. The connection to the robot is provided by the hardware plugin in
the :ref:`franka_hardware` package. The actions and services that it provided are currently
not offered in ``franka_ros2``.


Writing Controllers
^^^^^^^^^^^^^^^^^^^

Compared to ``franka_ros`` we currently offer a reduced set of controller interfaces:

* Joint positions
* Joint velocities
* Measured torques
* Franka robot state
* Franka robot model

.. important::
    Franka robot state is published through :ref:`franka_robot_state_broadcaster`
    package to the topic named  `/franka_robot_state_broadcaster/robot_state`

.. important::
    Both Franka robot state and Franka robot model are advised to use through :ref:`franka_semantic_components` class.
    They are stored in the state_interface as double pointers and casted back to their original objects inside the franka_semantic_component class.

    Example of using franka_model can be found in the franka_example_controllers package:
    `model_example_controller <https://github.com/frankaemika/franka_ros2/blob/humble/franka_example_controllers/src/model_example_controller.cpp>`_.


You can base your own controller on one of the :ref:`franka_example_controllers`. To compute kinematic
and dynamic quantities of the robot you can use the joint states and the URDF of the robot in libraries like
`KDL <https://www.orocos.org/kdl/user-manual>`_ (of which there is also a ROS 2 package available).

Non-realtime robot parameter setting
------------------------------------

Non-realtime robot parameter setting can be done via ROS 2 services. They are advertised after the robot hardware is initialized.

Service names are given below::

 * /service_server/set_cartesian_stiffness
 * /service_server/set_force_torque_collision_behavior
 * /service_server/set_full_collision_behavior
 * /service_server/set_joint_stiffness
 * /service_server/set_load
 * /service_server/set_parameters
 * /service_server/set_parameters_atomically
 * /service_server/set_stiffness_frame
 * /service_server/set_tcp_frame

Service message descriptions are given below.

 * ``franka_msgs::srv::SetJointStiffness`` specifies joint stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetCartesianStiffness`` specifies Cartesian stiffness for the internal
   controller (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetTCPFrame`` specifies the transformation from <arm_id>_EE (end effector) to
   <arm_id>_NE (nominal end effector) frame. The transformation from flange to end effector frame
   is split into two transformations: <arm_id>_EE to <arm_id>_NE frame and <arm_id>_NE to
   <arm_id>_link8 frame. The transformation from <arm_id>_NE to <arm_id>_link8 frame can only be
   set through the administrator's interface.
 * ``franka_msgs::srv::SetStiffnessFrame`` specifies the transformation from <arm_id>_K to <arm_id>_EE frame.
 * ``franka_msgs::srv::SetForceTorqueCollisionBehavior`` sets thresholds for external Cartesian
   wrenches to configure the collision reflex.
 * ``franka_msgs::srv::SetFullCollisionBehavior`` sets thresholds for external forces on Cartesian
   and joint level to configure the collision reflex.
 * ``franka_msgs::srv::SetLoad`` sets an external load to compensate (e.g. of a grasped object).

Launch franka_bringup/franka.launch.py file to initialize robot hardware::

    ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>

Here is a minimal example:

.. code-block:: shell

    ros2 service call /service_server/set_joint_stif
    fness franka_msgs/srv/SetJointStiffness "{joint_stiffness: [1000.0, 1000.0, 10
    00.0, 1000.0, 1000.0, 1000.0, 1000.0]}"

.. important::

    Non-realtime parameter setting can only be done when the robot hardware is in `idle` mode.
    If a controller is active and claims command interface this will put the robot in the `move` mode.
    In `move` mode non-realtime param setting is not possible.

.. important::

    The <arm_id>_EE frame denotes the part of the
    configurable end effector frame which can be adjusted during run time through `franka_ros`. The
    <arm_id>_K frame marks the center of the internal
    Cartesian impedance. It also serves as a reference frame for external wrenches. *Neither the
    <arm_id>_EE nor the <arm_id>_K are contained in the URDF as they can be changed at run time*.
    By default, <arm_id> is set to "panda".

    .. figure:: _static/frames.svg
        :align: center
        :figclass: align-center

        Overview of the end-effector frames.



Known Issues
------------

* When using the ``fake_hardware`` with MoveIt, it takes some time until the default position is applied.
