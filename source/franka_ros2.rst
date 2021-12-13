franka_ros2
===========
.. note::

 ``franka_ros2`` is not supported on Windows.

The `franka_ros2 repo <https://github.com/frankaemika/franka_ros2>`_ contains a ROS 2 integration of
:doc:`libfranka <libfranka>`.

.. caution::
    franka_ros2 is currently beta software, so be careful while using it and report bugs on
    `GitHub <https://github.com/frankaemika/franka_ros2/issues>`_.


Prerequisites
-------------

* A `ROS 2 Foxy installation <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>`_
  (ros-foxy-desktop)
* A :ref:`PREEMPT_RT kernel <preempt>` (optional, but strongly recommended).
* A system-wide :ref:`libfranka installation <build-libfranka>`. Here is a minimal example:

.. code-block:: shell

   sudo apt install -y libpoco-dev libeigen3-dev
   git clone https://github.com/frankaemika/libfranka.git --recursive
   cd libfranka
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
------------

1. Install requirements::

    sudo apt install -y \
    ros-foxy-control-msgs \
    ros-foxy-xacro \
    ros-foxy-angles \
    ros-foxy-ros2-control \
    ros-foxy-realtime-tools \
    ros-foxy-control-toolbox \
    ros-foxy-moveit \
    ros-foxy-ros2-controllers \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-ament-cmake-clang-format \
    python3-colcon-common-extensions

2. Create a ROS 2 workspace::

    mkdir -p ~/franka_ros2_ws/src

3. Clone repo and build packages::

    source /opt/ros/foxy/setup.bash
    cd ~/franka_ros2_ws
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
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

This is the simplest controller that we have and is a good starting point to write
your own. It sends zero as torque command to all joints, which means that the robot only compensates
its own weight.

.. code-block:: shell

    ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=<fci-ip>


Joint Impedance Example
^^^^^^^^^^^^^^^^^^^^^^^

The example moves joints 4 and 5 in a periodic movement that is very compliant. You can try to move the
joints while it is running.

.. code-block:: shell

    ros2 launch franka_bringup joint_impedance_example_controller.launch.py robot_ip:=<fci-ip>

Package Descriptions
--------------------

This section contains more detailed descriptions of what each package does. In general the package structure tries to
adhere to the structure that is proposed
`here <https://stoglrobotics.github.io/ros_team_workspace/guidelines/robot_package_structure.html>`_.


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

    ros2 control load_controller --set-state start  gravity_compensation_example_controller

This is the equivalent of running the ``gravity_compensation_example_controller.launch.py`` launch file mentioned in
:ref:`Gravity Compensation <gravity_example>`.

When the controller is stopped with::

    ros2 control set_controller_state gravity_compensation_example_controller stop

the robot will stop the torque control and will only send its current state over the FCI.

You can now choose to start the same controller again with::

    ros2 control set_controller_state gravity_compensation_example_controller start

or load and start a different one::

    ros2 control load_controller --set-state start joint_impedance_example_controller

.. note::

    When the robot stops due to an error, the ``ros2_control_node`` dies. This will shutdown all
    other nodes as well. To recover from this, you have to toggle the user stop button before you can restart the
    launch file.

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

.. important::
    In contrast to franka_ros, it is currently not possible to directly access properties like the mass matrix,
    coriolis torques or jacobians.

franka_gripper
^^^^^^^^^^^^^^

This package contains the ``franka_gripper_node`` for interfacing with the ``Franka Hand``.

The ``franka_gripper_node`` provides the following actions:

* ``homing`` - homes the gripper and updates the maximum width given the mounted fingers.
* ``move`` - moves to a target width with the defined speed.
* ``grasp`` - tries to grasp at the desired width with the desired force while closing with the given speed.

The operation is successful if the distance ``d`` between the gripper fingers is
``width - epsilon.inner < d < width + epsilon.outer``
* ``gripper_action`` - a special grasping action for MoveIt.

Also, there is a ``stop`` service that aborts gripper actions and stops grasping.


Use the following launch file to start the gripper::

    ros2 launch franka_gripper gripper.launch.py robot_ip:=<fci-ip>


In a different tab you can now perform the homing and send a grasp command.

.. code-block:: shell

    ros2 action send_goal /panda_gripper/homing franka_msgs/action/Homing {}
    ros2 action send_goal -f /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50}"

The inner and outer epsilon are 0.005 meter per default. You can also explicitly set the epsilon::

    ros2 action send_goal -f /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50, epsilon: {inner: 0.01, outer: 0.01}}"

To stop the grasping, you can use ``stop`` service.

.. code-block:: shell

    ros2 service call /panda_gripper/stop std_srvs/srv/Trigger {}

.. _franka_hardware:

franka_hardware
^^^^^^^^^^^^^^^

This package contains the ``franka_hardware`` plugin needed for `ros2_control <https://control.ros.org/index.html>`_.
The plugin is loaded from the URDF of the robot and passed to the controller manger via the robot description.
It provides for each joint:

* a ``position state interface`` that contains the measured joint position.
* a ``velocity state interface`` that contains the measured joint velocity.
* an ``effort state interface`` that contains the measured link-side joint torques including gravity.
* an ``effort command interface`` that contains the desired joint torques without gravity.

The IP of the robot is read over a parameter from the URDF.

franka_moveit_config
^^^^^^^^^^^^^^^^^^^^

This package contains the configuration for MoveIt2. There is a new move group called
``panda_manipulator`` that has its tip between the fingers of the gripper and has its Z-axis rotated by 45 degrees, so
that the X-axis is now facing forward, making it easier to use. The ``panda_arm`` move group is still available
for backward compatibility. New applications should use the new ``panda_manipulator`` move group instead.

.. figure:: _static/move-groups.png
    :align: center
    :figclass: align-center

    Visualization of the old and the new move group

franka_msgs
^^^^^^^^^^^

This package contains the definitions for the different gripper actions.

.. important::
    In contrast to franka_ros, there is no longer a FrankaState message, as there is currently no way to communicate it
    from the hardware class.

joint_effort_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This package contains a modified joint_trajectory_controller that can use the effort interface of the
``franka_hardware::FrankaHardwareInterface``. It is based on this
`Pull request <https://github.com/ros-controls/ros2_controllers/pull/225>`_ and backported to Foxy. It offers
a ``FollowJointTrajectory`` action that is needed for MoveIt.


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

The reason is that the hardware interface currently only supports double data types,
making it impossible to expose e.g. a ``franka::RobotState``.

You can base your own controller on one of the :ref:`franka_example_controllers`. To compute kinematic
and dynamic quantities of the robot you can use the joint states and the URDF of the robot in libraries like
`KDL <https://www.orocos.org/kdl/user-manual>`_ (of which there is also a ROS 2 package available).


Known Issues
------------

* When using the ``fake_hardware`` with MoveIt, it takes some time until the default position is applied.
