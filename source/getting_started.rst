Getting started
===============

After :doc:`setting up the required software <installation>`, it is time to
connect to the robot, and test the whole setup by using FCI to read the current
robot state.

Operating the robot
-------------------

Before going further though, here are a few safety considerations.
Always check the following things before powering on the robot:

1. Make sure that the Arm has been mounted on a stable base and cannot topple over, even
   when performing fast motions or abrupt stops.

.. caution::
   Only tabletop mounting is supported, i.e. the Arm must be mounted perpendicular to the
   ground! Other mountings will **void your warranty** and **might cause damage
   to the robot**!

2. Ensure that the cable connecting Arm and Control is firmly attached on both sides.
3. Connect the external activation device to Arm's base and keep it next to you in order to be
   able to stop the robot at any time.

.. hint::
   Activating the external activation device will disconnect the Arm from Control.
   The joint motor controllers will then hold their current position.
   **The external activation device is not an emergency stop!**

This list is non-exhaustive! The manual delivered with your robot contains a chapter dedicated
to safety. Please read it carefully and follow the instructions.

.. important::
   The workstation PC which commands your robot using the FCI must always be connected to the LAN
   port of Control (shop floor network) and **not** to the LAN port of the Arm (robot network).

.. _setting-up-the-network:

Setting up the network
----------------------

Good network performance is crucial when controlling the robot using FCI.
Therefore it is strongly recommended to use a direct connection between the
workstation PC and Panda's Control. This section describes how to configure your
network for this use case.

.. figure:: _static/control.png
    :align: center
    :figclass: align-center

    Use Control's LAN port when controlling the robot through FCI.
    Do not connect to the port in Arm's base.

The Control and your workstation must be configured to appear on the same
network. Simplest way to achieve that is to use static IP addresses. Any two
addresses on the same network would work, but the following values will be used
for the purpose of this tutorial:

+---------+----------------+------------+
|         | Workstation PC |  Control   |
+=========+================+============+
| Address | 172.16.0.1     | 172.16.0.2 |
+---------+----------------+------------+
| Netmask | 24             | 24         |
+---------+----------------+------------+

The Control's address (172.16.0.2) is called ``<fci-ip>`` in the following chapters.

The configuration process consists of two steps:

  * Configuring Control's network settings.
  * Configuring your workstation's network settings.

Control network configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For this step, the robot needs to be installed and tested. **Please read through
the documents shipped with your robot and follow the setup instructions before
continuing any further!**

The Control's network can be configured in the administrator's interface. For
the duration of this step you can connect to the robot through the port in the
robot's base. For details, consult the `Connecting a user interface device`
section in the manual delivered with your robot.

.. figure:: _static/accessing-admin.png
    :align: center
    :figclass: align-center

    Accessing the administrator's interface through Desk.

To set up a static address, enter the following values in the `Network` section:

.. figure:: _static/control-static-ip.png
    :align: center
    :figclass: align-center

    Setting a static IP for the Control's LAN port (Shop Floor network).
    DHCP Client option is deselected.

Press `Apply`. After the settings are successfully applied, connect your
workstation's LAN port to the robot's control unit.

Workstation network configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes how to set up a static IP address on Ubuntu 16.04
using the GUI. Follow the official Ubuntu guide_ if you prefer to use the
command line.

.. _guide: https://help.ubuntu.com/lts/serverguide/network-configuration.html

.. caution::
    The following steps will modify your network settings. If in doubt,
    contact your network's administrator.

First, go to Network Connection widget. Select the wired connection you
will be using and click edit.

.. figure:: _static/edit-connections.png
    :align: center
    :figclass: align-center

    Edit the connection in the Ethernet section.

Next, click on the IPv4 settings tab, set the method to Manual, and enter the
following values:

.. figure:: _static/static-ip-ubuntu.png
    :align: center
    :figclass: align-center

    Setting a static IP for the Workstation PC. Method is set to Manual.

.. hint::
   This step will disable DHCP, which means you will no longer obtain an address
   when connecting to a DHCP server, like the one in Arm's base. When you no
   longer use FCI, you can change the Method back to `Automatic (DHCP)`.

Save the changes, and close the Network Connection window. Click on the
connection name from the drop down menu. It should now be possible to connect to
the robot from your workstation. To verify this, perform the
:ref:`network-bandwidth-delay-test`. From now on, you can also access Desk
through this address in your browser.

Verifying the connection
------------------------

The previous section described how to specify the IP address of the Control's
LAN port. In the following sections that address is referred to as ``<fci-ip>``.

In order to verify that everything is correctly set up, run the ``echo_robot_state``
example from ``libfranka``. If you decided to install ``franka_ros`` and ``libfranka`` from the ROS
repository, you can instead read the instructions for
:ref:`visualizing the robot in ros <ros_visualization>` .

Change to the build directory of ``libfranka`` and execute the example:

.. code-block:: shell

    ./examples/echo_robot_state <fci-ip>

The program will print the current state of the robot to the console and terminate after a few
iterations.

Example output:

.. code-block:: json

    {
      "O_T_EE": [0.99507,-0.087825,0.0458691,0,-0.0932286,-0.98667,0.133307,0,0.0335506,-0.136928,
                 -0.990013,0,0.466187,-0.0437226,0.310521,1],
      "O_T_EE_d": [0.7071,0.7071,0,0,0.7071,-0.7071,0,0,0,0,-1,0,0.088,0,0.769,1],
      "F_T_EE": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], "EE_T_K": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
      "m_load": 0, "F_x_Cload": [0,0,0], "I_load": [0,0,0,0,0,0,0,0,0],
      "elbow": [-0.0658817,-1], "elbow_d": [0,1],
      "tau_J": [0.159435,-16.7615,0.0478516,18.8524,0.751037,1.43036,0.0289359],
      "dtau_J": [0,0,0,0,0,0,0],
      "q": [0.0412189,-0.294853,-0.0658817,-2.30709,-0.175077,2.04355,0.931813],
      "dq": [-0.000408694,-0.000135271,8.87298e-05,0.000983035,-0.00110876,-0.00250781,0.00128548],
      "q_d": [0,0,0,0,0,0,0], "dq_d": [0,0,0,0,0,0,0],
      "joint_contact": [0,0,0,0,0,0,0], "cartesian_contact": [0,0,0,0,0,0],
      "joint_collision": [0,0,0,0,0,0,0], "cartesian_collision": [0,0,0,0,0,0],
      "tau_ext_hat_filtered": [0.172327,-0.230889,-0.130005,-0.189423,0.067974,-0.237919,0.0259882],
      "O_F_ext_hat_K": [-1.28182,0.136979,0.436185,0.0400712,-0.651955,0.0273414],
      "K_F_ext_hat_K": [-1.26755,0.0424965,-0.493591,0.106515,0.0430354,-0.00899586],
      "current_errors": [], "last_motion_errors": [], "robot_mode": "Idle", "time": 12444112
    }

.. hint::

    If an error occurs at this point, perform the
    :ref:`ping test <troubleshooting_robot_not_reachable>` and ensure that the robot's fail-safe
    safety locking system is opened. Further information are provided in the manual shipped with
    the robot.
