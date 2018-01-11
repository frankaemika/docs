Setting up the network
======================

Good network performance is crucial when controlling the robot using ``FCI``.
Therefore it is strongly recommended to use a direct connection between the
workstation PC and Panda's Control. This chapter describes how to configure your
network for this use case.

.. figure:: _static/control.png
    :align: center
    :figclass: align-center

    Use Control's LAN port when controlling the robot through ``FCI``.
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
-----------------------------

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
---------------------------------

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
:ref:`network-bandwidth-delay-test`.

If the network test was successful, then you are ready to use ``libfranka``! From
now on, you can also access Desk through this address in your browser.
