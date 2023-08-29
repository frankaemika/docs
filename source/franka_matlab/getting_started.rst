Getting started
===============

Overview
--------

A set of Simulink & Matlab examples is included within the Franka Matlab Toolbox. Feel free to experiment, adjust 
them and expand them to fit your project needs!

You can navigate through the examples by typing: 

.. code-block:: shell

    >> uiopen(fullfile(franka_matlab_installation_path_get(),'examples','franka_matlab_toolbox_examples.slx'),1);

.. figure:: _static/franka_matlab_toolbox_examples.png
    :align: center
    :figclass: align-center

    Franka Matlab Toolbox Examples Navigator.

Initialization
--------------

After opening, double clicking on any of the simulink models a set of parameters will be loaded automatically in the 
workspace.

.. hint::

    The Simulink models are delivered in R2019a version. They will convert automatically to your Matlab version 
    when you try to save the model. 

.. figure:: _static/workspace_parameters.png
    :align: center
    :figclass: align-center

    Working space after loading a Simulink demo.

The robot_ip is set to 172.16.0.2 by default after loading the demos. Make sure that the robot_ip parameters matches your 
setup, either by modifying it in the `demos/demos_common_config.m` matlab script file or from the cmd line after the 
simulink demo is loaded, like:

.. code-block:: shell

    >> robot_ip = <your robot ip string>

At this point we can start the building & deployment of the Simulink model.

Execution
---------

.. hint::

    The current workflow presented is utilizing the "Run on Custom Hardware" Simulink App, present in Matlab versions
    :math:`\geq` R2020a. In case of Matlab 2019a you can build the model normaly by using the "Build Model" button.
    You then need to run the executable from a terminal as described bellow.

Let's start by selecting the `Run on custom hardware` App from the Apps pane in Simulink.

.. figure:: _static/cartesian_impedance_control_apps.png
    :align: center
    :figclass: align-center

    "Run on custom hardware" Simulink App.

.. note::

    The Franka Simulink Library has been tested with the Generic Real-Time Target (grt.tlc) as well as with the Embedded Coder Target (ert.tlc) in the context of the NVIDIA's Jetson Nano Hardware support package for Simulink.

.. important::

    Before executing make sure that the brakes of the robot are disengaged and that the robot is in execution mode!

You can then select from the Hardware tab either `Monitor & Tune` in case monitoring through the external mode is 
desired or `Build, Deploy & Start` for just executing the application without monitoring.

.. figure:: _static/cartesian_impedance_control_hardware.png
    :align: center
    :figclass: align-center

    Hardware Simulink App.

.. hint::

    As a reminder, in case of a robot with system version 4.2.0 or greater, the FCI control mode needs to be explicitly enabled through Desk --> Sidebar menu --> Activate FCI. 

.. caution::

    The robot will move! Make sure that you are monitoring the situation, ready to take action if necessary!

Alternatively you can run the executable, which is located in the current working space, manually.

In case of Linux:

.. code-block:: shell

    $ ./<simulink_model_name>

or in case of Windows:

.. code-block:: shell

    > <simulink_model_name>.exe

Automatic error recovery
------------------------

.. figure:: _static/simulink_view_diagnostics.png
    :align: center
    :figclass: align-center

    View error and other diagnostic messages in Simulink during the build, deployment and execution phases.

.. figure:: _static/terminal_error_message.png
    :align: center
    :figclass: align-center

    Error message displayed in terminal in case of manual execution.

If the robot encounters an error state and transitions to reflex mode, you may attempt to recover by executing the `franka_automatic_error_recovery` command in Matlab.

.. code-block:: shell

    >> franka_automatic_error_recovery(<robot ip string>);

In case the command fails and the robot remains in the erroneous state try using the guiding mode to manually bring 
back the robot to a valid configuration. 

.. hint::

    Checkout the :ref:`matlab library <matlab-library>` for a set of helper 
    functions that can help to optimize your workflow.
