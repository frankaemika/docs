NVIDIA Jetson Hardware Support
==============================

The Simulink implementations with the Franka MATLAB Toolbox can be readily built and deployed 
for NVIDIA's Jetson Hardware boards, by both Linux and Windows host PCs.

Installation and Requirements
-----------------------------

Please make sure that you are using the `Jetson Linux Real-Time Kernel Package <https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/Kernel/KernelCustomization.html#using-the-jetson-linux-real-time-kernel-package>`_.

Also please make sure that you have downloaded and installed the 
`MATLAB Coder Support Package for NVIDIA Jetson and NVIDIA DRIVE Platforms <https://www.mathworks.com/matlabcentral/fileexchange/68644-matlab-coder-support-package-for-nvidia-jetson-and-nvidia-drive-platforms>`_.

Lastly, make sure you have successfully installed the Franka MATLAB Toolbox.

Getting started
---------------

Start by clicking "Run on Custom Hardware" in the Simulink APPS pane.

.. figure:: _static/run_on_custom_hardware.png
    :align: center
    :figclass: align-center

    "Run on Custom Hardware"

Continue by clicking on the "Hardware Settings"

.. figure:: _static/hardware_settings.png
    :align: center
    :figclass: align-center

    "Hardware Settings"

Select the "NVIDIA Jetson" Hardware board

.. figure:: _static/select_nvidia_jetson.png
    :align: center
    :figclass: align-center

    "Select NVIDIA Jetson Hardware Board"

Set the `Device Address`, `Username` and `Password` for your NVIDIA Jetson platform.

.. figure:: _static/board_parameters.png
    :align: center
    :figclass: align-center

    "Board Parameters"

.. hint::

    In case the ssh server operates in an alternative port number in the Jetson 
    target hardware platfrom, other than the default `22`, you can manually set 
    it by excecuting the following MATLAB commands:

    .. code-block:: shell

        >> hd = nvidiacoder.internal.BoardParameters('NVIDIA Jetson');
        >> setParam(hb,'port','<new port number>');

It is also highly recommended to run the external mode as a background thread, so that
the real-time 1kHz won't get potentially disrupted when built with "Monitor & Tuning".

.. figure:: _static/external_mode_background_thread.png
    :align: center
    :figclass: align-center

    "Run External Mode as a Background Thread"

You can now build and deploy for your NVIDIA Jetson Platform!

.. warning::

    Before choosing the "Monitor & Tune" for utilzing the External Mode, make sure that 
    the option "Nonreusable function" has been chosen in Model Settings-->Code Generation-->
    Interface.

    .. figure:: _static/model_settings_interface_non_reusable_function.png
        :align: center
        :figclass: align-center
        :scale: 50%

        "Nonreusable function option is required for building with External-Mode"

.. figure:: _static/jetson_deploy.png
    :align: center
    :figclass: align-center

    "Build & Deploy"