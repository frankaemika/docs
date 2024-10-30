System Requirements
===================

The same system requirements apply for the target platform as the 
libfranka library. For detailed information about the system requirements, 
please visit the `Franka Emika System Requirements <https://frankaemika.github.io/docs/requirements.html>`_ page.

+------------------------+-------------------+------------------------+-------------------------------+
| Toolbox Version        | libfranka Version | MATLAB Version         | Ubuntu Version (recommended)  |
+========================+===================+========================+===============================+
| 2.0.0                  | 0.9.2 & 0.14.0*   | R2022a or newer        | 22.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 1.0.0                  | 0.9.2 & 0.13.3    | R2021a or newer        | 20.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 0.3.1                  | 0.9.x & 0.10.x    | R2019a or newer        | 20.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 0.3.0                  | 0.9.x & 0.10.x    | R2019a or newer        | 20.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 0.2.1                  | 0.9.x             | R2019a to R2021a       | 18.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 0.2.0                  | 0.8.0             | R2019a to R2021a       | 18.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 0.1.1                  | 0.7.1             | R2019a                 | 18.04                         |
+------------------------+-------------------+------------------------+-------------------------------+
| 0.1.0                  | 0.7.1             | R2019a                 | 18.04                         |
+------------------------+-------------------+------------------------+-------------------------------+

.. note::
   \*Starting with version 2.0.0, the toolbox includes prebuilt libfranka binaries, eliminating the need for manual installation.

For detailed information about robot system requirements, see the `libfranka compatibility documentation <https://frankaemika.github.io/docs/compatibility.html>`_.


MATLAB Toolbox Dependencies
---------------------------

The following Mathworks products are required: 

* `MATLAB <https://www.mathworks.com/products/matlab.html>`_ 
* `Simulink <https://www.mathworks.com/products/simulink.html>`_
* `Simulink Coder <https://www.mathworks.com/products/simulink-coder.html>`_
* `Matlab Coder <https://www.mathworks.com/products/matlab-coder.html>`_

**For working with the Franka AI Companion** please download and install the `MATLAB Coder Support Package for NVIDIA Jetson and NVIDIA DRIVE Platforms <https://www.mathworks.com/matlabcentral/fileexchange/68644-matlab-coder-support-package-for-nvidia-jetson-and-nvidia-drive-platforms>`_.

Some of demos provided with the franka_matlab could potentially need either of the following toolboxes:

* `Stateflow <https://www.mathworks.com/products/stateflow.html>`_ (required for the Simulink Example grasp_object.slx)
* `Matlab Robotics Toolbox <https://www.mathworks.com/products/robotics.html>`_ (required for the MATLAB example pick_and_place_with_RRT.mlx)


