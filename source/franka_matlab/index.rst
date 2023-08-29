Franka Matlab Documentation
===========================

.. todolist::

.. important::
    Franka Matlab is based on the `Franka Control Interface (FCI) <https://frankaemika.github.io/docs/>`_ and 
    the `libfranka C++ interface <https://frankaemika.github.io/docs/libfranka.html>`_. 
    All the same 
    `system and network requirements <https://frankaemika.github.io/docs/requirements.html>`_  do therefore apply.

Franka Matlab contains the following main components for interfacing the Franka Emika Robot through Matlab & Simulink:

* ``Simulink Franka Library``, a set of simulink blocks interfacing the Franka Emika Robot through automatic C++ code gen with Simulink Coder, for rapid-prototyping of robot control algorithms.

.. figure:: _static/simulink_library_browser.png
    :align: center
    :figclass: align-center

    Simulink Library for rapid-prototyping of controllers for the Franka Emika robot.

* ``Matlab Franka Library``, a set of matlab functions for reading current the robot state and applying some simple open loop motion commands.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   matlab_toolbox_dependencies
   compatibility
   installation
   getting_started
   simulink_library
   matlab_library
   franka_matlab_changelog
   troubleshooting