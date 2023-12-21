Franka Matlab
=============

.. todolist::

.. important::
    Franka Matlab artifacts can be obtained at `Franka World <https://franka.world/>`_.

.. important::
    Franka Matlab is based on the `Franka Control Interface (FCI) <https://frankaemika.github.io/docs/>`_ and
    specifically the `libfranka C++ interface <https://frankaemika.github.io/docs/libfranka.html>`_,
    therefore all the same
    `system and network requirements <https://frankaemika.github.io/docs/requirements.html>`_  apply.

.. hint::
    The design of the current libraries and tools in the package take as a given that the **build & execution will be performed in the same machine that is running the current Matlab session**. Therefore when we use the term
    **"deployment", the local PC is targeted**. Manual deployment to other target PCs is of course possible.

Franka Matlab contains the following main components for interfacing the Franka Robotics Robot through Matlab & Simulink:

* ``Simulink Franka Library``, a set of simulink blocks for interfacing the Franka Robotics Robot, for rapid-prototyping and evaluation of control algorithms.

.. figure:: _static/simulink_library_browser.png
    :align: center
    :figclass: align-center

    Simulink Library for rapid-prototyping of controllers for the Franka Robotics robot.

* ``Matlab Franka Library``, a set of helper matlab functions for reading current the robot state and applying some simple open loop motion commands.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   requirements
   compatibility
   installation
   getting_started
   simulink_library
   matlab_library
   franka_matlab_changelog
   troubleshooting
