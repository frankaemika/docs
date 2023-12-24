Troubleshooting
===============

.. hint::
    Checkout the `Franka Community <https://www.franka-community.de>`_  and the 
    `franka_matlab category <https://www.franka-community.de/c/franka-matlab/15>`_ for relevant posts or for creating new ones! 

libfranka reference
-------------------
.. hint::
    Same error messages and advised troubleshooting applies as `libfranka <https://frankaemika.github.io/docs/troubleshooting.html>`_.

Issues with the graphics driver in Linux
----------------------------------------

NVIDIA's graphics driver's are not officially supported in Linux with Real-Time Kernel. This could cause issues in graphics renderings in Matlab 
and Simulink, e.g with figures and scopes respectively. We would then recommend starting matlab with the `-softwareopengl` for avoiding these issues:

.. code-block:: shell

    $ matlab -softwareopengl

Mexing Simulink & Matlab library
--------------------------------

Simulink & Matlab libraries are delivered with compiled binaries. In case issues arise re-mexing is advised. 

You can mex the libraries by running:

.. code-block:: shell

    >> mex_simulink_library();
    >> mex_matlab_library();

Issues with libstdc++.so and other system dynamic libraries
-----------------------------------------------------------

Make sure that you have installed the `matlab-support package <https://packages.ubuntu.com/search?keywords=matlab-support>`_ for your system, in order for Matlab to reference the system dynamic libraries
instead of the precompiled ones that it ships with:

.. code-block:: shell

    sudo apt install matlab-support

Franka Simulink library number of block instances
-------------------------------------------------

.. important::
    Multiple instances for the Apply Control block are NOT allowed** in the same system (multiple instances can inserted in corresponding enabled 
    subsystems). One-and-only-one Apply Control block is necessary for all the other blocks to work.
    Multiple instances of the rest of the Franka Simulink library blocks ARE allowed.