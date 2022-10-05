Troubleshooting
===============

.. hint::
    Checkout the `Franka Community <https://www.franka-community.de>`_  and the
    `franka_matlab category <https://www.franka-community.de/c/franka-matlab/15>`_ for relevant posts or for creating new ones!

libfranka reference
-------------------
.. hint::
    Same error messages and advised troubleshooting as `libfranka <https://frankaemika.github.io/docs/troubleshooting.html>`_.

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

Issues with libstdc++.so
------------------------

It could be that Matlab throws an error related to libstdc++.so.6 after calling one of the matlab scripts of the franka-matlab library in a
Linux environment. If that's the case our current working solution involves renaming the precompiled libstdc++ library in the Matlab installation,
which forces Matlab to look in the system for the proper dynamic standard library.

This can be performed with e.g:

.. code-block:: shell

    $ mv matlabroot/sys/os/glnx64/libstdc++.so.6 matlabroot/sys/os/glnx64/libstdc++.so.6.off

Restarting Matlab is then recommended.

Franka Simulink library number of block instances
-------------------------------------------------

.. important::
    Multiple instances for the Apply Control block are NOT allowed** in the same system (multiple instances can be
    inserted in corresponding enabled subsystems). One-and-only-one Apply Control block is necessary for all the other
    blocks to work. Multiple instances of the rest of the Franka Simulink library blocks ARE allowed.