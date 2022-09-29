.. _matlab-library:

Matlab Library
==============

Automatic Error Recovery
------------------------

.. code-block:: shell

    >> franka_automatic_error_recovery(<robot ip string>);

Communication Test
------------------

.. code-block:: shell

    >> franka_communication_test(<robot ip string>);

Will return a struct with the communication test results.

Joint Point to Point Motion
---------------------------

.. code-block:: shell

    >> franka_joint_point_to_point_motion(<robot ip string>,<7 element double array with target configuration>, <0 to 1 scalar speed factor>);

Get Joint Poses
---------------

.. code-block:: shell

    >> franka_joint_poses(<robot ip string>);

Will return a 7 element cell array with the current robot joint poses.

Get Robot State
---------------

.. code-block:: shell

    >> franka_robot_state(<robot ip string>);

Will return a struct will the current robot state.