ROS getting started
===================

In this section we assume that the ``franka_ros`` repository was cloned and successfully integrated
into a catkin workspace, built and the ``setup.sh`` or ``setup.bash`` of the workspace was sourced.
.. code-block:: shell
   
   source /path/to/catkin_ws/devel/setup.sh    

The repository splits into the following packages:
* ``franka_description``:
    This package contains the description of the franka ARM and a franka HAND in terms of kinematics,
    limits, visible surfaces and a collisioń space. The latter is a simplified version of the more
    detailed visual description to decrease the computational weight of collision detection. The
    descriptions are based on the urdf format (according to http://wiki.ros.org/urdf/XML (TODO link)).
    The files do NOT contain inertial terms, as they are considered confidential, and can therefore
    not be used for dynamics simulations (e.g. gazebo). The package contains to launch files, allowing
    to display the robot descriptions of the arm and the gripper. To visualize e.g. the franka ARM
    with a franka HAND mounted run
    
    .. code-block:: shell
    
        roslaunch franka_description visualize_franka.launch                      # with gripper
        roslaunch franka_description visualize_franka.launch load_gripper:=false  # without gripper
       
* ``franka_example_controllers``:
    This package implements a set of exemplary controllers for use on a franka ARM via the 
    ros_control framework. The controllers depict the variety in interfaces offered by the 
    ``FrankaHW`` class the according usage. Each controller comes with a separate integral launch
    file by the according name that launches everything required to run the controller on the robot
    and visualize it. As arguments to these launchfiles, a robot IP and a bool to load/not load 
    
    
    
