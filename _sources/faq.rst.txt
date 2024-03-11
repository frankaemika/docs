FAQ
===

How can I use libfranka in my own CMake-based project?
------------------------------------------------------

Add the following to your ``CMakeLists.txt``:

.. code-block:: cmake

    find_package(Franka REQUIRED)

    # ...

    target_link_libraries(<target-name> Franka::Franka)


To build against a custom ``libfranka`` location, compile
your project with::

    # Build against libfranka build directory
    cmake -DFranka_DIR=/path/to/libfranka/build

    # Build against custom libfranka installation
    cmake -DFranka_DIR=/path/to/usr/lib/cmake/Franka

.. important::

    Don't forget to build your project in release mode
    (``cmake -DCMAKE_BUILD_TYPE=Release``)!
