Installation on Linux
=====================

This chapter describes how to install ``libfranka`` and ``franka_ros``, either
as binary packages or by building from source, and how to install a real-time
Linux kernel. ``franka_ros`` is only required if you want to control your robot
using `ROS <http://www.ros.org/>`_.

.. note::

   While ``libfranka`` and the ``franka_ros`` packages should work on different Linux distributions,
   official support is currently only provided for:

   * Ubuntu 16.04 LTS `Xenial Xerus` and ROS `Kinetic Kame`
   * Ubuntu 18.04 LTS `Bionic Beaver` and ROS `Melodic Morenia` (requires at least ``libfranka`` 0.6.0)

   The following instructions are exemplary for Ubuntu 16.04 LTS system and ROS `Kinetic Kame`.
   They only work in the supported environments.

Installing from the ROS repositories
------------------------------------

.. hint::

    These packages might not always be up-to-date, as they are only synced at certain intervals.
    Read the changelog at https://frankaemika.github.io to find out which ``libfranka`` version is required for
    a particular robot software version. If this doesn't match the ``ros-kinetic-libfranka`` version from the
    repositories, you need to :ref:`build from source <installation-build-from-source>`.

Binary packages for ``libfranka`` and ``franka_ros`` are available from the ROS repositories.
After `setting up ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`__, execute::

    sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros

.. _installation-build-from-source:

Building from source
--------------------

Before building from source, please uninstall existing installations of ``libfranka`` and
``franka_ros`` to avoid conflicts::

    sudo apt remove "*libfranka*"

Building libfranka
^^^^^^^^^^^^^^^^^^

To build ``libfranka``, install the following dependencies from Ubuntu's package manager::

    sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

Then, download the source code by cloning ``libfranka`` from `GitHub <https://github.com/frankaemika/libfranka>`__:

.. code-block:: shell

    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka

By default, this will check out the newest release of ``libfranka``. If you want to build a particular version of
``libfranka`` instead, check out the corresponding Git tag::

    git checkout <version>
    git submodule update

In the source directory, create a build directory and run CMake:

.. code-block:: shell

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .

Optionally, a ``libfranka`` Debian package can be built using the following command:

.. code-block:: shell

    cpack -G DEB

This creates `libfranka-<version>-<architecture>.deb`.

Building the ROS packages
^^^^^^^^^^^^^^^^^^^^^^^^^

After `setting up ROS Kinetic <https://wiki.ros.org/kinetic/Installation/Ubuntu>`__, create a Catkin
workspace in a directory of your choice:

.. code-block:: shell

    cd /path/to/desired/folder
    mkdir -p catkin_ws/src
    cd catkin_ws
    source /opt/ros/kinetic/setup.sh
    catkin_init_workspace src

Then clone the ``franka_ros`` repository from `GitHub <https://github.com/frankaemika/franka_ros>`__::

    git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

By default, this will check out the newest release of ``franka_ros``. If you want to build a particular version of
``franka_ros`` instead, check out the corresponding Git tag::

    git checkout <version>

Install any missing dependencies and build the packages:

.. code-block:: shell

    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
    source devel/setup.sh

.. warning::
    If you also installed ``ros-kinetic-libfranka``, ``libfranka`` might be picked up from ``/opt/ros/kinetic``
    instead of from your custom ``libfranka`` build!

Setting up the real-time kernel
-------------------------------

In order to control your robot using ``libfranka``, the controller program on
the workstation PC must run with `real-time priority` under a ``PREEMPT_RT``
kernel. This section describes the procedure of patching a kernel to support
``PREEMPT_RT`` and creating an installation package.

.. note::

    NVIDIA binary drivers are not supported on ``PREEMPT_RT`` kernels.

First, install the necessary dependencies::

    apt-get install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex

Then, you have to decide which kernel version to use. To find the one you are
using currently, use ``uname -r``. Real-time patches are only available for
select kernel versions, see
https://www.kernel.org/pub/linux/kernel/projects/rt/. We recommend choosing the
version closest to the one you currently use. The following commands assume the
``4.14.12`` kernel version with the ``4.14.12-rt10`` patch. If you choose a
different version, simply substitute the numbers. Having decided on a version,
use ``curl`` to download the source files::

    curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz
    curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign
    curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
    curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.sign

And decompress them with::

    xz -d linux-4.14.12.tar.xz
    xz -d patch-4.14.12-rt10.patch.xz

Verifying file integrity
^^^^^^^^^^^^^^^^^^^^^^^^

The ``.sign`` files can be used to verify that the downloaded files were not
corrupted or tampered with. The steps shown here are adapted from the
`Linux Kernel Archive <https://www.kernel.org/signature.html>`_ , see the
linked page for more details about the process.

You can use ``gpg2`` to verify the ``.tar`` archives::

    gpg2 --verify linux-4.14.12.tar.sign

If your output is similar to the following::

    $ gpg2 --verify linux-4.14.12.tar.sign
    gpg: assuming signed data in 'linux-4.14.12.tar'
    gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
    gpg: Can't check signature: No public key

You have to first download the public key of the person who signed the above
file. As you can  see from the above output, it has the ID ``6092693E``. You can
obtain it from the key server::

    gpg2  --keyserver hkp://keys.gnupg.net --recv-keys 0x6092693E

Similarly for the patch::

    gpg2 --keyserver hkp://keys.gnupg.net --recv-keys 0x2872E4CC

Note that keys for other kernel version might have different IDs, you will have to
adapt accordingly.

Having downloaded the keys, you can now verify the sources. Here is an example of
a correct output::

    $ gpg2 --verify linux-4.14.12.tar.sign
    gpg: assuming signed data in 'linux-4.14.12.tar'
    gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
    gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" [unknown]
    gpg:                 aka "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
    gpg:                 aka "Greg Kroah-Hartman (Linux kernel stable release signing key) <greg@kroah.com>" [unknown]
    gpg: WARNING: This key is not certified with a trusted signature!
    gpg:          There is no indication that the signature belongs to the owner.
    Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E

See `Linux Kernel Archive <https://www.kernel.org/signature.html>`_
for more information about the warning. To verify the patch, use::

    gpg2 --verify patch-4.14.12-rt10.patch.sign


Compiling the kernel
^^^^^^^^^^^^^^^^^^^^

Once you are sure the files were downloaded properly, you can extract the source
code and apply the patch::

    tar xf linux-4.14.12.tar
    cd linux-4.14.12
    patch -p1 < ../patch-4.14.12-rt10.patch

The next step is to configure your kernel::

    make oldconfig

This opens a text-based configuration menu. When asked for the Preemption Model, choose
the Fully Preemptible Kernel::

    Preemption Model
        1. No Forced Preemption (Server) (PREEMPT_NONE)
        2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
        3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
        4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
        > 5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)

We recommend keeping other options at their default values.
Afterwards, you are ready to compile the kernel. As this is a lengthy process, set the
multithreading option ``-j`` to the number of your CPU cores::

    fakeroot make -j4 deb-pkg

Finally, you are ready to install the newly created package. The exact names
depend on your environment, but you are looking for ``headers`` and ``images``
packages without the ``dbg`` suffix. To install::

    sudo dpkg -i ../linux-headers-4.14.12-rt10_*.deb ../linux-image-4.14.12-rt10_*.deb

Verifying the new kernel
^^^^^^^^^^^^^^^^^^^^^^^^

Restart your system. The Grub boot menu should now allow you to choose your
newly installed kernel. To see which one is currently being used, see the output
of the ``uname -a`` command. It should contain the string ``PREEMPT RT`` and the
version number you chose. Additionally, ``/sys/kernel/realtime`` should exist and
contain the the number ``1``.

.. _installation-real-time:

Allow a user to set real-time permissions for its processes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the ``PREEMPT_RT`` kernel is installed and running, add a group named
`realtime` and add the user controlling your robot to this group::

    sudo addgroup realtime
    sudo usermod -a -G realtime $(whoami)

Afterwards, add the following limits to the `realtime` group in
``/etc/security/limits.conf``::

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

The limits will be applied after you log out and in again.
