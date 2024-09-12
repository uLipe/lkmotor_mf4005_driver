.. _lkmotor_mf4005_driver:

LKM MF4005 Brushless Motor Driver
#################################

The MF4005 is a Brushless, Permanent Magnet motor manufactured by
LK Motor company, it offers both torque, velocity and position 
closed loop control, and it can be controlled via CAN network.

This Zephyr driver implements the CAN transport to the MF4005,
it includes the commands like torque, velocity and position,
making it suitable for general motion control applications and
robotics low-level controllers.

You can find more about this motor here: http://en.lkmotor.cn/Product.aspx?TypeID=17

Adding The Driver to your project via ``west``
**********************************************

The recommended way is to use ``west`` to initialize this repository directly and
all its dependencies:

.. code-block:: console

   $ west init -m git@github.com:uLipe/lkmotor_mf4005_driver.git 
   $ west update

Alternatively you can add a local copy of this module by adding the following sections
to ``zephyr/west.yml``:

1. In the ``manifest/remotes`` section add:

.. code-block::

   remotes:
     - name: uLipe
       url-base: https://github.com/uLipe

2. In the ``manifest/projects`` section add:

.. code-block::

   - name: lkmotor_mf4005_driver
     remote: uLipe
     path: modules/lib/lkmotor_mf4005_driver
     revision: main

3. Save the file, and run ``west update`` from the project root to retrieve the
latest version of the library from Github, or whatever ``revision`` was

Playing with your motor:
************************

After setting up zephyr, you can enable the options, `CONFIG_LKMOTOR_MF4005_DRIVER`, and,
`CONFIG_MF4005_DRIVER_SHELL` to enable the driver and the shell of the motor driver, you
can also refer `samples/mf4005_shell` to have something to get started.