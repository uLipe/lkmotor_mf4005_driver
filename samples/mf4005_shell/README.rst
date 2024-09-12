.. mf4005_shell:

MF4005 Dual Motor Shell over CAN sample
#######################################

Overview
********

A simple sample that can be used with any board that supports CAN
bus, it checks the connection of up two motors, and exposes the
motor control api through the shell, the sample is initially based on
the Arduino GIGA R1 board, and it uses the FDCAN2 wired to a transceiver,
which access a CAN network composed by two motors

Building and Running
********************

This application can be built as follows for Arduino GIGA R1 board:

.. code-block:: console

    $ west build -pauto -barduino_giga_r1/stm32h747xx/m7 mf4005_shell

Hit the rst button two times in a row to put the board in bootloader mode,
then you can flash the application using `west` :

.. code-block:: console

    $ west flash


Expected Output
===============

Using your favorite terminal tool, connect the console of your board to the host computer,
assuming Arduino GIGA R1 you might able to connect using the USB. after opening the terminal
you should see:

.. code-block:: console

    Running MF4005 Shell sample at arduino_giga_r1/stm32h747xx/m7
    [00:00:00.000,000] <inf> mf4005: Motor: (0x141) connection checked, and its ready to use!
    [00:00:00.001,000] <inf> mf4005: Motor: (0x142) connection checked, and its ready to use!
    *** Booting Zephyr OS build v3.7.0-2699-gf05b0cd4e082 ***
    [00:00:00.017,000] <inf> usb_cdc_acm: Device suspended
    [00:00:00.361,000] <inf> usb_cdc_acm: Device configured

type `lkmotor_mf4005_driver` + ENTER, then you should see the menu:

.. code-block:: console

    uart:~$ lkm_mf4005 
    lkm_mf4005 - LK MF4005 Brushless motor commands
    Subcommands:
    motor_start        : Turn on the power stage of the motor required to perform
                        motionSyntax:
                        <device_name>
    motor_stop         : Turns off the power stage of the motor and stops all
                        motion Syntax:
                        <device_name>
    motor_attr_set     : Set one of the motor attribute. Syntax:
                        <device_name> <attribute_name> <value> 
    motor_channel_set  : Set the motors's channel value. Syntax:
                        <device_name> <channel_name> <value>
    motor_channel_get  : Prints motor channel value. Syntax:
                        <device_name> <channel_name>

The shell support the auto-complete feature, you can do some play with the two motors:

.. code-block:: console

    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg -45000
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg 0
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg 1000
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg 0
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg 90
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg 90000
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 position_milideg 0
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 speed_deg_seconds 150
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@142 speed_deg_seconds -150
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@142 speed_deg_seconds 150
    uart:~$ lkm_mf4005 motor_channel_set mf4005_motor@141 speed_deg_seconds 
