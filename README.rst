Arduino for EZ-RASSOR
---------------------

Compile and flash code for Arduino Mega 2560, Adafruit Motorshield V2, and stepper motors to operate EZ-RASSOR rover.

TODO: Check out the instructions for building the EZ-RASSOR rover. Instructions are WIP.

PREREQUISITES
-------------
- `Ubuntu 18.04`_
- `EZ_RASSOR`_
- `Arduino-Mk`_ (Terminal)
- `Arduino IDE`_ (GUI)
- `ROSSerial for Arduino`_ (ROS Melodic)

Compile and Upload Instructions
-------------------------------

First, connect Arduino Mega 2560 to your computer via serial USB connection.

**Option 1: Terminal**

.. code-block:: bash

   test
   cd /home/$USER
   git clone https://github.com/ffermo/EZ-RASSOR_Arduino
   cd EZ-RASSOR_Arduino
   make
   make upload

If "serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied: '/dev/ttyACM0'", read write permissions needs to be set for Arduino.
   
.. code-block:: bash

   sudo chmod a+rw /dev/ttyACM0

During upload procedure, it should print

.. code-block:: bash

   "avrdude done. Thank you."

Your Arduino is now properly flashed!

**Option 2: Arduino IDE**

Coming soon.

**ROS Launch Instructions**

The steps below launch the EZ-RASSOR keyboard communication to operate the rover.
2 terminals minimum are required for this step. Launch file to streamline this process is currently WIP.

Terminal 1:

.. code-block:: bash

   roslaunch ezrassor_launcher configurable_communication.launch control_methods:="keyboard"

Terminal 2:

.. code-block:: bash

   rosrun rosserial_python serial_node.py /dev/ttyACM0

(Optional) Terminal 3 to view topics being sent to Arduino:

.. code-block:: bash

   rostopic echo /wheel_instructions

You're all set! Stepper motors should be turning based on keyboard instructions. Be sure to check out the `EZ_RASSOR`_ to learn more about the capabilities of this project.

.. _`Ubuntu 18.04`: https://releases.ubuntu.com/18.04/
.. _`EZ_RASSOR`: https://github.com/FlaSpaceInst/EZ-RASSOR
.. _`Arduino-Mk`: https://github.com/sudar/Arduino-Makefile
.. _`Arduino IDE`: https://www.arduino.cc/en/software
.. _`ROSSerial for Arduino`: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
