EZ-RASSOR for Arduino
---------------------

Compile and flash code for Arduino Mega 2560, Adafruit Motorshield V2, and stepper motors to operate EZ-RASSOR rover.

TODO: Check out the instructions for building the EZ-RASSOR rover. Instructions are WIP.

DEPENDENCIES
------------
- `Ubuntu 18.04`_
- `EZ_RASSOR`_
- `Arduino-Mk`_
- `Arduino IDE`_
- `ROSSerial for Arduino`_ (ROS Melodic)
- `ros_lib` Arduino Library
- `Adafruit MotorShield V2` Arduino Library
- `AccelStepper` Library

Do NOT install Arduino IDE or the Arduino-Mk package using Ubuntu's APT package manager (i.e. "sudo apt install ..."). Doing this mistakenly installs outdated libraries and dependencies that are required for the Arduino firmware.

To install the Arduino IDE, please follow the Ubuntu's `Arduino IDE installation`_ tutorial. Make note of the installation directory.

To install Arduino-Mk, follow the `installation from source`_ instructions from the Arduino-Mk Github repository. Make note of installation directory.

Compile and Upload Instructions
-------------------------------

Connect Arduino Mega 2560 to your computer via serial USB connection.

**Option 1: GUI using Arduino IDE**

Clone or download this repository to directory of your choice.

.. code-block:: bash

   git clone https://github.com/ffermo/EZ-RASSOR_Arduino

Launch the Arduino IDE. Select 'File' then 'Open...'. Locate EZR_Mega2560.ino from the cloned or downloaded respository and click 'Open'. The IDE will ask you to move and create a folder for the .ino file. Select 'Ok'.

To include the necessary libraries, you can either use Arduino IDE's library manager or manually add them from the `libraries`_ folder.

Click the checkmark button to compile.

Click the right arrow button to upload to Mega.

The Arduino Mega is now flashed!

**Option 2: Terminal using Arduino-Mk**

This method is useful and preferred when Arduino is connected to a computer (Jetson Nano or Raspberry Pi) with no display and remote development in necessary (SSH).

.. code-block:: bash

   cd /home/$USER
   git clone https://github.com/ffermo/EZ-RASSOR_Arduino
   cd EZ-RASSOR_Arduino

Before attempting to compile and upload, be sure to review the Makefile in the root directory. This file contains path pointers to Arduino IDE and Arduino-Mk installations, location of this repository, and included libraries.

Compile and upload code to Mega.

.. code-block:: bash

   make
   make upload

If "serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied: '/dev/ttyACM0'", read write permissions needs to be set for Arduino.
   
.. code-block:: bash

   sudo chmod a+rw /dev/ttyACM0

During upload procedure, it should print

.. code-block:: bash

   "avrdude done. Thank you."

The Arduino Mega is now  flashed!

Usage
-----

**Keyboard Control**

The steps below launch the EZ-RASSOR keyboard communication software package to operate the rover.
2 terminals minimum are required for this step. Launch file to streamline this process is currently WIP.

Terminal 1:

.. code-block:: bash

   roslaunch ezrassor_launcher configurable_communication.launch control_methods:="keyboard"

Terminal 2:

.. code-block:: bash

   rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

(Optional) Terminal 3 to view topics being sent to Arduino:

.. code-block:: bash

   rostopic echo /wheel_instructions

You're all set! Stepper motors should be turning based on keyboard instructions. Be sure to check out the `EZ_RASSOR`_ repository to learn more about the capabilities of this project.

.. _`Ubuntu 18.04`: https://releases.ubuntu.com/18.04/
.. _`EZ_RASSOR`: https://github.com/FlaSpaceInst/EZ-RASSOR
.. _`Arduino-Mk`: https://github.com/sudar/Arduino-Makefile
.. _`Arduino IDE`: https://www.arduino.cc/en/software
.. _`ROSSerial for Arduino`: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
.. _`ros_lib`: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
.. _`Arduino MotorShield V2`: https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
.. _`AccelStepper`: https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
.. _`Arduino IDE installation`: https://ubuntu.com/tutorials/install-the-arduino-ide
.. _`installation from source`: https://github.com/sudar/Arduino-Makefile/blob/master/README.md#from-source
.. _`libraries`: https://github.com/ffermo/EZ-RASSOR_Arduino/tree/master/libraries
