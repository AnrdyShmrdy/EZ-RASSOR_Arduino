# Arduino Compile and Upload Instructions
Do this either SSH'd into or directly in Jetson Nano Ubuntu OS 18.04 terminal.
IMPORTANT: If SSH'ing, use flags -X -Y for display forwarding. Otherwise, certain commands will fail.
	ex. ssh -X -Y user@192.168.1.1
You can also do this with the Arduino Mega directly connected to your computer with Ubuntu 18.04.

IMPORTANT: EZRASSOR software needs to be completely installed from: https://github.com/FlaSpaceInst/EZ-RASSOR/

Compiling and uploading Arduino code using terminal. To learn more, see: https://github.com/sudar/Arduino-Makefile

1. sudo apt-get update
2. sudo apt-get install arduino-mk
3. cd root directory "cd /home/$USER"
4. download and copy "RASSOR_Arduino" folder into /home/$USER directory.
    a. NOTE: Google Drive will download folder as .zip file. Do "unzip NAMEOFFOLDER.zip" to unzip folder into home directory.
5. Read comments in ezrassor_twist_stepper.ino file on instructions for connecting motors. Specifically lines 7-14.
	a. NOTE: This is important for which wheels are front, left, back, or right.
6. Make sure Arduino and Motorshields are powered on and connected to Nano/Computer via USB.
6. cd "RASSOR Arduino"
7. make // This compiles ezrassor_twist_stepper.ino
8. make upload // This uploads compiled .ino onto Arduino Mega.
    a. If you receive error "serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied: '/dev/ttyACM0'"
        1. sudo chmod a+rw /dev/ttyACM0
    b. Should say "avrdude done. Thank you."

Arduino should be good to go for EZ-RASSOR software.

- EZ-RASSOR Launch Instructions -

For these steps, you will need 2 terminals minimum. A 3rd terminal can be opened to read rostopic messages.
To do this via SSH, tmux is recommended for terminal multiplexing. To learn how to use tmux, see: https://github.com/tmux/tmux and https://www.youtube.com/watch?v=Lqehvpe_djs. Otherwise you can simply open extra terminal windows manually.

Launch files with proper bash commands will later be implemented to avoid having to open multiple terminals.

IMPORTANT:: Rosserial package needs to be installed for Arduino. Can be installed with: sudo apt-get install ros-melodic-rosserial-arduino

9. On terminal 1, run: roslaunch ezrassor_launcher configurable_communication.launch control_methods:="keyboard"
10. On terminal 2, run: rosrun rosserial_python serial_node.py /dev/ttyACM0
	a. NOTE: /dev/ttyACM0 is default USB bus that Arduino connects to, this can be verified during the "sudo make upload" command.
11. OPTIONAL: On terminal 3, run "rostopic echo /wheel_instructions" to see messages being sent from keyboard.

You're all set! Stepper motors should be turning based on keyboard instructions. See: https://github.com/FlaSpaceInst/EZ-RASSOR/wiki/ezrassor_keyboard_controller
