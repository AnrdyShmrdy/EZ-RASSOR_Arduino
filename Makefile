# For usage instructions, please read README.md
BOARD_TAG = mega
BOARD_SUB = atmega2560

# Path where repository was cloned or downloaded.
ARDUINO_SKETCHBOOK = /home/$(USER)/EZ-RASSOR_Arduino

# Path where Arduino IDE is installed.
ARDUINO_DIR = /opt/arduino-1.8.13

# Library dependencies found in libraries folder.
ARDUINO_LIBS = Adafruit_Motor_Shield_V2_Library ros_lib AccelStepper Wire

# Arduino.mk file from Arduino-Mk installation directory.
include /home/$(USER)/git/Arduino-Makefile/Arduino.mk
