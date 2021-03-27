BOARD_TAG = mega
BOARD_SUB = atmega2560
ARDUINO_SKETCHBOOK = /home/$(USER)/EZ-RASSOR_Arduino
ARDUINO_DIR = /opt/arduino-1.8.13
ARDUINO_LIBS = Adafruit_Motor_Shield_V2_Library Wire ros_lib AccelStepper Wire

include /home/$(USER)/git/Arduino-Makefile/Arduino.mk
