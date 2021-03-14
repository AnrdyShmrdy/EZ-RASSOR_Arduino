
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

void setup() {
  // Open the serial connection on the arduino
  Serial.begin(9600);           
  Serial.println("Stepper test!");

  // Initialize the motors
  AFMS.begin();
  myMotor->setSpeed(10);

  // Wait until the serial connection starts before continuing
  while( Serial.available() < 0){
    ;
  }
  Serial.println("Beginning test");
}

void loop() {
  Serial.println("testing inside loop");
  if(Serial.available() > 0){

    int input = Serial.read();

    if(input == 119){
      // Move forward 100 single steps. Each step is 1.8 degrees so it moves 180 degrees in total
      myMotor->step(100, FORWARD, SINGLE); 
    }
  }
}
