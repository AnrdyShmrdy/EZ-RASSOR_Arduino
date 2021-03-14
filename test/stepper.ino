/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/String.h>

// Create the motor shield object with the default I2C address
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(); 
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *myMotor1 = AFMS1.getStepper(200, 1);
Adafruit_StepperMotor *myMotor2 = AFMS2.getStepper(200, 1);

ros::NodeHandle nh;

std_msgs::String message;
ros::Publisher chatter("chatter", &message);

char forward[18] = "Spinning Forward!";
char backward[19] = "Spinning Backward!";

void setup() {
	Serial.begin(9600); // set up Serial library at 9600 bps
	Serial.println("Stepper test!");

	nh.initNode();
	nh.advertise(chatter);

	AFMS1.begin();  // create with the default frequency 1.6KHz
	AFMS2.begin();
			  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
			    
	myMotor1->setSpeed(10);  // 10 rpm
	myMotor2->setSpeed(10);
					
}

void loop() {
	int maxLoops = 0, loop = 0;
	Serial.println("Single coil steps");
	int fullrev = 200;

	message.data = forward;
	chatter.publish(&message);
	nh.spinOnce();
	delay(1000);

	myMotor1->step(fullrev, FORWARD, SINGLE);
	delay(1000);

	message.data = backward;
	chatter.publish(&message);
	nh.spinOnce();
	delay(1000);

	myMotor1->step(fullrev, BACKWARD, SINGLE);
	delay(1000);
}

