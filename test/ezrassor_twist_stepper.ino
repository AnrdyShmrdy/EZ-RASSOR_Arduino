#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_MotorShield.h>

// Drop of solder on motorshield determines the address inside Adafruit_Motorshield(). Empty parameter in Adafruit_MotorShield() means default address (0x60). Google "Stacking Motorshields" to learn more about the soldering process.
Adafruit_MotorShield FrontWheelMS = Adafruit_MotorShield(); // Address 0x60 will be used for front wheels.
Adafruit_MotorShield BackWheelMS = Adafruit_MotorShield(0x61); // Address 0x61 will be used for back wheels.

// Once motorshields have been assigned to an object, we can now put each motor from each shield into it's own object.
Adafruit_StepperMotor *fl_wheel = FrontWheelMS.getStepper(200, 1); // Front left wheel | M1-M2
Adafruit_StepperMotor *fr_wheel = FrontWheelMS.getStepper(200, 2); // Front right wheel | M3-M4
Adafruit_StepperMotor *bl_wheel = BackWheelMS.getStepper(200, 1); // Back left wheel | M1-M2
Adafruit_StepperMotor *br_wheel = BackWheelMS.getStepper(200, 2); // Back right wheel | M3-M4


// Node handler for ROS.
ros::NodeHandle nh;

// /wheel_instructions topic handler.
void twistCb( const geometry_msgs::Twist& msg)
{
	// FORWARD and BACKWARD refers to the stepper turning clock-wise or counter clock-wise.
	// Stepper motors are flipped on each side of rover, so left side needs to spin forward while right side spins backwards.
	// spinOnce updates /wheel_instructions topic so that infinite while loop does not occur.

	if (msg.linear.x == 1) // Move forward.
	{
		fl_wheel->onestep(FORWARD, SINGLE);
		bl_wheel->onestep(FORWARD, SINGLE);
		fr_wheel->onestep(BACKWARD, SINGLE);
		br_wheel->onestep(BACKWARD, SINGLE);
	}  
	else if (msg.linear.x == -1) // Move backward.
	{
		fl_wheel->onestep(BACKWARD, SINGLE);
		bl_wheel->onestep(BACKWARD, SINGLE);
		fr_wheel->onestep(FORWARD, SINGLE);
		br_wheel->onestep(FORWARD, SINGLE);
	}  
	else if (msg.angular.z == 1 ) // Turn left.
	{
		fl_wheel->onestep(FORWARD, SINGLE);
		bl_wheel->onestep(FORWARD, SINGLE);
		fr_wheel->onestep(FORWARD, SINGLE);
		br_wheel->onestep(FORWARD, SINGLE);
	}  
	else if (msg.angular.z == -1 ) // Turn right.
	{
		fl_wheel->onestep(BACKWARD, SINGLE);
		bl_wheel->onestep(BACKWARD, SINGLE);
		fr_wheel->onestep(BACKWARD, SINGLE);
		br_wheel->onestep(BACKWARD, SINGLE);
	}
	else
	{
		fl_wheel->release();
		bl_wheel->release();
		fr_wheel->release();
		br_wheel->release();
	}
}

// Subscriber for topics and initializes callbacks (second parameter) for functions.
ros::Subscriber<geometry_msgs::Twist> subTwist("/wheel_instructions", &twistCb);

void setup()
{ 
	// Initializes serial connection to Mega.
	Serial.begin(9600);

	// Initializes node handler and subscribers.
	nh.initNode();
	nh.subscribe(subTwist);

	// Initializes motor shields.
	FrontWheelMS.begin();
	BackWheelMS.begin();

	// Sets speed for each motor in terms of RPM
	fl_wheel->setSpeed(100);
	fr_wheel->setSpeed(100);
	bl_wheel->setSpeed(100);
	br_wheel->setSpeed(100);
}

void loop()
{  
    // Gets messages from ROS topics.
	nh.spinOnce();
}
