#include <Arduino.h>

// signal pins
const int signal1 = 2;
const int signal2 = 3;

// motor drivers (2 pins per motor)
const int motorPins[2][2] = {{6, 7}, {8, 9}};

// PWM pin for speed control
const int PWMPin = 5;     

// PID parameters to be altered accordingly
float kp = 0.2;
float ki = 0.0005 ;
float kd = 0.5;

float setpoint = 1;
float input [2] = {0,0};
float output;
float lastInput [2] = {0,0};
float integral = 0;

//setting up all the motor and PWM pins for OUTPUT
void setup() {
	Serial.begin(9600);

	pinMode(PWMPin, OUTPUT); 

	pinMode(signal1, INPUT);
	pinMode(signal2, INPUT);

	// Use the motorPins array to set the pin modes
	for (int i = 0; i < 2; i++) {
		pinMode(motorPins[i][0], OUTPUT);
	    pinMode(motorPins[i][1], OUTPUT);
	}
}

//using output and direction to control motor
void controlMotor(int motor, float output, bool b) {
	//forward direction
	if (b==true) {
	    digitalWrite(motorPins[motor][0], HIGH);
	    digitalWrite(motorPins[motor][1], LOW);
	}
	//backward direction
	else {
	   	digitalWrite(motorPins[motor][0], LOW);
	    digitalWrite(motorPins[motor][1], HIGH);
	}
	//changing speed
	analogWrite(PWMPin, (int)output);
}

void loop() {
	// reading from raspberry pi signals
	input[0] = digitalRead(signal1);
	input[1] = digitalRead(signal2);

	// PID controller calculations
	for (int i = 0; i < 2; i++) {
    	float error = setpoint - input[i];
    	integral += (ki * error);
    	if (integral > 255) integral = 255;
    	else if (integral < -255) integral = -255;

    	float derivative = (input[i] - lastInput[i]);
    	output = kp * error + integral - kd * derivative;

    	if (output > 255) output = 255;
    	else if (output < 0) output = 0;

    	// controlling motor based on calculated output
    	controlMotor(i, output, input[i] > 0.5);
    	lastInput[i] = input[i];
  }
}
