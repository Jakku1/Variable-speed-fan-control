/*
 * main.c
 *
 *	main program for EE30186
 *
 *  Created on: 22 Sep 2015
 *      Author: Alex Beasley
 */

#include "EE30186.h"
#include "system.h"
#include "socal/socal.h"
#include <inttypes.h>
#include <stdio.h>


#define key0 0xE
#define key1 0xD
#define key2 0xB
#define key3 0x7

//define PID Constants
#define Kp 10
#define Ki 0.0000025
#define Kd 70

 //define functions
int encoderSense(volatile int *);
int SevenSegmentDisplay(char);
int pwmGenerator(int, volatile unsigned int *);
int rotaryencoder(int, int, volatile int *, unsigned volatile int *);
int MultiDigitDisplay(int);
int TachoReader(volatile int*, volatile unsigned int *);
int PIDControl(int, int, int);


int main(int argc, char** argv)
{

	volatile int * ExtensionBoard = (volatile int *)ALT_LWFPGA_GPIO_0A_BASE;
	volatile int * Switches = (volatile int *)ALT_LWFPGA_SWITCH_BASE;
	volatile unsigned int * Counter = (volatile unsigned int *)ALT_LWFPGA_COUNTER_BASE;
	volatile int * Hex3to0 = (volatile int *)(ALT_LWFPGA_HEXA_BASE);
	volatile int * Hex5to4 = (volatile int *)(ALT_LWFPGA_HEXB_BASE);
	volatile int * Keys = (volatile int *)(ALT_LWFPGA_KEY_BASE);


	//function call to initialise the FPGA configuration
	EE30186_Start();


	volatile int *ExtensionBoardDriver = ExtensionBoard + 1; //set the driving pin (pin3) as an output
	*ExtensionBoardDriver = 8;
	int sensitivity = 50;
	int chosenSpeed = 500;
	int measuredSpeed = 0;
	int desiredSpeed = 500;
	int PWMpin;
	int controlType = 0;
	int systemState = 0;
	int Key0Count = 0;

	void Delay(int ms)
		{
			// Record time the delay begins
			int Begin = *Counter;
			// hold program until the duration of the delay is over
			while ((Begin + ms*50000) > *Counter);
		}

	while (1)
	{
		if (*Keys == key0) {
					//if key0 is pressed, a counter is incremented
					Key0Count += 1;
					//find if the key has been pressed an odd or even number of times
					systemState = Key0Count % 2;
					//delay to avoid holding the button
					Delay(300);
				}

				if (systemState == 0) {
					//if on key has been pressed an even number of times then it is in the off state
					//speed in off state set to 0
					chosenSpeed = 0;
					//display reads "OFF"
					*Hex3to0 = 0xFF400E0E;
					*Hex5to4 = 0xFFFF;
				}
				else {

						// switch 0 determines the control type
						controlType = 0b1 & *Switches;

					if(controlType == 0){

						//determine if the system is in open or closed loop control
						*Hex5to4 = 0x40C7;

						// find the increment value based on switches selected
						sensitivity = encoderSense(Switches);

						//adjust the speed of the fan using the rotary encoder
						chosenSpeed = rotaryencoder(chosenSpeed, sensitivity, ExtensionBoard, Counter);

						//record the speed of the fan using the built in tachometer
						measuredSpeed = TachoReader(ExtensionBoard, Counter);

						//convert the rpm output to a PWM signal which the fan can interpret
						PWMpin = pwmGenerator(chosenSpeed, Counter);

						//assign PWM signal output to appropriate pin
						*ExtensionBoard = PWMpin << 3;

						//display different values for testing
						if (*Keys == key1) {
							*Hex3to0 = MultiDigitDisplay(measuredSpeed);
						}
						else if (*Keys == key2) {
							*Hex3to0 = MultiDigitDisplay(desiredSpeed);
						}
						else {
							*Hex3to0 = MultiDigitDisplay(chosenSpeed);
						}

						//update the control type if it changes
						controlType = 0b1 & *Switches;;
					}

					//if the system is operating in closed loop control then the PID function is called.
					else if(controlType == 1) {

						//set the display output to signify closed loop
						*Hex5to4 = 0xC6C7;

						//choose appropriate speed
						chosenSpeed = rotaryencoder(chosenSpeed, sensitivity, ExtensionBoard, Counter);

						//measure the speed of the fan through the tachometer
						measuredSpeed = TachoReader(ExtensionBoard, Counter);

						//find closed loop speed control
						desiredSpeed = PIDControl(chosenSpeed, measuredSpeed, desiredSpeed);

						if (*Keys == key1) {
								*Hex3to0 = MultiDigitDisplay(measuredSpeed);
							}
							else if (*Keys == key2) {
								*Hex3to0 = MultiDigitDisplay(desiredSpeed);
							}
							else {
								*Hex3to0 = MultiDigitDisplay(chosenSpeed);
							}
						PWMpin = pwmGenerator(desiredSpeed, Counter);
						//send PWM output to correct pin
						*ExtensionBoard = PWMpin << 3;

						//update control type if it changes
						controlType = 0b1 & *Switches;
					}
				}
	}
	//function call to clean up and close the FPGA configuration
	EE30186_End();

	return 0;

}
/*This function defines the amount a single increment of the rotary encoder will change the fan speed.
 * This is controlled via switches 5 through 9 on the FPGA. When they are high that will be the selected sensitivity*/

int encoderSense(volatile int *Switches) {
	int sensitivity; //contains the how many rpm a unit increase of the rotary encoder corresponds to

	//set the value of a rotary encoder increment depending on the active switches
	switch (*Switches) {
	case 64: //switch 6
		sensitivity = 10;
		break;
	case 128: //switch 7
		sensitivity = 20;
		break;
	case 256: //switch 8
		sensitivity = 50;
		break;
	case 512: // switch 9
		sensitivity = 100;
		break;
	case 1028: //switch 10
		sensitivity = 250;
		break;
	default://default value in case of error
		sensitivity = 75;
		break;
	}

	//return the increment value
	return sensitivity;

}

/*TheGrey code is used to find whether the rotary encoder is rotated and if so which direction*/
int rotaryencoder(int chosenSpeed, int increment, volatile int *ExtensionBoard, volatile unsigned int *Counter)
{
	//initialise the variables within the function
	static int prevCount;
	static int prevA, prevB;
	//Ensure the two pins are read every 20ms
	if (*Counter - prevCount > 1000000)
	{
		//taking the value of the appropriate encoder pins
		int rotaryA = 0x1 & (*ExtensionBoard >> 17);
		int rotaryB = 0x1 & (*ExtensionBoard >> 19);

		//checks for a change in the rotary encoder pin A value
		if ((rotaryA != prevA))
		{
			// Checks to see if it is anti-clockwise movement
			if (!(rotaryA ^ rotaryB))
			{
				//update the chosen speed
				chosenSpeed -= increment;
			}
			// checks for clockwise movement
			if (rotaryA ^ rotaryB)
			{
				//update the chosen speed
				chosenSpeed += increment;
			}
		}

		//checks for a change in the rotary encoder pin A value
		if ((rotaryB != prevB))
		{
			// Checks to see if it is anti-clockwise movement
			if (rotaryA ^ rotaryB)
			{
				//update the chosen speed
				chosenSpeed -= increment;
			}
			// checks for clockwise movement
			if (!(rotaryA ^ rotaryB))
			{
				//update the chosen speed
				chosenSpeed += increment;
			}
			}

		//updating the counter value
		prevCount = *Counter;

		//sets lower limit as the speed of the fan cannot be negative
		if (chosenSpeed < 0)
		{
			chosenSpeed = 0;
		}

		//sets upper limit for fan speed as 2500 is the maximum rpm
		if (chosenSpeed > 2500)
		{
			chosenSpeed = 2500;
		}
		//update the previous pin value ready for the next iteration of the program
		prevA = rotaryA;
		prevB = rotaryB;
	}

	//return chosenSpeed for the next function in the main function
	return chosenSpeed;
}


/*Create a signal which can be used to drive the fan output. This if determined by the percentage of the pulse which is high.
 * If the pulse is all high then it will operate at max speed, if it is always low then it will not spin*/
int pwmGenerator(int chosenSpeed, volatile unsigned int *Counter) {

	//New Counter for signal counter and output value for PWM signal
	int PWMsignal = ((*Counter / 2000) % 2501), PWMpin;

	if (chosenSpeed > PWMsignal) {
		//set pulse to high if chosen speed is more than the current
		PWMpin = 1;
	}
	else {
		//set signal low
		PWMpin = 0;
	}

	return PWMpin;
}

int TachoReader(volatile int* ExtensionBoard, volatile unsigned int *Counter) {

	//During each fan rotation, two signals are produced
	//calculate the time taken between the current rising and falling edge and the previous
	static int edgeDetector = 0, pulseWidth, prevEdge = 0;
	static int edgeDetector2 = 0, pulseWidth2, prevEdge2 = 0;
	static int read4, read3, read2, read1, read0, combRead = 0;
	int measuredSpeed;

	//make all pins have the same initial value so the program can initially run
	read4 = read3;
	read3 = read2;
	read2 = read1;
	read1 = read0;

	//Read tachometer pin form the extension board
	read0= (0x1 & (*ExtensionBoard >> 1));
		//string the 5 values together by shifting and adding them
	combRead = ((read4 << 4) + (read3 << 3) + (read2 << 2) + (read1 << 1) + read0);
		//Three high outputs in a row indicates that the system is in a high state
	if ((combRead & 0b11111) == 0b00111)
	{
		//assign counter value at rising Edge
		edgeDetector = *Counter;
		//find the time taken between the two points
		pulseWidth = (edgeDetector - prevEdge);
		//update previous peak detector value
		prevEdge = edgeDetector;
	}

	//same as above function but for detecting falling edge
	if((combRead & 0b11111) == 0b11000){
		edgeDetector2 = *Counter;
		pulseWidth2 = (edgeDetector2 - prevEdge2);
		prevEdge2 = edgeDetector2;
	}

	//find average time spent at high or low
	int avgPulseWidth = (pulseWidth + pulseWidth2)/2;

	//Same function as above but to detect a falling edge and the system going low


	//take many recordings of values and store them in an array to compare them properly
	int n = 200, i;
	float Readings[201], total;
	for (i = 0; i < n; i++)
	{
		//covert value to rpm for user
		Readings[i] = 1500000000/avgPulseWidth;
		//find sum of all values stored in the array
		total += Readings[i];
	}
	//calculate the average and set it to output
	measuredSpeed = total/201;
	//Pass realSpeed back into the main loop
	return measuredSpeed;
}

/*Closed loop control for the system. Values for P,I,D defined at the beginning of the program*/
int PIDControl(int chosenSpeed, int measuredSpeed, int desiredSpeed){
	//create variables to store the different components of the PID equation
	int error, prevError = 0, integral = 0, differential = 0;

	//find error between the actual speed and the desired speed
	error = measuredSpeed - chosenSpeed;

	//sum of all these values for the area under the curve
	integral += error;

	//find the difference between the current and previous error
	differential = error - prevError;

	//Main loop runs on 200ms time period thus the integral value but be divided by 1000000
	//perform PID calculation
	desiredSpeed = ((Kp * error)/2500 + (Ki * integral)/2500000000 + (Kd * differential)/2500 + chosenSpeed);

	//If value exceeds upper limit, reset ot to the boundary
	if(desiredSpeed>2500){
		desiredSpeed=2500;
	}
	//If value falls below the inimum value, it resets it to the boundary
	else if(desiredSpeed<0){
		desiredSpeed = 0;
	}

	//update error value
	prevError = error;

	return desiredSpeed;
}


int MultiDigitDisplay(int Value) {

	//display a blank display as default
	int displayValue = 0xFFFFFFFF;

	//variable to define which segment the value is going to be displayed in
	int currentDigit = 0;

	//hold value current segment will display
	int segmentDisplay;

	//if the input is a non-zero value then it will be displayed accross the segments.
	while (Value > 0) {

		//Isolate the last digit
		segmentDisplay = SevenSegmentDisplay(Value % 10);

		//divide itself by 10 so the value matches the isolated last digit
		Value /= 10;

		//Clear the display value to be used
		displayValue = displayValue & ~(0xFF << (currentDigit * 8));

		//shift the digit into its correct place
		displayValue = displayValue | (segmentDisplay << (currentDigit * 8));

		//increment so the program can move onto the next segment
		currentDigit++;
	}

	return displayValue;
}

int SevenSegmentDisplay(char Digit)
{
	int Segments;

	//Case for all the different possible characters that can be displayed on a seven segment display
	switch (Digit)
	{
	case 0:
		Segments = 0x40;
		break;
	case 1:
		Segments = 0xF9;
		break;
	case 2:
		Segments = 0x24;
		break;
	case 3:
		Segments = 0x30;
		break;
	case 4:
		Segments = 0x19;
		break;
	case 5:
		Segments = 0x12;
		break;
	case 6:
		Segments = 0x02;
		break;
	case 7:
		Segments = 0xF8;
		break;
	case 8:
		Segments = 0x00;
		break;
	case 9:
		Segments = 0x10;
		break;
	case 'F':
		Segments = 0x0E;
		break;
	case 'n':
		Segments = 0x2B;
		break;
	case 'O':
		Segments = 0x40;
		break;
	case 'P':
		Segments = 0x0C;
		break;
	case 'A':
		Segments = 0x08;
		break;
	case 'U':
		Segments = 0x41;
		break;
	case 'C':
		Segments = 0x46;
		break;
	case 'S':
		Segments = 0x12;
		break;
	case 'E':
		Segments = 0x06;
		break;
	case 'r':
		Segments = 0x2F;
		break;
	case 'c':
		Segments = 0x27;
		break;
	case 'H':
		Segments = 0x09;
		break;
	case 'L':
		Segments = 0xC7;
		break;
	default:
		Segments = 0x7F;
		break;
	}
	return Segments;
}
