#include <Adafruit_Sensor.h>	// https://github.com/adafruit/Adafruit_Sensor
#include <SdFat.h>				// https://github.com/greiman/SdFat-beta
#include "RCR_Bmp180.h"
#include <Wire.h>
#include <SPI.h>

//sd card filename
#define LOG_FILENAME		"RRS_data.dat"

//debug options
#define DEBUG_ALTITUDEPLZ	false
#define DEBUG_FLIGHTMODE	false
#define DEBUG_VELOCITY		false
#define DEBUG_EMERGENCY		false

//constants
#define PAYLOAD_MASS_LBM	9									//the weight of the payload (pounds!)
#define TOO_FAST_MPS		(0.3048*(-sqrt(2*75/PAYLOAD_MASS_LBM)))	//the descent speed at which the RRS will deploy its chute (m/s)
#define BUFF_N				32									//the number of past vehicle states kept in memory for the calculation of velocity (unitless)
#define SAFETY_THRESHOLD_M	0										//the altitude, below which, the backup chute will not deploy (meters!)

//pins
#define BACKUP_CHUTE_PIN	A14
#define RSO_PERMISSION_PIN	0
//#define MRS_SIGNAL_PIN	

struct stateStruct {
	float alt;                                                    //The most recent altitude reading from Adafruit BMP180 sensor           (m)
	float vel;                                                    //The most recent velocity derived from calculateVelocity() function     (m/s)
	unsigned long time;                                           //Time since the program began                                           (us)
	float buff_t;                                                 //The time relative to the present moment. (used in calculateVelocity()) (s)
} ;                       //Stores past BUFF_N state structures

//begin global variables----------------------------
//SD card object
SdFatSdio sd;   
File data;

//bmp180 card object
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);   //stores BMP180 object

char response;
float padAlt = 0;

//end global variables------------------------------

void setup()
{
	Serial.begin(9600);
	delay(1000);
	Serial.println("serial has begun...");

	//declare pinModes
	pinMode(BACKUP_CHUTE_PIN, OUTPUT);
	pinMode(RSO_PERMISSION_PIN, INPUT);

	//initialize SD card
	if (!sd.begin()) {                                            //Determine if microSD card is initialized and ready to be used.
		Serial.println("No SD card DETECTED!");
		return;
	}
	else {
		Serial.println("SD card Initialized");         
	}

	//initialize bmp180
	if (!bmp.begin()) {                                           //Determine if BMP180 is initialized and ready to be used
		Serial.println("NO Bmp180 DETECTED!");
	}
	else {
		Serial.println("Bmp180 Initialized");
	}

	//begin a new data file on the SD card, set the launch pad altitude, and  enter flight mode
	newFlight();
	setPadAlt();
	Serial.printf("The vertical velocity at which the RRS will deploy the backup chute is %.3f m/s or %.3f ft/s\n", TOO_FAST_MPS, TOO_FAST_MPS*3.28084);
	delay(3000);
	flightMode();
}


/*__  __       _         _
|  \/  |     (_)       | |
| \  / | __ _ _ _ __   | |     ___   ___  _ __
| |\/| |/ _` | | '_ \  | |    / _ \ / _ \| '_ \
| |  | | (_| | | | | | | |___| (_) | (_) | |_) |
|_|  |_|\__,_|_|_| |_| |______\___/ \___/| .__/
										 | |
										 |_| */
/*To add a menu item, add a case statement below and add a print statement in printMenu*/

void loop()
{
	if (Serial.available() > 0) {
		switch (Serial.read()) {
		case 'F':
			Serial.println("\n\n----- Entering Flight Mode -----;");
			eatYourBreakfast();                                       //Flushes serial port
			newFlight();
			setPadAlt();
			delay(2000);                                            //pause for dramatic effect....
			flightMode();                                           //Initiate Flight Mode
			break;
		default:
			Serial.print("Unkown code received: ");
			Serial.println(response);
			break;
		}
		printMenu();
	}
}

/**************************************************************************/
/*!
@brief  Launch and test sequence. Where all the action is.
Author: Daniel and Ben
*/
/**************************************************************************/
void flightMode() {
	bool ignited = false;									//a boolean used to keep track of whether or not the RRS has fired its backup chute
	struct stateStruct vehicleState;						//a stateStruct that stores the current state of the vehicle
	while (Serial.available() == 0) {						//while we haven't hit any keys on the serial monitor
		//get vehicle's current state
		vehicleState.time = micros();	
		vehicleState.alt = altitude_plz() - padAlt;
		vehicleState.vel = calculateVelocity(vehicleState);

		//rsoPermission = digitalRead(RSO_PERMISSION_PIN);

		if ((vehicleState.alt > SAFETY_THRESHOLD_M) &&  ((vehicleState.vel <= TOO_FAST_MPS))) { //
			digitalWrite(BACKUP_CHUTE_PIN, HIGH);
			ignited = true;
		}
		logData(vehicleState.time, vehicleState.alt, vehicleState.vel, ignited);

#if DEBUG_FLIGHTMODE
		Serial.println("");
		Serial.println("FLIGHT MODE--------------");
		Serial.printf("Time: %lu\nAltitude: %.3f\nVelocity: %.3f\nIgnition: %d\n", vehicleState.time, vehicleState.alt, vehicleState.vel, ignited);
#endif
	}
}


/**************************************************************************/
/*!
@brief  Stores all data to the SD card
Author: Ben
*/
/**************************************************************************/
void logData(unsigned long myTime, float myAlt, float myVel, bool deploy) {
	File myFile = sd.open(LOG_FILENAME, FILE_WRITE);
	if (myFile) {
		myFile.printf("%lu,%.3f,%.3f,%d", myTime, myAlt, myVel, deploy);
		myFile.println("");
		myFile.close();
	}
}


/**************************************************************************/
/*!
@brief  Prepares varaibles for new launch
Author: Jacob
*/
/**************************************************************************/
void newFlight(void) {
	
	sd.remove(LOG_FILENAME);                             //Removes prior flight data file

	File data = sd.open(LOG_FILENAME, FILE_WRITE);       //Creates new data file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
	}
	else {                                         
		data.println("times, alts, vels, deploys");
		data.close();                                               //Closes data file after use.
	}
	//initialize past raw states
	//for (unsigned int i = 0; i<BUFF_N; i++) {
	//	pastRawStates[i].alt = (float)(0);
	//	pastRawStates[i].vel = (float)(0);
	//	pastRawStates[i].time = (unsigned long)(0);
	//}
} // END newFlight()


  /**************************************************************************/
  /*!
  @brief  Checks if Bmp180 has a reading ready, retrieves reading and 
  requests a new readings if yes, returns false if not ready yet
  Pronounced "altitude please".
  Author: Ben
  */
  /**************************************************************************/
float altitude_plz(void) {
	float returnVal = 0;
	float pressure_kPa;
	float pressure_; //units are Pa*10?
	static float lastAlt= 0;

	if (bmp.RCR_readyYet()) {

		bmp.RCR_getPressure(&pressure_kPa);                         //picks up the pressure reading from the Bmp180, then puts in a request for a new one
#if DEBUG_ALTITUDEPLZ
		Serial.print("RCR_getPressure returned: ");
		Serial.print(pressure_kPa);
		Serial.print("  kPa at t = ");
		Serial.println(millis());
#endif
		pressure_ = pressure_kPa / 100.0F;
		lastAlt = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure_);
		returnVal = lastAlt;
		//Serial.print("ready: ");
		//Serial.println(returnVal);
#if DEBUG_ALTITUDEPLZ
		Serial.print("RCR_getPressure returned: ");
		Serial.print(pressure_kPa);
		Serial.print("  kPa at t = ");
		Serial.println(millis());
		Serial.print("  altitude = ");
		Serial.println(returnVal);
#endif
	}
	else {
		returnVal = lastAlt;
		//Serial.print("not ready: ");
		//Serial.println(returnVal);
	}
	return returnVal;
} // END altitude_plz()


  /**************************************************************************/
  /*!
  @brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
  Author: Jacob & Ben
  - Algorithm developed by Ben Stringer, function written by Jacob Cassady
  */
  /**************************************************************************/
float calculateVelocity(struct stateStruct rawState) { //VARIABLES NEEDED FOR CALULATION
	float sumTimes = 0, sumTimes2 = 0, sumAlt = 0, sumAltTimes = 0;
	float numer = 0, denom = 0, velocity = 0, pastTime = 0, newTime = 0;
	static stateStruct pastRawStates[BUFF_N];

	//shift new readings into arrays   
	for (int i = BUFF_N; i > 0; i--) {
		copyState(&pastRawStates[i], &pastRawStates[i - 1]);           //copyState(1,2) deep copies information from struct 1 into struct 2.
	}
	rawState.buff_t = 0;
	copyState(&pastRawStates[0], &rawState);                      //Moves newest state into the 0 position of pastRawStates array.

																  //time relative to the current moment
	for (int i = BUFF_N; i > 0; i--) {
		pastTime = (float)pastRawStates[i - 1].time;
		newTime = (float)rawState.time;
		pastRawStates[i - 1].buff_t = (pastTime - newTime) / (float)1000000;   //Calculates buff_t values for pastRawStates array
	}

#if DEBUG_VELOCITY && DEBUG_EMERGENCY
	Serial.println("");
	Serial.println("Past states post-shift");
	printPastStates(pastRawStates);                             //If in DEBUG_VELOCITY and DEBUG_EMERGENCY, print all pastRawStates for verification of function output
#endif

																	//FIND SUMS FOR BMP
	for (unsigned int i = 0; i < BUFF_N; i++) {                   //Calculates sums for left side of velocity equation.
		sumTimes += (float)(pastRawStates[i].buff_t);
		sumTimes2 += (float)((pastRawStates[i].buff_t) * (pastRawStates[i].buff_t));
		sumAlt += pastRawStates[i].alt;
		sumAltTimes += ((float)pastRawStates[i].buff_t * pastRawStates[i].alt);
	}

	//CALCULATE LEFT SIDE OF EQUATION
	numer = ((sumTimes * sumAlt) - (BUFF_N * sumAltTimes));
	denom = ((sumTimes*sumTimes) - (BUFF_N * sumTimes2));
	velocity = numer / denom;

#if DEBUG_VELOCITY                                              //Reports velocity equation pieces for debugging if in DEBUG_VELOCITY mode.
	Serial.println();
	Serial.println("VELOCITY--------------------;");
	Serial.print("leftSide: ");
	Serial.print(velocity, 3);
	Serial.println(";");
	Serial.print("numer: ");
	Serial.print(numer, 3);
	Serial.println(";");
	Serial.print("denom: ");
	Serial.print(denom, 3);
	Serial.println(";");
	Serial.print("sumTimes: ");
	Serial.print(sumTimes, 3);
	Serial.println(";");
	Serial.print("sumTimes2: ");
	Serial.print(sumTimes2, 3);
	Serial.println(";");
	Serial.print("sumAlt: ");
	Serial.print(sumAlt, 3);
	Serial.println(";");
	Serial.print("sumAltTimes: ");
	Serial.print(sumAltTimes, 3);
#endif
	return velocity;
}// END calculateVelocity()

 /**************************************************************************/
 /*!
 @brief  Deep copies one state to another
 Author: Jacob
 */
 /**************************************************************************/
void copyState(struct stateStruct* destination, struct stateStruct* original) {
	destination->alt = original->alt;
	destination->vel = original->vel;
	destination->time = original->time;
	destination->buff_t = original->buff_t;
} // END copyState()


  /**************************************************************************/
  /*!
  @brief  Clears the serial buffer.. This
  is helpful for carriage returns and things of that sort that
  hang around after you got what you wanted.
  Author: Ben
  */
  /**************************************************************************/
void eatYourBreakfast() {
	while (Serial.available() > 0) {
		delay(2);
		Serial.read();
	}
} // END eatYourBreakfast()

void setPadAlt(void) {
	padAlt = altitude_plz();//puts in a request for a reading
	delay(40);
	padAlt = altitude_plz();//retrieves reading
	Serial.print("Launch pad altitude = ");
	Serial.println(padAlt);
}

/**************************************************************************/
/*!
@brief  *HIDDEN* Menu Function.  Prints menu options.
Author: Jacob Cassady
*/
/**************************************************************************/
void printMenu() {
	Serial.println("");
	Serial.println("\n\n--------- Menu -----------;");
	Serial.println("'F' - (F)light Mode;");
}


/**************************************************************************/
/*!
@brief  prints the state struct
Author: Ben
*/
/**************************************************************************/
void printState(struct stateStruct state, String label) {
	Serial.println(label);
	Serial.print("alt =   ");
	Serial.println(state.alt, 3);
	Serial.print("vel =   ");
	Serial.println(state.vel, 4);
	Serial.print("t =     ");
	Serial.println(state.time, 6);
} // END printState()

  /**************************************************************************/
  /*!
  @brief  Prints all pastRawState values.
  Author: Jacob
  */
  /**************************************************************************/
void printPastStates(struct stateStruct* pastStates) {
	Serial.println("");
	for (int i = 0; i < BUFF_N; i++) {
		printState(pastStates[i], i);
	}
} // END printPastStates()

  /**************************************************************************/
  /*!
  @brief  Prints one state and it's location in the pastRawStates array
  Author: Jacob
  */
  /**************************************************************************/
void printState(struct stateStruct state, int label) {
	Serial.print(label);
	Serial.print(") alt = ");
	Serial.print(state.alt, 4);
	Serial.print(", vel = ");
	Serial.print(state.vel, 4);
	Serial.print(", time = ");
	Serial.print(state.time);
	Serial.print(", buff_t = ");
	Serial.print(state.buff_t, 4);
	Serial.println(");");
} //End printState()