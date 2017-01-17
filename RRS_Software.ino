#include <SdFat.h> //will need to install library. Ask Ben for it.


#define LOG_FILENAME "RRS_data.dat"
#define ERROR_FILENAME "RRS_errorLog.dat"

SdFatSdio sd;


void setup()
{

	if (!sd.begin()) {                                            //Determine if microSD card is initialized and ready to be used.
		Serial.println("No SD card DETECTED!");
		return;
	}
	else {
		Serial.println("SD card Initialized");         
	}
	newFlight();
}

void loop()
{

  /* add main program code here */

}



void logData(unsigned long myTime, float myAlt, float myVel, bool deploy) {
	File myFile = sd.open(LOG_FILENAME, FILE_WRITE);
	if (myFile) {
		myFile.printf("%lu,%.3f,%.3f,%d", myTime, myAlt, myVel, deploy);
		myFile.println("");
		myFile.close();
	}
}

void newFlight(void) {
	sd.remove(LOG_FILENAME);                             //Removes prior flight data file
	sd.remove(ERROR_FILENAME);                                 //Removes prior error file

	File data = sd.open(LOG_FILENAME, FILE_WRITE);       //Creates new data file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
	}
	else {                                         
		data.println("times, alts, vels, deploys");
		data.close();                                               //Closes data file after use.
	}

	data = sd.open(ERROR_FILENAME, FILE_WRITE);                //Creates new error file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
	}
	else {
		data.println("time(us),error");
		data.close();                                               //Closes data file after use.
	}
} // END newFlight()
