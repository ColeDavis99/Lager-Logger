/*================================================================
  Sketch displays Fahrenheit ambient temperature on an LCD screen
  ...Also the voltage read in from the thermistor (HW498 module)
  Using an Arduino Nano.
  
  Also logs temps to a micro SD card (FAT32 formatted)
 SD card attached to SPI bus as follows:
 ** MOSI - pin 11 on Arduino Uno/Duemilanove/Diecimila
 ** MISO - pin 12 on Arduino Uno/Duemilanove/Diecimila
 ** CLK - pin 13 on Arduino Uno/Duemilanove/Diecimila
 ** CS - pin 4 on Arduino Uno/Duemilanove/Diecimila
==================================================================*/

//LCD library
//#include <LiquidCrystal.h>

//SD library and SPI library
#include <SPI.h>
#include <SD.h>
File myFile; //This is our .csv that will store thermistor readings.
const int CHIP_SELECT_PIN = 4;
long writeCtr = 1;

//Variables for LCD display and LCD object
/*
   const int rs = 2,
          en = 3,
          d4 = 6,
          d5 = 7,
          d6 = 8,
          d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
*/

//Variables for thermistor module (HW498)
int ThermistorPin = 0;
int Vo;
float R1 = 9850;  //10,000 is stock, seems to be the easiest way to calibrate.
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07; //Find specs from manufacturer (or random tutorial online)

//Arduino Nano-specific constants for displaying voltage values
const float V_REF = 5.0;     // Analog reference voltage (e.g., 5V or 3.3V)
const float R_BITS = 10.0;   // ADC resolution (bits)
const float ADC_STEPS = (1 << int(R_BITS)) - 1; // Number of steps (2^R_BITS - 1)
float actualVoltage;

//Vars for doing multiple voltage reads then averaging
const int NUM_TEMP_SAMPLES = 20; //Can't go over 30 due to potential integer overflow (since max theoretical reading is 1024)
int Vo_sum;
float T_sum;
const unsigned int TOT_TEMP_SAMPLE_RANGE = 60000; //Take NUM_TEMP_SAMPLES # of readings across this # of ms.
const int SINGLE_TEMP_SAMPLE_DELAY = TOT_TEMP_SAMPLE_RANGE / NUM_TEMP_SAMPLES;


void setup() {
  Serial.begin(9600);
  while (!Serial){;} // wait for serial port to connect. Needed for native USB port only
  Serial.println("Attempting to initialize SD");

  delay(5000);//Delay, just to make sure stuff inits okay.
  if(SD.begin(CHIP_SELECT_PIN)){
    Serial.println("SD.begin() successful");
  }
  else{
    Serial.println("SD.begin() failed. Terminating");
    while(1){;}
  }

  
  //lcd.begin(16, 2);
  //lcd.blink();  
}

void loop() {
  Vo_sum = 0;
  T_sum = 0.0;

  //Sum up the temp samples
  for(int i=0; i<NUM_TEMP_SAMPLES; i++){
    //Read in the voltage
    Vo = analogRead(ThermistorPin);
    Vo_sum += Vo;
    
    //Calculate Temperature
    T_sum += CalcTemp(Vo); 
    delay(SINGLE_TEMP_SAMPLE_DELAY);
  }

  //Now get the the average across the samples
  Vo = Vo_sum / NUM_TEMP_SAMPLES;
  T = T_sum / NUM_TEMP_SAMPLES;

  //Calculate Voltage (for fun)
  actualVoltage = (Vo / ADC_STEPS) * V_REF;

  //Serial outputs
  
  Serial.print("Voltage: ");
  Serial.print(actualVoltage);
  Serial.println("v");
  Serial.print("Temperature: "); 
  Serial.print(T);
  Serial.println(" F");
  Serial.println();

  //LCD outputs
  /*
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(T);
  lcd.print("F");
  lcd.setCursor(0,1);
  lcd.print("Volt: ");
  lcd.print(actualVoltage);
  lcd.print("v");
  lcd.setCursor(15,1);
  */

  //SD Card outputs
  SDWrite(T, writeCtr);
  writeCtr++;
 }


/*==========================
   CUSTOM FUNCTIONS BELOW
============================*/
float CalcTemp(int Vo){
  //Hall's equation
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  T = (T * 9.0)/ 5.0 + 32.0;
  
  return T;
}


void SDWrite(float T, long writeCtr){
  myFile = SD.open("TEST.csv", FILE_WRITE);

// if the file opened okay, write to it:
  if (myFile)
  {
    myFile.print(writeCtr);
    myFile.print(", ");
    myFile.println(T);
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening TEST.csv");
  }
  
  // close the file:
  myFile.close();
}



void SDTest(){
  Serial.print("Initializing SD card...");

  if (!SD.begin(CHIP_SELECT_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
