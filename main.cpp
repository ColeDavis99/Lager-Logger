/*================================================================
  * Using an Arduino Nano
  * Calculates temperature from a thermistor (HW498 module)
  * Logs temperature to an SD card (FAT32 formatted)
  
  * SD card attached to SPI bus as follows:
      MOSI - pin 11 on Arduino Nano
      MISO - pin 12 on Arduino Nano
      CLK - pin 13 on Arduino Nano
      CS - pin 4 on Arduino Nano
==================================================================*/

//Arduino library
#include <Arduino.h>

//SD library and SPI library
#include <SPI.h>
#include <SD.h>
const int CHIP_SELECT_PIN = 4;
unsigned long writeCtr = 0;

//Degbug LED Pin & debug control variables
const int DEBUG_LED_PIN = 2;
const bool REQUIRE_SD_CARD_INSERTED = 1;  //0 prevents SD.begin() from infinite looping if SD card not inserted, 1 lets the SD.begin() check happen
const bool DISPLAY_SERIAL_OUTPUT = 1;     //0 disables serial output, 1 enables it

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
const int NUM_TEMP_SAMPLES = 20;  //Take this many temp readings, then store their mean value on SD card
unsigned long Vo_sum;             //Stores the voltage sum
float T_sum;                      //Stores temperature sum
const unsigned int TOT_TEMP_SAMPLE_RANGE = 60000; //How often the mean temp gets stored as an entry in temperature_arr (in ms). Ultimately, each .csv row will represent TOT_TEMP_SAMPLE_RANGE units of time. 60,000 = 1 minute.
const int SINGLE_TEMP_SAMPLE_DELAY = TOT_TEMP_SAMPLE_RANGE / NUM_TEMP_SAMPLES;  //The ms delay needed to uniformly sample temperature across the TOT_TEMP_SAMPLE_RANGE
const int BATCH_SIZE = 30; // Increase SD card life by minimizing # of SD.open() and SD.close() calls. Do this by writing the temperature readings BATCH_SIZE at a time
float temperature_arr[BATCH_SIZE]; // Declare array to store the temperature readings between write batches.

/*=====================================================================================================
                                      CUSTOM FUNCTIONS BELOW
=======================================================================================================*/
float CalcTemp(int Vo)
{
  // Hall's equation
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 273.15;
  T = (T * 9.0) / 5.0 + 32.0;

  return T;
}

void SDWrite(unsigned long writeCtr, float* temperature_arr, const int BATCH_SIZE)
{
  //Create a new file object each time to ensure any previous opening errors are corrected.
  File myFile = SD.open("TEST2.csv", FILE_WRITE);
  delay(500);

  // if the file opened okay, write to it & turn off solid debug LED:
  if (myFile)
  {
    digitalWrite(DEBUG_LED_PIN, LOW);

    for(int i=0; i<BATCH_SIZE; i++){
      myFile.print(writeCtr+1 - (BATCH_SIZE-i));
      myFile.print(", ");
      myFile.println(temperature_arr[i]);
    }
    // close the file:
    myFile.close();
  }
  else
  {
    // if the file didn't open, print an error & solid light on debug LED:
    Serial.println("error opening TEST2.csv");
    digitalWrite(DEBUG_LED_PIN, HIGH);
  }

}


void setup() {
  //Init debug stuff.
  Serial.begin(9600);
  pinMode(DEBUG_LED_PIN, OUTPUT);

  // Wait for serial port to connect.
  while (!Serial){;} 
  Serial.println("Attempting to initialize SD");

  // Delay, to make sure SD card can init okay.
  delay(5000);

  //Success :)
  //pinMode(CHIP_SELECT_PIN, INPUT_PULLUP);
  if(SD.begin(CHIP_SELECT_PIN, SPI_HALF_SPEED) || !REQUIRE_SD_CARD_INSERTED){
    Serial.println("SD.begin() successful");
  }
  //Fail :(
  else{
    Serial.println("SD.begin() failed. Flashing LED 3 times repeatedly.");
    while(1){
      for(int i=0; i<3; i++){
        digitalWrite(DEBUG_LED_PIN, HIGH);
        delay(200);
        digitalWrite(DEBUG_LED_PIN, LOW);
        delay(200);  
      }
      delay(5000);
    }
  } 
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

  //Write to appropriate index of batch temperature storage array.
  temperature_arr[writeCtr % BATCH_SIZE] = T;

  //Calculate Voltage (for fun)
  actualVoltage = (Vo / ADC_STEPS) * V_REF;

  //Serial outputs
  if (DISPLAY_SERIAL_OUTPUT)
  {
    Serial.print("Sample #");
    Serial.println(writeCtr);
    Serial.print("Voltage: ");
    Serial.print(actualVoltage);
    Serial.println("v");
    Serial.print("Temperature: ");
    Serial.print(temperature_arr[writeCtr % BATCH_SIZE]);
    // Serial.print(T);
    Serial.println(" F");
    Serial.println();
  }
    // SD Card outputs
    if((((writeCtr+1) % BATCH_SIZE) == 0)){
      Serial.print("GETCHA SOME ");
      Serial.println(writeCtr);
      SDWrite(writeCtr, temperature_arr, BATCH_SIZE);
    }
    else{
      Serial.print("Not YET ");
      Serial.println(writeCtr);
    }
    writeCtr++;
  }