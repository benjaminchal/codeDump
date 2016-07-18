#include <dht.h>
#include <SPI.h>
#include <SD.h>

dht DHT; //Construct instances of the dht class

//Defining the pins for the SPI cs on the SD card and the DHT11 s pin.
#define DHT11_PIN 7
#define CHIP_SEL 4


String dataString; //Setting up the dataString string for the saveRow() function



void setup(){

  SD.begin(CHIP_SEL); //Open up communication with the SD card

  //If the file datalog.csv does not exist then set up the headings for the spreadsheet
  if (!SD.exists("datalog.csv")){
    dataString = "Time,Temp,Humidity";
    saveRow();
  }
  
}

void loop()
{
  DHT.read11(DHT11_PIN); //get data from the DHT11 sensor

  //Construct the datastring formatted for CSV
  dataString = millis();
  dataString += ",";
  dataString += DHT.temperature;
  dataString += ",";
  dataString += DHT.humidity;
  
  saveRow(); //Calls the saveRow() function to write the dataString to the SD card
  delay(5000);
}

void saveRow(){
  File dataFile = SD.open("datalog.csv", FILE_WRITE); // Open datalog.csv the file in WRITE mode
  if (dataFile) {
    dataFile.println(dataString); //Appends the dataString to the end of the file
    dataFile.close(); //Closes the file
  }
}
