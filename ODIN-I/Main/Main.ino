
#include <stdlib.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <dht.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//Construct instances classes
//dht DHT;
SoftwareSerial GPS(6, 7);
TinyGPSPlus tGPS;

//Defining the pins
#define DHT11_PIN 7
#define CHIP_SEL 4
#define RADIOPIN 9
#define BAUD_RATE 50 // change as required
 
volatile int tx_status = 0;
volatile char *ptr = NULL;
char currentbyte;
int currentbitcount;
char log_dataString[50];
volatile boolean sentence_needed = true;
int tx_counter = 10000;
byte gps_set_sucess = 0 ;

char tLAT[10];
char tLNG[10];
//char tALT[5];

// 0 = pre-launch
// 1 = Accent
// 2 = Deccent
// 3 = landed
int flight_stage = 0;


char send_datastring[102];
char live_datastring[102];

unsigned long int logTimer = 2000;
 
 
// RTTY Interrupt Routine
ISR(TIMER1_COMPA_vect){
  switch (tx_status){
 
    case 0: // when the next byte needs to be gotten
      if (ptr){
        currentbyte = *ptr; // read first byte where pointer is pointing too
        if (currentbyte){
          tx_status = 1;
          sentence_needed = false;
          // warning! The lack of "break" in this branch means that we
          // fall through to "case 1" immediately, in order to start
          // sending the start bit.
        }
        else {
          //ptr = &send_datastring[0];
          sentence_needed = true;
          break;
        }
      }
      else {
        sentence_needed = true;
        break;
      }
 
    case 1: // first bit about to be sent
      rtty_txbit(0); // send start bit
      tx_status = 2;
      currentbitcount = 1; // set bit count to 0 ready for incrementing to 7 for last bit of a ASCII-7 byte
      break;
 
    case 2: // normal status, transmitting bits of byte (including first and last)
 
      rtty_txbit(currentbyte & 1); // send the currentb bit
 
      if (currentbitcount == 7){ // if we've just transmitted the final bit of the byte
        tx_status = 3;
      }
 
      currentbyte = currentbyte >> 1; // shift all bits in byte 1 to right so next bit is LSB
      currentbitcount++;
      break;
 
    case 3: // if all bits have been transmitted and we need to send the first of two stop bits
      rtty_txbit(1); // send first stop bit
      tx_status = 4;
      break;
 
    case 4: // ready to send the last of two stop bits
      rtty_txbit(1); // send the final stop bit
      ptr++; // increment the pointer for reading next byte in buffer
      tx_status = 0;
      break;
 
    }
 
}


 
 
// function t o toggle radio pin high and low as per the bit
void rtty_txbit (int bit)
{
 if (bit)
 {
 // high
 digitalWrite(RADIOPIN, HIGH);
 }
 else
 {
 // low
 digitalWrite(RADIOPIN, LOW);
 }
}  
 
 
 
void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / (BAUD_RATE - 1);  // set compare match register to desired timer count
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
 
 
 
void setup()
{
  GPS.begin(9600); 
  Serial.begin(9600);
//  updateSen();
  pinMode(RADIOPIN, OUTPUT);
  initialise_interrupt();
  //Serial.begin(9600);
  sdINIT();
}
 
void loop()
{
  while(GPS.available() > 0)
  {
    tGPS.encode(GPS.read());
    //Serial.write(GPS.read());
  }
  // for you to have a play with ;-
  if(logTimer < millis()){
    switch(flight_stage){
      case 0: //pre-flight mode
        if(tGPS.sentencesWithFix() == 0){
          Serial.println("NO FIX FOUND");
          snprintf(live_datastring,102,"FIX NOT FOUND");
        }else{
          //String tlat = String(tGPS.location.lat(), 6);
          //String tlng = String(tGPS.location.lat(), 6);
          
          dtostrf(tGPS.location.lat(),9, 6, tLAT);
          
          dtostrf(tGPS.location.lng(),9, 6, tLNG);
          
          snprintf(live_datastring,102,"$$test,%d,%2d:%2d:%2d,%s,%s,%d", tx_counter,
          tGPS.time.hour(), tGPS.time.minute(), tGPS.time.second(),
          tLAT, tLNG, tGPS.altitude.meters());
          tx_counter++;
        }
        break;
    }
    //updateSen();
    logData();
    logTimer = millis() + 2000;
  }
  
  if(sentence_needed){
    //snprintf(live_datastring,102,"$$test,%d,19:59:00,21.0000,12.4549,1000,%d", tx_counter, temp);
    setDataStr(live_datastring);
    tx_counter++;
  }
}


void setDataStr(char s[102]){
  int temp = 25;//DHT.temperature;
  
  //snprintf(send_datastring,102,"$$test,%d,19:59:00,21.0000,12.4549,1000,%d", tx_counter, temp);
  //

  strcpy(send_datastring ,s);
  unsigned int CHECKSUM = gps_CRC16_checksum(send_datastring); // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(send_datastring,checksum_str);
  ptr = &send_datastring[0];
  sentence_needed = false;
  Serial.print("TX SET: ");
  Serial.print(send_datastring);
}

void sdINIT(){
  if(SD.begin(CHIP_SEL)){ //Open up communication with the SD card
    Serial.println("SD card Sucsess");
  }else{
    Serial.println("WARNING: SD fail");
  }
  //If the file datalog.csv does not exist then set up the headings for the spreadsheet
  if (!SD.exists("datalog.csv")){
    Serial.println("datalog.csv, Writing Headers");
    snprintf(log_dataString,50, "Time,Latitude,Longditude,Altitude");
    saveRow();
  }
}

void updateSen(){
  //DHT.read11(DHT11_PIN); //get data from the DHT11 sensor
}

void logData(){
  //Construct the datastring formatted for CSV
  snprintf(log_dataString, 50,"%2d:%2d:%2d,%s,%s,%d",
  tGPS.time.hour(), tGPS.time.minute(), tGPS.time.second(),
  tLAT, tLNG, tGPS.altitude.meters());

  if(saveRow()){ //Calls the saveRow() function to write the dataString to the SD card
      Serial.print("LOGGED: ");
      Serial.println(log_dataString);
  }else{
    Serial.println("Log Failed");
  }
}

int saveRow(){
  File dataFile = SD.open("datalog.csv", FILE_WRITE); // Open datalog.csv the file in WRITE mode
  if (dataFile) {
    dataFile.println(log_dataString); //Appends the dataString to the end of the file
    dataFile.close(); //Closes the file
    return 1;
  }else{
    return 0;
    Serial.println("Could not open datalog.csv");
  }
}

uint16_t gps_CRC16_checksum (char *strings) {
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(strings); i++) {
    c = strings[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}

