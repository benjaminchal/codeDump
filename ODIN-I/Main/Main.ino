#include <string.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <dht.h>
#include <SPI.h>
#include <SD.h>

dht DHT; //Construct instances of the dht class

//Defining the pins
#define DHT11_PIN 7
#define CHIP_SEL 4
#define RADIOPIN 9
#define BAUD_RATE 50 // change as required
 
volatile int tx_status = 0;
volatile char *ptr = NULL;
char currentbyte;
int currentbitcount;
String log_dataString;
volatile boolean sentence_needed = true;
int tx_counter = 10000;


char send_datastring[102];

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
  updateSen();
  pinMode(RADIOPIN, OUTPUT);
  initialise_interrupt();
  Serial.begin(9600);
  sdINIT();
}
 
void loop()
{
 // for you to have a play with ;-
  
 if(sentence_needed){
   setDataStr();
 }

 if(millis() > logTimer){
  updateSen();
  logData();
  logTimer = millis() + 2000;
 }
}


void setDataStr(){
  int temp = DHT.temperature;
  snprintf(send_datastring,102,"$$test,%d,19:59:00,21.0000,12.4549,1000,%d", tx_counter, temp);
  tx_counter++;
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
    log_dataString = "Time,Temp,Humidity";
    saveRow();
  }
}

void updateSen(){
  DHT.read11(DHT11_PIN); //get data from the DHT11 sensor
}

void logData(){
  

  //Construct the datastring formatted for CSV
  log_dataString = millis();
  log_dataString += ",";
  log_dataString += DHT.temperature;
  log_dataString += ",";
  log_dataString += DHT.humidity;

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

uint16_t gps_CRC16_checksum (char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}
