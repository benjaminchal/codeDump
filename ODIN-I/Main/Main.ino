#include <string.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <dht.h>
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
String log_dataString;
volatile boolean sentence_needed = true;
int tx_counter = 10000;
byte gps_set_sucess = 0 ;


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
  Serial.begin(9600);
  GPS_init();
  updateSen();
  pinMode(RADIOPIN, OUTPUT);
  initialise_interrupt();
  Serial.begin(9600);
  sdINIT();
}
 
void loop()
{
  while(GPS.available() > 0)
  {
    tGPS.encode(GPS.read());
  }
  // for you to have a play with ;-
  if(logTimer < millis()){
    switch(flight_stage){
      case 0: //pre-flight mode
        if(tGPS.sentencesWithFix() == 0){
          Serial.println("NO FIX FOUND");
          snprintf(live_datastring,102,"FIX NOT FOUND");
        }
        break;
    }
    updateSen();
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
    log_dataString = "Time,Temp,Humidity";
    saveRow();
  }
}

void updateSen(){
  //DHT.read11(DHT11_PIN); //get data from the DHT11 sensor
}

void logData(){
  //Construct the datastring formatted for CSV
  log_dataString = millis();
  log_dataString += ",";
  log_dataString += 25;//DHT.temperature;
  log_dataString += ",";
  log_dataString += 20;//DHT.humidity;

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

void GPS_init(){
  GPS.begin(9600); 
  // START OUR SERIAL DEBUG PORT
  Serial.println("GPS INIT");
  Serial.println("Initialising....");
  //
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  //
  GPS.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  GPS.begin(4800);
  GPS.flush();
 
  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
}

//GPS STUFF
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    GPS.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (GPS.available()) {
      b = GPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}
