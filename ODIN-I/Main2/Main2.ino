//Includes
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <dht.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS_UBX.h>
TinyGPS tGPS;


//DEFINE
#define DHT22_PIN 7
#define CHIP_SEL 4
#define RADIOPIN 9
#define BAUD_RATE 50 // change as required

//GLOBAL VARS

  // rtty innterupt vars
  volatile int tx_status = 0;
  volatile char *ptr = NULL;
  char send_datastring[102];
  char live_datastring[102];
  volatile boolean sentence_needed = true;
  char currentbyte;
  int currentbitcount;
  int tx_counter = 0;

  // gps vars
  char tLAT[10];
  char tLNG[10];
  char tALT[10];
  byte gps_hour, gps_minute, gps_second;
  long gps_lat, gps_lon;
  unsigned long gps_fix_age;
  byte gps_set_sucess = 0;
  
  // sd vars
  char log_dataString[50];

  // misc vars
  int flight_stage = 0;
  unsigned long int logTimer = 10000;

//RTTY CODE

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
  
  // checksum gen
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

  // set standard tx
  void stdTxStr(char s[102]){
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
  }
  
  void setDataStr(char *s){
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

//SPI SD CARD

  // initilise sd card
  void sdINIT(){
    SD.begin(CHIP_SEL); //Open up communication with the SD card
    
    //If the file datalog.csv does not exist then set up the headings for the spreadsheet
    if (!SD.exists("datalog.csv")){
      snprintf(log_dataString,50, "Time,Latitude,Longditude,Altitude");
      saveRow();
    }
  }

  // construct and log datastring
  void logData(){
    //Construct the datastring formatted for CSV
    snprintf(log_dataString, 50,"%02d:%02d:%02d,%s,%s,%s",
    gps_hour, gps_minute, gps_second,
    tLAT, tLNG, tALT);
    saveRow();
  }

  // Save row to datastring (char log_datastring[])
  int saveRow(){
    File dataFile = SD.open("datalog.csv", FILE_WRITE); // Open datalog.csv the file in WRITE mode
    if (dataFile) {
      dataFile.println(log_dataString); //Appends the dataString to the end of the file
      dataFile.close(); //Closes the file
      return 1;
    }else{
      return 0;
    }
  }

//GPS SERIAL

  // setup gps
  
  void GPS_setup() {
    Serial.begin(9600);
    // switch baudrate to 4800 bps
    //GPS_Serial.println("$PUBX,41,1,0007,0003,4800,0*13"); 
    //GPS_Serial.begin(4800);
    //GPS_Serial.flush();
    
    delay(5000);
    uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    while(!gps_set_sucess)
    {
      sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
      gps_set_sucess=getUBX_ACK(setNav);
    }
    gps_set_sucess=0;
    // turn off all NMEA sentences for the uBlox GPS module
    // ZDA, GLL, VTG, GSV, GSA, GGA, RMC
    Serial.println("$PUBX,40,ZDA,0,0,0,0*44");
    Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
    Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
    Serial.println("$PUBX,40,GSV,0,0,0,0*59");
    Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
    Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
    Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  
    delay(500);
  }


  // send gps
  void sendUBX(uint8_t *MSG, uint8_t len) {
    for(int i=0; i<len; i++) {
      Serial.write(MSG[i]);
    }
    Serial.println();
  }
  
  // Check ACK
  boolean getUBX_ACK(uint8_t *MSG) {
    uint8_t b;
    uint8_t ackByteID = 0;
    uint8_t ackPacket[10];
    unsigned long startTime = millis();
    //Serial.print(" * Reading ACK response: ");
   
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
        //Serial.println(" (SUCCESS!)");
        return true;
      }
   
      // Timeout if no valid response in 3 seconds
      if (millis() - startTime > 3000) { 
        //Serial.println(" (FAILED!)");
        return false;
      }
   
      // Make sure data is available to read
      if (Serial.available()) {
        b = Serial.read();
   
        // Check that bytes arrive in sequence as per expected ACK packet
        if (b == ackPacket[ackByteID]) { 
          ackByteID++;
          //Serial.print(b, HEX);
        } 
        else {
          ackByteID = 0;  // Reset and look again, invalid order
        }
   
      }
    }
  }

  // request uBlox to give fresh data
  boolean GPS_poll() {
    //GPS_Serial.println("$PUBX,00*33");
    Serial.println("$PUBX,00*33");
    delay(300);
    unsigned long starttime = millis();
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        if (tGPS.encode(c))
          return true;
      }
      // timeout
      if (millis() - starttime > 1000) {
        break;
      }
    }
    return false;
  }

  // string parse for outputs
  void gpsStrParse() {
    tGPS.crack_time(&gps_hour, &gps_minute, &gps_second, &gps_fix_age);
    tGPS.get_position(&gps_lat, &gps_lon, &gps_fix_age);
    dtostrf(gps_lat/100000.0,9, 6, tLAT);
    dtostrf(gps_lon/100000.0,9, 6, tLNG);
    dtostrf(tGPS.altitude()/100.0,9, 0, tLNG);
  }

void setup() {
  pinMode(RADIOPIN, OUTPUT);
  GPS_setup();
  sdINIT();
  initialise_interrupt();
}

void loop() {
  
    if(logTimer < millis()){
    switch(flight_stage){
      case 0: //pre-flight mode
        if(tGPS.has_fix() == 0){
          snprintf(live_datastring,102,"FIX NOT FOUND");
        }else{
          //String tlat = String(tGPS.location.lat(), 6);
          //String tlng = String(tGPS.location.lat(), 6);
          
          gpsStrParse();
          
          snprintf(live_datastring,102,"$$test,%d,%2d:%2d:%2d,%s,%s,%s", tx_counter,
          gps_hour, gps_minute, gps_second,
          tLAT, tLNG, tALT);
          tx_counter++;
        }
        break;
    }
    //updateSen();
    logData();
    logTimer = millis() + 2000;
  }
  
  if(sentence_needed){
    setDataStr(live_datastring);
    tx_counter++;
  }
}
