#include <string.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
#define RADIOPIN 9
#define LED_1 13
#define BAUD_RATE 50 // change as required
 
volatile int tx_status = 0;
volatile char *ptr = NULL;
char currentbyte;
int currentbitcount;
 
volatile boolean sentence_needed = true;
int tx_counter = 10000;
 
char send_datastring[102] = "M0NBA Interupt Driven RTTY beacon \n";
 
 
 
// RTTY Interrupt Routine
ISR(TIMER1_COMPA_vect){
  switch (tx_status){
 
    case 0: // when the next byte needs to be gotten
      if (ptr){
  currentbyte = *ptr; // read first byte where pointer is pointing too
        if (currentbyte){
          tx_status = 1;
          sentence_needed = false;
          digitalWrite(LED_1, LOW);
          // warning! The lack of "break" in this branch means that we
          // fall through to "case 1" immediately, in order to start
          // sending the start bit.
        }
        else {
          sentence_needed = true;
          digitalWrite(LED_1, HIGH);
          break;
        }
      }
      else {
        sentence_needed = true;
        digitalWrite(LED_1, HIGH);
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
      Serial.println("");
      break;
 
    }
 
}
 
 
// function to toggle radio pin high and low as per the bit
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
  pinMode(RADIOPIN, OUTPUT);
  initialise_interrupt();
  delay(100);
  ptr = &send_datastring[0];
  Serial.begin(9600);
}
 
void loop()
{
 // for you to have a play with ;-
 float timemill = millis();
 int timemin = timemill/60000;
 Serial.println(timemin);
  
 if(sentence_needed){
  snprintf(send_datastring,102,"$$test,%d,19:59:00,21.0000,12.4549,1000", tx_counter);
  tx_counter++;
  unsigned int CHECKSUM = gps_CRC16_checksum(send_datastring); // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(send_datastring,checksum_str);
  ptr = &send_datastring[0];
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

