// *
// * --------------------------------------
// * Arduino with a 12V MagLock - XBee for comms. 0013A200 418D8E11
// * --------------------------------------
// * 
// * 
// */

#include <Wire.h>

#include <XBee.h>
#include <SoftwareSerial.h>

#include <Printers.h>
#include <elapsedMillis.h>


// MAGLOCK
int lockPin = 3;


//XBEE & COMMUNICATIONS
SoftwareSerial xbeeSerial(2, 4); // RX, TX

//Works with Series1 and 2
XBeeWithCallbacks xbee;

// Build a reuseable message packet to send to the Co-Ordinator
XBeeAddress64 coordinatorAddr = XBeeAddress64(0x00000000, 0x00000000);

uint8_t testMessagePayload[1] = {0};
ZBTxRequest testMessage = ZBTxRequest(coordinatorAddr, testMessagePayload, sizeof(testMessagePayload));


#define MSG_LOCK   'l'
#define MSG_UNLOCK 'u'



void setup() {
  Serial.begin(9600);   // Initialize serial communications with the PC

  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);


  // Solenoid
  pinMode(lockPin, OUTPUT);




  // XBEE
  xbeeSerial.begin(9600);
  xbee.setSerial(xbeeSerial);

  // Make sure that any errors are logged to Serial. The address of
  // Serial is first cast to Print*, since that's what the callback
  // expects, and then to uintptr_t to fit it inside the data parameter.
  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onZBTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);

  // These are called when an actual packet received
  xbee.onZBRxResponse(zbReceive, (uintptr_t)(Print*)&Serial);

  // Print any unhandled response with proper formatting
  xbee.onOtherResponse(printResponseCb, (uintptr_t)(Print*)&Serial);

  // Enable this to print the raw bytes for _all_ responses before they
  // are handled
  xbee.onResponse(printRawResponseCb, (uintptr_t)(Print*)&Serial);


  //Start Unlocked
  Unlock();

  Serial.println("SETUP");
}

void loop() {
  // Continuously let xbee read packets and call callbacks.
  xbee.loop();
}

//The funciton that shoots the solenoid/key
void Lock()
{
  Serial.println("Locking");
  digitalWrite(lockPin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

}
void Unlock()
{
  Serial.println("Unlocking");
  digitalWrite(lockPin, LOW);
  digitalWrite(LED_BUILTIN, LOW);


}

//// XBEE / COMMUNICATION FUNCTIONS
//FrameType:  0x90  recieved.
void zbReceive(ZBRxResponse& rx, uintptr_t data) {

  if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED
      || rx.getOption() == ZB_BROADCAST_PACKET ) {

      //Debug it out - copied from the lib
      Print *p = (Print*)data;
      if (!p) {
        Serial.println("ERROR 2");
        //flashSingleLed(LED_BUILTIN, 2, 500);
        return;
      }
      p->println(F("Recieved:"));
        p->print("  Payload: ");
        printHex(*p, rx.getFrameData() + rx.getDataOffset(), rx.getDataLength(), F(" "), F("\r\n    "), 8);
      p->println();
        p->print("  From: ");
        printHex(*p, rx.getRemoteAddress64() );
      p->println();

      //This Project only ever takes a 1 char command
      //printHex(rx.getData()[0], 2);
      parseCommand( (char) rx.getData()[0] );
      
      //flashSingleLed(LED_BUILTIN, 5, 50);
      
  } else {
      // we got it (obviously) but sender didn't get an ACK
      Serial.println("ERROR 1");
      //flashSingleLed(LED_BUILTIN, 1, 500);
  }
}


void serialEvent() {
  while (Serial.available()) {
    
    // get the new byte and act if it was a command
    //parseCommand( (char) Serial.read() );
  }
}

// Parse input, take action if it's a valid character
void parseCommand( char cmd )
{
  Serial.print("Cmd:");
  Serial.println(cmd);
  switch (cmd)
  {
  case MSG_UNLOCK: 
    Unlock();
    break;
    
  case MSG_LOCK:
    Lock();
    break;

  default: // If an invalid character, do nothing
    Serial.print("Unable to parse command: ");
    Serial.println(cmd);
    break;
  }
}



// UTIL FUNCTIONS
void printHex(int num, int precision) {
     char tmp[16];
     char format[128];

     sprintf(format, "0x%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
}
// Note, this blocks
void flashSingleLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}
