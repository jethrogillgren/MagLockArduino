/*
 * --------------------------------------
 * Arduino with a 12V MagLock - XBee for comms. 0013A200 418D8E11
 * Also has a VL6180x/GY6180 Range sensor.
 * --------------------------------------
 * 
 * PINOUT
 * 
 * Maglock > MOSFET > Arduino
 * TODO
 * 
 * XBEE > Arduino
 * 10 Gnd > Gnd
 * 1 VCC > 3.3V
 * 2 DOUT > 2
 * 3 DIN > 4
 * 
 * VL6180x/GY6180 Range
 * VIN > 5V
 * GND > GNd
 * SCL > A5
 * SDA > A4
 * 0 > D5
 * TODO
 */

#include <Wire.h>

#include <XBee.h>
#include <SoftwareSerial.h>

#include <VL6180X.h>

#include <Printers.h>
#include <elapsedMillis.h>


// MAGLOCK
int lockPin = 3;
bool locked = false;

// RANGE
#define RANGE 1
elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs
uint8_t wobbleIgnore = 3;
uint8_t d0;
uint8_t d1;
uint8_t distance;
uint8_t lastDistance = 0;

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x20
#define address1 0x22

///* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin0 = 5;
int enablePin1 = 6;

///* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;


//XBEE & COMMUNICATIONS
SoftwareSerial xbeeSerial(2, 4); // RX, TX

//Works with Series1 and 2
XBeeWithCallbacks xbee;

// Build a reuseable message packet to send to the Co-Ordinator
XBeeAddress64 coordinatorAddr = XBeeAddress64(0x00000000, 0x00000000);

uint8_t distanceMessagePayload[1] = {0};
ZBTxRequest distanceMessage = ZBTxRequest(coordinatorAddr, distanceMessagePayload,
sizeof(distanceMessagePayload));


#define MSG_LOCK   'l'
#define MSG_UNLOCK 'u'



void setup() {
  Serial.begin(9600);   // Initialize serial communications with the PC

  Wire.begin();

  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);


  // Solenoid
  pinMode(lockPin, OUTPUT);

  // RANGE
  SetSensorI2CAddresses();


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

  
  // RANGE
  //if(!locked) //This can be uncommented if we don't want to check for the draw being left opened
  //while the lock is engaged.
  //{
    d0 = sensor0.readRangeContinuous();
    d1 = sensor1.readRangeContinuous();
    
    distance = ( d0+d1) / 2;
    
    //Send updates regularly.  Sanity checks are not yet done, so that we at
    //least get some reading every 3s
    if (timeElapsed > 3000 )
    {
      Serial.println("Sending 3000ms packet");
      
      
      SendDistancePacket( distance );
      timeElapsed = 0;              // reset the counter to 0 so the counting starts over...
    }
    //Send updates when a change happens
    else if( distance != lastDistance )
    {
  //    Serial.print("New distance: ");
  //    Serial.println(distance);
  
      if( wobbleIgnore>0 )
      {
        if( distance > lastDistance  && (distance - lastDistance) < wobbleIgnore  )
          return;
        if( distance < lastDistance  && (lastDistance - distance) < wobbleIgnore  )
          return;
      }
  
      if(distance < 20 && lastDistance < 20)
      {
        //Serial.println("Ignoring negligable change for close distance");
        return;
      }
  
      if( abs(d1-d0) > 15 )
      {
        Serial.println("Skipping as reads are not agreeing.");
        return;
      }

      //Tell the server.
      lastDistance = distance;
      SendDistancePacket( distance );
    }
  //}//end if unlocked

}

//The funciton that shoots the solenoid/key
void Lock()
{
  Serial.println("Locking");
  locked=true;
  digitalWrite(lockPin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

}
void Unlock()
{
  Serial.println("Unlocking");
  locked=false;
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
        //(LED_BUILTIN, 2, 500);
        return;
      }
      p->println(F("Recieved:"));
        p->print("  Payload: ");
        printHex(*p, rx.getFrameData() + rx.getDataOffset(),
        rx.getDataLength(), F(" "), F("\r\n    "), 8);
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



//// RANGE

void SetSensorI2CAddresses()
{
  // Reset all connected sensors
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);

  digitalWrite(enablePin0, LOW);
  digitalWrite(enablePin1, LOW);

  delay(1000);
  
  SetSensorI2CAddress(0, enablePin0, &sensor0, address0 );
  SetSensorI2CAddress(1, enablePin1, &sensor1, address1 );

  delay(1000);
 
  Serial.println("Sensors ready! Start reading sensors in 3 seconds ...!");
  delay(3000);

}

void SetSensorI2CAddress( int i, int enablePin, VL6180X *sensor, int address )
{

  Serial.print("Start Sensor: ");
  Serial.print(i);
  Serial.print(" using pin ");
  Serial.print(enablePin);
  Serial.print(" as I2C Address ");
  Serial.print( address);
  Serial.println();
  
  digitalWrite(enablePin, HIGH);
  delay(50);
  sensor->init();
  sensor->configureDefault();
  sensor->setAddress(address);
  Serial.println(sensor->readReg(0x212),HEX); // read I2C address
  sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor->writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor->setTimeout(500);
  sensor->stopContinuous();
  sensor->setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor->startInterleavedContinuous(100);
  //sensor->startRangeContinuous(100);
  delay(100);
}

void SendDistancePacket( uint8_t distance )
{
  Serial.print(F("SENDING distance: "));
  Serial.print(distance);
  Serial.println();

  distanceMessagePayload[0] = distance;
  distanceMessage.setFrameId(xbee.getNextFrameId());
  
  //xbee.send(distanceMessage);

  // Send the command and wait up to N ms for a response.  xbee loop continues during this time.
  uint8_t status = xbee.sendAndWait(distanceMessage, 3000);
  if (status == 0)
  {
    Serial.println(F("SEND ACKNOWLEDGED"));
    timeElapsed = 0;              // reset the counter to 0 so the counting starts over...

  }
  else
  { //Complain, but do not reset timeElapsed - so that a new packet comes in and
    //tried again immedietly.
    Serial.print(F("SEND FAILED: "));
    printHex(status, 2);
    Serial.println();

    lastDistance = 999; //Trigger a resend right away

    //flashSingleLed(LED_BUILTIN, 3, 500);
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

