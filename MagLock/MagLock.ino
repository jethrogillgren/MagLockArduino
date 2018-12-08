// *
// * --------------------------------------
// * Arduino with a 12V MagLock - wired.
// * --------------------------------------
// * 
// * 
// */


#include <Printers.h>
#include <elapsedMillis.h>

// MagLock
int lockPin = 3;

#define MSG_ACTIVATE 'a'
#define MSG_RESET   'r'


void setup() {
  Serial.begin(9600);   // Initialize serial communications with the PC

  // Solenoid
  pinMode(lockPin, OUTPUT);
}

void loop() {
  
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    
    // get the new byte:
    char cmd = (char)Serial.read();

    Serial.print("Cmd:");
    Serial.println(cmd);
    switch (cmd)
    {
    case MSG_ACTIVATE: 
      digitalWrite(lockPin, HIGH);
      break;
      
    case MSG_RESET: 
      digitalWrite(lockPin, LOW);
      break;
      
    default: // If an invalid character, do nothing
      Serial.print("Unable to parse command: ");
      Serial.println(cmd);
      break;
    }
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
