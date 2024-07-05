#include <Arduino.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <wdt_samd21.h>

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
#define USING_TIMER_TC3         false     // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4         false     // Not to use with Servo library
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         true
#define USING_TIMER_TCC1        false
#define USING_TIMER_TCC2        false     // Don't use this, can crash on some boards

// Uncomment To test if conflict with Servo library
//#include "Servo.h"

/////////////////////////////////////////////////////////////////

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "SAMDTimerInterrupt.h"

//////////////////////////////////////////////

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define TIMER_INTERVAL_MS        1000

#if USING_TIMER_TC3
  #define SELECTED_TIMER      TIMER_TC3
#elif USING_TIMER_TC4
  #define SELECTED_TIMER      TIMER_TC4
#elif USING_TIMER_TC5
  #define SELECTED_TIMER      TIMER_TC5
#elif USING_TIMER_TCC
  #define SELECTED_TIMER      TIMER_TCC
#elif USING_TIMER_TCC1
  #define SELECTED_TIMER      TIMER_TCC1
#elif USING_TIMER_TCC2
  #define SELECTED_TIMER      TIMER_TCC
#else
  #error You have to select 1 Timer  
#endif

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);



const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message
int relay = 2;
int sensorPin=A0;
int sensorValue;
int delayTime;

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to

String message = "Smartduck";   // send a message

int seconden;

bool onReceive(int packetSize);
void sendMessage(String outgoing);

void TimerHandler()
{
  // Doing something here inside ISR
   seconden++;
   Serial.print("seconden: ");
   Serial.println(seconden);
   if (seconden>=delayTime){
    seconden=0;
    ITimer.disableTimer();
    digitalWrite(relay, LOW);
    Serial.println("relay off");
   }
}

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define TIMER_INTERVAL_MS        1000      // 1s = 1000ms

void setup() {
  Serial.begin(9600);                   // initialize serial
  //while (!Serial);

  
  if (!LoRa.begin(868E6)) {             // initialize ratio at 868MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  pinMode(relay, OUTPUT);
  // Initialze WDT with a 2 sec. timeout
   wdt_init ( WDT_CONFIG_PER_2K );
}

void loop() {
  if (onReceive(LoRa.parsePacket())) {
     
      if (ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler)){
       
        digitalWrite(relay, HIGH);
        Serial.println("relay on");
        sendMessage(message);
        Serial.println("Sending " + message);  
      }
      else
        Serial.println("Can't set ITimer. Select another freq. or timer");
    
    
  }

  // "feed" the WDT to avoid restart
      wdt_reset();
  sensorValue = analogRead(sensorPin);
  delayTime=sensorValue*900/1023;
  Serial.print("Delay time= ");
  Serial.println(delayTime);
  
  
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

bool onReceive(int packetSize) {
  if (packetSize == 0) return false;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return false;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return false;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  return incoming.equals(message); 
}

