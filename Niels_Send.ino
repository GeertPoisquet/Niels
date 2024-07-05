
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <wdt_samd21.h>

const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin
int button = 2;
int buttonState;

bool buttonflag=false;
bool sendmessage=false;
String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
long debouncetime = 0;        // last send time
int debounceperiod=100;
int interval = 2000;          // interval between sends

String message = "Smartduck";   // send a message

void sendMessage(String outgoing);
bool onReceive(int packetSize);

void setup() {
  Serial.begin(9600);                   // initialize serial
  

  Serial.println("LoRa Duplex");
  pinMode(button, INPUT_PULLUP);
  // override the default CS, reset, and IRQ pins (optional)
  //LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(868E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  // Initialze WDT with a 2 sec. timeout
   wdt_init ( WDT_CONFIG_PER_2K );
}

void loop() {
  buttonState = digitalRead(button);
  if (buttonState == LOW && buttonflag== false) {
     buttonflag=true;
     sendmessage=true;
     debouncetime= millis();     
  }
  if(( millis() - debouncetime > debounceperiod)&& buttonState == HIGH){
    buttonflag=false;
  }

  if ((millis() - lastSendTime > interval) &&  sendmessage==true){
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
  }
  if ((msgCount>5) || (onReceive(LoRa.parsePacket()))){
    sendmessage=false;
    msgCount=0;
  }
  
  // "feed" the WDT to avoid restart
      wdt_reset();
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
     wdt_reset();
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

