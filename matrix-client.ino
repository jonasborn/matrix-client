#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiUdp.h>
#include <Ticker.h>

int LED_COUNT = 85;
int LED_PIN = 4;

WiFiManager wifiManager;
Ticker ticker;
WiFiUDP udp;
unsigned int udpPort = 4210;
char udpPacket[18];

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void tick() {
  digitalWrite(BUILTIN_LED, !(digitalRead(BUILTIN_LED)));
}

long btol(byte a, byte b, byte c, byte d) {
  long val2 = (unsigned long) a << 24 ;
  val2 |= (unsigned long) b << 16 ;
  val2 |= (unsigned long) c << 8 ;
  val2 |= (unsigned long) d ;
  return val2;
}

long btoi(byte a, byte b) {
  int val2 = (unsigned int) a << 8 ;
  val2 |= (unsigned int) b ;
  return val2;
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void setup() {
  wifiManager.setAPCallback(configModeCallback);
  pinMode(BUILTIN_LED, OUTPUT);
  ticker.attach(0.6, tick);

  if (!wifiManager.autoConnect("matrix")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.reset();
    delay(1000);
  }

  Serial.println("connected...yeey :)");
  ticker.detach();
  digitalWrite(BUILTIN_LED, LOW);

  strip.begin();
  strip.show();

  udp.begin(udpPort);

}

void loop() {

  // 0 = night      [m]
  // 1 = clear      [m]
  // 2 = show       [m]
  // 3 = brightness [m,h_]
  // 4 = set        [m,id__,r_,g_,b_]
  // 5 = strip      [m,id__,r_,g_,b_,from,to__]

  int packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
    int len = udp.read(udpPacket, 18);
    if (len > 0)
    {
      udpPacket[len] = '\0';
    }
    Serial.printf("UDP packet contents: %s\n", udpPacket);

    byte mode = udpPacket[0];

    if (mode == 0x00) {
      byte replyPacket = {0}; //TODO ADD REAL VALUE
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write(replyPacket);
      udp.endPacket();
    }
    if (mode == 0x01) strip.clear();
    if (mode == 0x02) strip.show();
    if (mode == 0x03) {
      strip.setBrightness(btoi(udpPacket[1], udpPacket[2]));
    }
    if (mode == 0x04) {
      long id = btol(udpPacket[1], udpPacket[2], udpPacket[3], udpPacket[4]);
      int red = btoi(udpPacket[5], udpPacket[6]);
      int green = btoi(udpPacket[7], udpPacket[8]);
      int blue = btoi(udpPacket[9], udpPacket[10]);
      strip.setPixelColor(id, red, green, blue);
    }
    if (mode == 0x05) {
      long id = btol(udpPacket[1], udpPacket[2], udpPacket[3], udpPacket[4]);
      int red = btoi(udpPacket[5], udpPacket[6]);
      int green = btoi(udpPacket[7], udpPacket[8]);
      int blue = btoi(udpPacket[9], udpPacket[10]);
      uint32_t color = strip.Color(red, green, blue);

      long first = btol(udpPacket[11], udpPacket[12], udpPacket[13], udpPacket[14]);
      long count = btol(udpPacket[15], udpPacket[16], udpPacket[17], udpPacket[18]);

      strip.fill(color, first, count);
    }

  }



}
