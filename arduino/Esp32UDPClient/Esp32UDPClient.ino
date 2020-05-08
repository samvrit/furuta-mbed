//===================Includes===================//
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

//===================Defines===================//
#define PI 3.1415F
#define ANGLE_SCALING_FACTOR (PI/(1U << 16U))

//===================Global Variables===================//
typedef union {
    float value;
    char buffer[sizeof(float)];
} udpPacket_t;
udpPacket_t udpPacket;
unsigned long t0 = 0, t1 = 0, dt = 0;
double velocity = 0.0;
unsigned int previous_position;
byte mac[6];
bool device1, device2;
//==================================================//

//===================Wi-Fi Configs===================//
const char * networkName = "ublox";
const char * networkPswd = "furuta123";
IPAddress *ip;  // instantiate an IP address, but initialize it depending on MAC address
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(10, 0, 0, 1);
IPAddress dns2(10, 0, 0, 1);

const char * udpAddress = "10.0.0.1"; // IP address to send the data to
const int udpPort = 5050;
const int udpLocalPort = 8888;

WiFiUDP udp;
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);
boolean wifi_connected = false;
//==================================================//

//===================SPI Configs===================//
SPIClass * hspi = NULL;
static const int spiClk = 25000000; // 25 MHz
//================================================//

void setup()
{
  hspi = new SPIClass(HSPI);
  hspi->begin();
  pinMode(15, OUTPUT); //HSPI SS
  Serial.begin(115200);

  WiFi.macAddress(mac); // get MAC address of the PHY
  device1 = ((mac[3] == 0x7A) && (mac[4] == 0x10) && (mac[5] == 0xEC)) ? 1 : 0; // use MAC address to differentiate between the two devices
  device2 = !device1;

  if (device1)
    ip = new IPAddress(10, 0, 0, 5);
  else if (device2)
    ip = new IPAddress(10, 0, 0, 10);

  connectToWiFi(networkName, networkPswd);  //Connect to the WiFi network
}

void loop()
{
  udpPacket.value = getAngle();
  Serial.print("Position: ");
  Serial.print(udpPacket.value);
  Serial.print(" | ");
  Serial.print(udpPacket.buffer[0], HEX);
  Serial.print(" ");
  Serial.print(udpPacket.buffer[1], HEX);
  Serial.print(" ");
  Serial.print(udpPacket.buffer[2], HEX);
  Serial.print(" ");
  Serial.println(udpPacket.buffer[3], HEX);
  if (wifi_connected)  //only send data when connected
  {
    //Send a packet
    udp.beginPacket(udpAddress, udpPort);
    udp.write((const uint8_t *)udpPacket.buffer, 4);
    udp.endPacket();

    if (udp.parsePacket())
    {
      // code for encoder zero setting
    }
  }

  delayMicroseconds(1);
}
