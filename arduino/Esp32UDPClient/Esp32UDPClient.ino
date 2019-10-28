#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "ublox";
const char * networkPswd = "furuta123";
IPAddress ip(10, 0, 0, 5);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(10, 0, 0, 1);
IPAddress dns2(10, 0, 0, 1);

const char * udpAddress = "10.0.0.1"; // IP address to send the data to
const int udpPort = 5050;

//The udp library class
WiFiUDP udp;
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);
boolean connected = false;

struct udp_frame
{
  unsigned int pos_rad;
  uint8_t vel_sign;
  unsigned int vel_rad;
} frame;

void setup()
{
  // Initilize hardware serial:
  Serial.begin(115200);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  frame.pos_rad = 314;
  frame.vel_sign = 1;
  frame.vel_rad = 5067;

}

void serialize_frame(unsigned char* buffer, struct udp_frame* frame)
{
  memcpy(buffer, (const unsigned char*)&frame->pos_rad, 4);
  memcpy(buffer + 4, (const unsigned char*)&frame->vel_sign, 1);
  memcpy(buffer + 5, (const unsigned char*)&frame->vel_rad, 4);
}

void loop()
{
  if (connected)  //only send data when connected
  {
    //Send a packet
    unsigned char buf9[9];
    serialize_frame(buf9, &frame);
    udp.beginPacket(udpAddress, udpPort);
    udp.write(buf9, 9);
    udp.endPacket();
    Serial.println("Packet sent!");
  }
  delayMicroseconds(1);
}
