#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

#define PI_SCALED 314

//===================Global Variables===================//
struct udp_frame
{
  unsigned short pos_rad;
  uint8_t vel_sign;
  unsigned int vel_rad;
} frame;
unsigned long t0 = 0, t1 = 0, dt = 0;
double velocity = 0.0;
short current_position = 0, previous_position = 0;
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

  frame.pos_rad = 0;
  frame.vel_sign = 0;
  frame.vel_rad = 0;
}

void serialize_frame(unsigned char* buffer, struct udp_frame* frame)
{
  memcpy(buffer, (const unsigned char*)&frame->pos_rad, 2);
  memcpy(buffer + 2, (const unsigned char*)&frame->vel_sign, 1);
  memcpy(buffer + 3, (const unsigned char*)&frame->vel_rad, 4);
}

void loop()
{
  current_position = getAngle();
  frame.pos_rad = current_position;
  if(current_position >= PI_SCALED) current_position -= 2*PI_SCALED;
  t1 = micros();
  dt = t1 - t0;
  velocity = 1000000 * (((1.0 * current_position) - (1.0 * previous_position)) / dt);
  previous_position = current_position;
  t0 = t1;
  frame.vel_sign = velocity < 0 ? 1 : 0;
  frame.vel_rad = velocity < 0 ? (unsigned int)(-1 * velocity * 100) : (unsigned int)(velocity * 100);
  Serial.print(frame.pos_rad);
  Serial.print(", ");
  Serial.print(frame.vel_sign);
  Serial.print(", ");
  Serial.println(frame.vel_rad);
  if (wifi_connected)  //only send data when connected
  {
    //Send a packet
    unsigned char buf7[7];
    serialize_frame(buf7, &frame);
    udp.beginPacket(udpAddress, udpPort);
    udp.write(buf7, 7);
    udp.endPacket();

    if (udp.parsePacket())
    {
      // code for encoder zero setting
    }
  }

  delayMicroseconds(1);
}
