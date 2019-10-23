/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
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

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "10.0.0.1";
const int udpPort = 5050;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;
unsigned int pwm_dc = 0;

struct udp_frame
{
  unsigned int pos_rad;
  unsigned int vel_sign;
  unsigned int vel_rad;
} frame;

void setup(){
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
    memcpy(buffer+4, (const unsigned char*)&frame->vel_sign, 1);
    memcpy(buffer+5, (const unsigned char*)&frame->vel_rad, 4);
}

void loop(){
  //only send data when connected
  if(connected){
    //Send a packet
    unsigned char buf9[9];
    char str[] = "";
    serialize_frame(buf9, &frame);
//    sprintf(str, "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", buf9[0], buf9[1], buf9[2], buf9[3], buf9[4], buf9[5], buf9[6], buf9[7], buf9[8]);
//    Serial.write(str);
//    Serial.write('|');
    Serial.print(9);
    Serial.write("\n");
    udp.beginPacket(udpAddress,udpPort);
    udp.write(buf9, 9);
    udp.endPacket();
    Serial.println("Packet sent!");
    if(pwm_dc == 100)
      pwm_dc = 0;
    else
      pwm_dc += 10;
  }
  //Wait for 1 second
  delay(1000);
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  WiFi.config(ip, gateway, subnet, dns1, dns2);
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}
