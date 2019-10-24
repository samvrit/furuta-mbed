#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

#ifdef DEVICE_WIFI_AP
static const char *wifi_ssid = MBED_CONF_APP_WIFI_SSID;
static const char *wifi_password = MBED_CONF_APP_WIFI_PASSWORD;
static const char *ap_ip = MBED_CONF_APP_AP_IP;
static const char *ap_netmask = MBED_CONF_APP_AP_NETMASK;
static const char *ap_gateway = MBED_CONF_APP_AP_GATEWAY;
#endif

struct udp_frame
{
    unsigned int pos_rad;
    uint8_t vel_sign;
    unsigned int vel_rad;
};

extern MemoryPool<udp_frame, 12> mpool;
extern Queue<udp_frame, 12> queue;

static void start_ap(nsapi_security_t security);

static void stop_ap();

void deserialize_frame(unsigned char *buffer, struct udp_frame *frame);

void sensors_receive();