#include "acquire_feedback.h"

#ifdef DEVICE_WIFI_AP
static const char *wifi_ssid = MBED_CONF_APP_WIFI_SSID;
static const char *wifi_password = MBED_CONF_APP_WIFI_PASSWORD;
static const char *ap_ip = MBED_CONF_APP_AP_IP;
static const char *ap_netmask = MBED_CONF_APP_AP_NETMASK;
static const char *ap_gateway = MBED_CONF_APP_AP_GATEWAY;
#endif

void process_data(unsigned char *recv_buf, const nsapi_addr_t *address);

OdinWiFiInterface *_wifi;
#define ECHO_SERVER_PORT 5050

MemoryPool<state_vector, 16> mpool;
Queue<state_vector, 16> feedback_queue;

volatile udpPacket_t x2, x3;

static void start_ap(nsapi_security_t security = NSAPI_SECURITY_WPA_WPA2)
{
    nsapi_error_t error_code;

    printf("\nStarting AP\r\n");

    // AP Configure and start
    error_code = _wifi->set_ap_network(ap_ip, ap_netmask, ap_gateway);
    MBED_ASSERT(error_code == NSAPI_ERROR_OK);

    //DHCP not available
    error_code = _wifi->set_ap_dhcp(false);
    MBED_ASSERT(error_code == NSAPI_ERROR_OK);

    //Set beacon interval to default value
    _wifi->set_ap_beacon_interval(100);

    //Set ap ssid, password and channel
    error_code = _wifi->ap_start(wifi_ssid, wifi_password, security, cbWLAN_CHANNEL_01);
    MBED_ASSERT(error_code == NSAPI_ERROR_OK);

    printf("\nAP started successfully, wifi password: %s\r\n", wifi_password);
}

static void stop_ap()
{
    nsapi_error_t error_code;

    error_code = _wifi->ap_stop();
    MBED_ASSERT(error_code == NSAPI_ERROR_OK);

    printf("\nAP stopped\r\n");
}

void process_data(unsigned char *recv_buf, const nsapi_addr_t *address)
{
    state_vector *x_vect = mpool.alloc();

    if(10U == address->bytes[3])
    {
        memcpy((void *)x2.buffer, recv_buf, 4);
        x_vect->x[1] = x2.value;
        //printf("Got X1: %.5f\r\n", x2.value);
    }
    else if(5U == address->bytes[3])
    {
        memcpy((void *)x3.buffer, recv_buf, 4);
        x_vect->x[2] = x3.value;
        //printf("Got X2: %.5f\r\n", x3.value);
    }

    feedback_queue.put(x_vect);
}

void sensors_receive()
{
    nsapi_size_or_error_t errcode;
#ifdef USE_TCP
    nsapi_error_t *err;
    TCPSocket sock, *sock_data;
#else
    UDPSocket sock;
#endif

    SocketAddress sockAddr;
    int n = 0;

    /*Start AP*/
    _wifi = new OdinWiFiInterface(true);
    start_ap();

    /*Socket initialization*/
    errcode = sock.open(_wifi);
    if (errcode != NSAPI_ERROR_OK)
    {
#ifdef USE_TCP
        printf("TCPSocket.open() fails, code: %d\r\n", errcode);
#else
        printf("UDPSocket.open() fails, code: %d\r\n", errcode);
#endif
        // return -1;
    }

    errcode = sock.bind(ap_ip, ECHO_SERVER_PORT);
    if (errcode < 0)
    {
#ifdef USE_TCP
        printf("TCPSocket.connect() fails, code: %d\r\n", errcode);
#else
        printf("UDPSocket.connect() fails, code: %d\r\n", errcode);
#endif
        // return -1;
    }
    else
    {
#ifdef USE_TCP
        printf("TCP: connected with %s server\r\n", ap_ip);
#else
        printf("UDP: connected with %s server\r\n", ap_ip);
#endif
    }

/*Echo server*/
#ifdef USE_TCP
    if (sock.listen() == 0)
    {
        sock_data = sock.accept(err = NULL);

        if (sock_data != NULL)
        {
            while (true)
            {

                char recv_buf[7] = "";
                n = sock_data->recvfrom(&sockAddr, (void *)recv_buf, sizeof(recv_buf));
                if (n > 0)
                {
                    nsapi_addr_t address;
                    address = sockAddr.get_addr();
                    process_data((unsigned char *)&recv_buf, &address);
                }
                else
                {
                    printf("\n TCPSocket.recv() failed");
                    // return -1;
                }
            }
        }
    }
    sock_data->close();
#else
    while (1)
    {
        char recv_buf[7] = "";
        n = sock.recvfrom(&sockAddr, (void *)recv_buf, sizeof(recv_buf));
        if (n > 0)
        {
            nsapi_addr_t address;
            address = sockAddr.get_addr();
            process_data((unsigned char *)&recv_buf, &address);
        }
        else
        {
            printf("\n UDPSocket.recv() failed\r\n");
            // return -1;
        }
    }
    sock.close();
#endif
    stop_ap();
}
