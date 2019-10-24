#include "acquire_feedback.h"

OdinWiFiInterface *_wifi;
#define ECHO_SERVER_PORT 5050

MemoryPool<udp_frame, 12> mpool;
Queue<udp_frame, 12> queue;

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

void deserialize_frame(unsigned char *buffer, struct udp_frame *frame)
{
    frame->pos_rad = (*(buffer + 3) << 24) | (*(buffer + 2) << 16) | (*(buffer + 1) << 8) | (*(buffer + 0));
    frame->vel_sign = *(buffer + 4);
    frame->vel_rad = (*(buffer + 8) << 24) | (*(buffer + 7) << 16) | (*(buffer + 6) << 8) | (*(buffer + 5));
}

void sensors_receive()
{
    nsapi_size_or_error_t errcode;
    nsapi_error_t *err;
#ifdef USE_TCP
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

                char recv_buf[1024] = "";
                n = sock_data->recvfrom(&sockAddr, (void *)recv_buf, sizeof(recv_buf));
                if (n > 0)
                {
                    nsapi_addr_t address;
                    address = sockAddr.get_addr();
                    // printf("\n Received from client %d.%d.%d.%d, %d bytes: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X \r\n", address.bytes[0], address.bytes[1], address.bytes[2], address.bytes[3], n, recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3], recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7], recv_buf[8]);

                    udp_frame *frame = mpool.alloc();
                    deserialize_frame((unsigned char *)recv_buf, frame);

                    queue.put(frame);
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
        char recv_buf[9] = "";
        n = sock.recvfrom(&sockAddr, (void *)recv_buf, sizeof(recv_buf));
        if (n > 0)
        {
            nsapi_addr_t address;
            address = sockAddr.get_addr();
            // printf("\n Received from client %d.%d.%d.%d, %d bytes: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X \r\n", address.bytes[0], address.bytes[1], address.bytes[2], address.bytes[3], n, recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3], recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7], recv_buf[8]);

            udp_frame *frame = mpool.alloc();
            deserialize_frame((unsigned char *)recv_buf, frame);

            queue.put(frame);
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
