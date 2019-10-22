/* WiFi AP Example
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "OdinWiFiInterface.h"
#include <string>

DigitalOut in_A(PF_1);
DigitalOut in_B(PF_0);
PwmOut motor_pwm(PA_10);

double pwm_dc = 0.0;

#ifdef DEVICE_WIFI_AP
static const char *wifi_ssid = MBED_CONF_APP_WIFI_SSID;
static const char *wifi_password = MBED_CONF_APP_WIFI_PASSWORD;
static const char *ap_ip = MBED_CONF_APP_AP_IP;
static const char *ap_netmask = MBED_CONF_APP_AP_NETMASK;
static const char *ap_gateway = MBED_CONF_APP_AP_GATEWAY;
#endif

#define ECHO_SERVER_PORT 5050

#pragma pack (1)
struct udp_frame
{
  unsigned int pos_rad;
  uint8_t vel_sign;
  unsigned int vel_rad;
} frame;

OdinWiFiInterface *_wifi;

void deserialize_frame(unsigned char* buffer, struct udp_frame* frame)
{
    frame->pos_rad = (*(buffer + 3) << 24) | (*(buffer + 2) << 16) | (*(buffer + 1) << 8) | (*(buffer + 0));
    frame->vel_sign = *(buffer + 4);
    frame->vel_rad = (*(buffer + 8) << 24) | (*(buffer + 7) << 16) | (*(buffer + 6) << 8) | (*(buffer + 5));
}

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

int main()
{

    in_A = 1;
    in_B = 0;
    motor_pwm.period_us(500);
    motor_pwm.write(pwm_dc);
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
        return -1;
    }

    errcode = sock.bind(ap_ip, ECHO_SERVER_PORT);
    if (errcode < 0)
    {
#ifdef USE_TCP
        printf("TCPSocket.connect() fails, code: %d\r\n", errcode);
#else
        printf("UDPSocket.connect() fails, code: %d\r\n", errcode);
#endif
        return -1;
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
                n = sock_data->recv((void *)recv_buf, sizeof(recv_buf));
                if (n > 0)
                {
                    printf("\n Received from client %d bytes: %s \n", n, recv_buf);

                    errcode = sock_data->send((void *)recv_buf, n);
                    if (errcode < 0)
                    {
                        printf("\n TCPSocket.send() fails, code: %d\n", errcode);
                        return -1;
                    }
                    else
                    {
                        printf("\n TCP: Sent %d Bytes to client\n", n);
                    }
                }
                else
                {
                    printf("\n TCPSocket.recv() failed");
                    return -1;
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
            printf("\n Received from client %d bytes: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X \r\n", n, recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3], recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7], recv_buf[8]);

            deserialize_frame((unsigned char*)recv_buf, &frame);
    
            printf("%d,%d,%d\r\n", frame.pos_rad, frame.vel_sign, frame.vel_rad);

            printf("Decoded: %u\r\n", frame.pos_rad);

            // motor_pwm.write(int_buffer*0.01);

            // errcode = sock.sendto(sockAddr, (void *)recv_buf, n);
            // if (errcode < 0)
            // {
            //     printf("\n UDPSocket.sendto() fails, code: %d\r\n", errcode);
            //     return -1;
            // }
            // else
            // {
            //     printf("\n UDP: Sent %d Bytes to client\r\n", n);
            // }
        }
        else
        {
            printf("\n UDPSocket.recv() failed\r\n");
            return -1;
        }
    }
    sock.close();
#endif

    
    stop_ap();
    return 0;
}