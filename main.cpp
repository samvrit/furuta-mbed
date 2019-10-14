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
#include "TCPSocket.h"
#include <string> 

#ifdef DEVICE_WIFI_AP
static const char *wifi_ssid = MBED_CONF_APP_WIFI_SSID;
static const char *wifi_password = MBED_CONF_APP_WIFI_PASSWORD;
static const char *ap_ip = MBED_CONF_APP_AP_IP;
static const char *ap_netmask = MBED_CONF_APP_AP_NETMASK;
static const char *ap_gateway = MBED_CONF_APP_AP_GATEWAY;
#endif

#define ECHO_SERVER_PORT   5050

OdinWiFiInterface   *_wifi;


static void start_ap(nsapi_security_t security = NSAPI_SECURITY_WPA_WPA2)
{
    nsapi_error_t error_code;

    printf("\nStarting AP\n");

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
    
    printf("\nAP started successfully\n");
}


static void stop_ap()
{
    nsapi_error_t error_code;

    error_code = _wifi->ap_stop();
    MBED_ASSERT(error_code == NSAPI_ERROR_OK);
    
    printf("\nAP stopped\n");

}
 
int main()
{
    nsapi_size_or_error_t errcode;
    nsapi_error_t *err;
    TCPSocket sock, *sock_data;
    int n = 0;
    char recv_buf[1024];
    
    /*Start AP*/
    
    _wifi = new OdinWiFiInterface(true);

    start_ap(); 
    
    /*Socket initialization*/
    
    errcode = sock.open(_wifi);
    if (errcode != NSAPI_ERROR_OK) {
        printf("TCPSocket.open() fails, code: %d\n", errcode);
        return -1;
    }  

    errcode = sock.bind("10.0.0.1", ECHO_SERVER_PORT);
    if (errcode < 0) {
        printf("TCPSocket.connect() fails, code: %d\n", errcode);
        return -1;
    }
    else {
        printf("TCP: connected with %s server\n", "10.0.0.1");
    }
   
    /*Echo server*/
    if (sock.listen() == 0)
    {
        sock_data = sock.accept(err = NULL);
        
        if (sock_data != NULL)
        {
            while (true)
            {
                n = sock_data->recv((void*) recv_buf, sizeof(recv_buf));
                if (n > 0) 
                {
                    printf("\n Received from client %d bytes: %s \n", n, recv_buf);
                    
                    errcode = sock_data->send((void*) recv_buf, n);
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
    sock.close();
    
    stop_ap();
                        
    return 0;
}
