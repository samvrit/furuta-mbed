#include "kalman_filter.h"
#include "OdinWiFiInterface.h"

#ifdef DEVICE_WIFI_AP
static const char *wifi_ssid = MBED_CONF_APP_WIFI_SSID;
static const char *wifi_password = MBED_CONF_APP_WIFI_PASSWORD;
static const char *ap_ip = MBED_CONF_APP_AP_IP;
static const char *ap_netmask = MBED_CONF_APP_AP_NETMASK;
static const char *ap_gateway = MBED_CONF_APP_AP_GATEWAY;
#endif

static void process_data(unsigned char *recv_buf, const nsapi_addr_t *address);

OdinWiFiInterface *_wifi;
#define ECHO_SERVER_PORT 5050

DigitalOut debug_pin1(D5);
CAN can1(PA_11, PA_12, 1000000);

typedef union {
    float value;
    char buffer[sizeof(float)];
} interfacePacket_t;

volatile interfacePacket_t canPacket = {.value = 0.0f};

volatile interfacePacket_t x2 = {.value = 500.0f}, x3 = {.value = 500.0f};

Ticker ticker;

EventQueue control_queue(32 * EVENTS_EVENT_SIZE);

const float32_t A_minus_B_K_f32[36] = 
{
    1.0000,    0.0001,    0.0003,    0.0001,    0.0000,    0.0000,
   -0.0000,    1.0001,    0.0003,   -0.0000,    0.0001,    0.0000,
    0.0000,   -0.0001,    0.9997,    0.0000,   -0.0000,    0.0001,
   -0.0466,    1.8160,    5.1239,    0.9133,    0.6802,    0.7570,
   -0.0533,    2.0811,    5.8598,   -0.0992,    1.7782,    0.8660,
    0.0582,   -2.2760,   -6.4036,    0.1084,   -0.8511,    0.0529
};

const float32_t C_f32[18] =
{
     1,     0,     0,     0,     0,     0,
     0,     1,     0,     0,     0,     0,
     0,     0,     1,     0,     0,     0
};

const float32_t Kalman_gain_f32[18] = 
{
    0.0205,    0.0223,   -0.0241,
    0.0223,    0.0263,   -0.0277,
   -0.0241,   -0.0277,    0.0315,
    7.2089,    8.2396,   -9.0073,
    8.2435,    9.4313,  -10.3079,
   -9.0136,  -10.3125,   11.2827
};

const float32_t LQR_gain_f32[6] = 
{
    3.1623, -122.7917, -348.0623, 5.8876, -46.2046, -51.4185
};

float32_t Xest_f32[6];
float32_t Xest_prev_f32[6];
float32_t measurement_vector_f32[3];
float32_t measurement_prediction_vector_f32[3];
float32_t measurement_error_vector_f32[3];
float32_t correction_vector_f32[6];
float32_t u_f32[1];

arm_matrix_instance_f32 A_minus_B_K;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 Kalman_gain;
arm_matrix_instance_f32 LQR_gain;
arm_matrix_instance_f32 Xest;
arm_matrix_instance_f32 Xest_prev;
arm_matrix_instance_f32 measurement_vector;
arm_matrix_instance_f32 measurement_prediction_vector;
arm_matrix_instance_f32 measurement_error_vector;
arm_matrix_instance_f32 correction_vector;
arm_matrix_instance_f32 u;

void matrices_init(void)
{
    uint32_t srcRows, srcColumns;

    srcRows = 6;
    srcColumns = 6;
    arm_mat_init_f32(&A_minus_B_K, srcRows, srcColumns, (float32_t *)A_minus_B_K_f32);

    srcRows = 3;
    srcColumns = 6;
    arm_mat_init_f32(&C, srcRows, srcColumns, (float32_t *)C_f32);

    srcRows = 6;
    srcColumns = 3;
    arm_mat_init_f32(&Kalman_gain, srcRows, srcColumns, (float32_t *)Kalman_gain_f32);

    srcRows = 1;
    srcColumns = 6;
    arm_mat_init_f32(&LQR_gain, srcRows, srcColumns, (float32_t *)LQR_gain_f32);

    srcRows = 6;
    srcColumns = 1;
    arm_mat_init_f32(&Xest, srcRows, srcColumns, (float32_t *)Xest_f32);

    srcRows = 6;
    srcColumns = 1;
    arm_mat_init_f32(&Xest_prev, srcRows, srcColumns, (float32_t *)Xest_prev_f32);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&measurement_vector, srcRows, srcColumns, (float32_t *)measurement_vector_f32);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&measurement_prediction_vector, srcRows, srcColumns, (float32_t *)measurement_prediction_vector_f32);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&measurement_error_vector, srcRows, srcColumns, (float32_t *)measurement_error_vector_f32);

    srcRows = 6;
    srcColumns = 1;
    arm_mat_init_f32(&correction_vector, srcRows, srcColumns, (float32_t *)correction_vector_f32);

    srcRows = 1;
    srcColumns = 1;
    arm_mat_init_f32(&u, srcRows, srcColumns, (float32_t *)u_f32);

}

int compute_a_priori(void)
{
    const arm_status status = arm_mat_mult_f32(&A_minus_B_K, &Xest_prev, &Xest);

    if(ARM_MATH_SUCCESS == status)
    {
        return 1;
    }
    else return 0;
}

int compute_a_posteriori(void)
{
    const arm_status status1 = arm_mat_mult_f32(&C, &Xest_prev, &measurement_prediction_vector);
    const arm_status status2 = arm_mat_sub_f32(&measurement_vector, &measurement_prediction_vector, &measurement_error_vector);
    const arm_status status3 = arm_mat_mult_f32(&Kalman_gain, &measurement_error_vector, &correction_vector);

    if((ARM_MATH_SUCCESS == status1) && (ARM_MATH_SUCCESS == status2) && (ARM_MATH_SUCCESS == status3))
    {
        return 1;
    }
    else return 0;
}

int add_a_priori_a_posteriori(void)
{
    const arm_status status = arm_mat_add_f32(&Xest, &correction_vector, &Xest);

    if(ARM_MATH_SUCCESS == status)
    {
        return 1;
    }
    else return 0;
}

int compute_torque_command(void)
{
    const arm_status status = arm_mat_mult_f32(&LQR_gain, &Xest, &u);
    *u_f32 *= -1.0f;    // multiply with -1 because equation is u = -K*x

    if(ARM_MATH_SUCCESS == status)
    {
        return 1;
    }
    else return 0;
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

static void process_data(unsigned char *recv_buf, const nsapi_addr_t *address)
{
    if(10U == address->bytes[3])
    {
        debug_pin1 = 1;
        memcpy((void *)x2.buffer, recv_buf, 4);
        printf("%.5f\r\n",x2.value);
        debug_pin1 = 0;
    }
    else if(5U == address->bytes[3])
    {
        memcpy((void *)x3.buffer, recv_buf, 4);
    }
}

static void control_calculation(void)
{
    
    compute_a_priori();
    compute_a_posteriori();
    add_a_priori_a_posteriori();
    compute_torque_command();
    canPacket.value = 0.1f;
    
}

void control_loop(void)
{
    debug_pin1 = 0;
    nsapi_size_or_error_t errcode;
    UDPSocket sock;

    SocketAddress sockAddr;
    int n = 0;

    /*Start AP*/
    _wifi = new OdinWiFiInterface(true);
    start_ap();

    /*Socket initialization*/
    errcode = sock.open(_wifi);
    if (errcode != NSAPI_ERROR_OK)
    {
        printf("UDPSocket.open() fails, code: %d\r\n", errcode);
    }

    errcode = sock.bind(ap_ip, ECHO_SERVER_PORT);
    if (errcode < 0)
    {
        printf("UDPSocket.connect() fails, code: %d\r\n", errcode);
    }
    else
    {
        printf("UDP: connected with %s server\r\n", ap_ip);
    }

    matrices_init();
    ticker.attach_us(control_calculation, 200);
    while(1)
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
            //printf("\n UDPSocket.recv() failed\r\n");
            // return -1;
        }
        can1.write(CANMessage(0x1, (const char *)&canPacket.buffer, sizeof(float)));        
    }
    sock.close();
    stop_ap();
}
