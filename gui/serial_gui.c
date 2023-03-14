#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <tchar.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "host_comms_shared.h"

#define TIMESTEP (1e-3f)

#define ZERO_OFFSET_BTN_ID (1)
#define STREAMING_BTN_ID (2)
#define COM_CONNECT_BTN_ID (3)
#define DUTY_OVERRIDE_BTN_ID (4)
#define TORQUE_CMD_BTN_ID (5)
#define OVERRIDES_TOGGLE_BTN_ID (6)
#define DIRECTION_TOGGLE_BTN_ID (7)
#define MOTOR_ENABLE_TOGGLE_BTN_ID (8)
#define RESET_MOTOR_BTN_ID (9)
#define TRIGGER_FAST_LOGGING_BTN_ID (10)

#define PI (3.1415f)
#define DEG2RAD(x) ((x) * (2.0f * PI / 360.0f ))
#define ANGLES_WITHIN_BOUNDS_DEG (10.0f)

HWND hwnd;

HWND com_port_edit;
HWND duty_ratio_override_edit;
HWND torque_cmd_edit;
HWND com_port_baud_rate;
HWND x_hat_label[6];
HWND torque_cmd_label;
HWND measurement_label[3];
HWND rls_fault_bitfield_label;
HWND motor_fault_flag_label;
HWND controller_state_label;
HWND current_fb_label;
HWND v_bridge_label;
HWND duty_ratio_label;
HWND buffer_label;

HWND direction_btn;
HWND overrides_toggle_btn;
HWND motor_enable_toggle_btn;
HWND torque_cmd_btn;
HWND duty_override_btn;
HWND reset_motor_btn;
HWND streaming_btn;
HWND com_connect_btn;
HWND zero_offset_btn;
HWND trigger_fast_logging_btn;
HWND info_message;

HANDLE hCom;

static HBRUSH hbrBkgnd_regular = NULL;
static HBRUSH hbrBkgnd_green = NULL;
static HBRUSH hbrBkgnd_red = NULL;

// Types
union uint_to_float_U
{
    float value;
    uint8_t raw[4];
};

// Global variable declarations
bool currently_streaming = false;
bool com_currently_connected = false;

bool meas1_within_bounds = false;
bool meas2_within_bounds = false;
bool meas3_within_bounds = false;
bool rls_fault_flag = false;
bool motor_fault_flag = false;
bool controller_active = false;

bool fast_logging_triggered = false;

uint16_t fast_logging_index = 0U;

uint16_t direction_override_val = 0U;
uint16_t override_enable_val = 0U;
uint16_t motor_enable_val = 0U;

float fast_logging_signals[FAST_LOGGING_NUM_SIGNALS][FAST_LOGGING_BUFFER_SIZE] = { { 0.0f } };

void write_signals_to_file(void)
{
    FILE * logging_file_handle = fopen("logs/signals.csv", "w+");

    if (logging_file_handle != NULL)
    {
        float t = 0.0f;

        for(uint16_t i = 0; i < FAST_LOGGING_BUFFER_SIZE; i++)
        {
            char text[50] = "";
            snprintf(text, 50, "%f,%f,%f\n", t, fast_logging_signals[0][i], fast_logging_signals[1][i]);

            t += TIMESTEP;

            fputs(text, logging_file_handle);
        }
    }

    fclose(logging_file_handle);
}

// Step 4: the Window Procedure
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch(msg)
    {
        case WM_CLOSE:
            if(com_currently_connected)
            {
                const char stream = 't';
                WriteFile(hCom, &stream, 1, NULL, NULL);
            }
            CloseHandle(hCom);
            DestroyWindow(hwnd);
        case WM_COMMAND:
        {
            if (wParam == ZERO_OFFSET_BTN_ID)
            {
                const uint8_t data_to_send[5] = { (uint8_t)ZERO_POSITION_OFFSET, 0U, 0U, 0U, 0U };

                WriteFile(hCom, &data_to_send, 5, NULL, NULL);
            }
            else if (wParam == COM_CONNECT_BTN_ID)
            {
                if(!com_currently_connected)
                {
                    DCB dcb;

                    char com_port[10] = "";
                    GetWindowText(com_port_edit, com_port, 10);

                    char com_baud_rate[10] = "";
                    GetWindowText(com_port_baud_rate, com_baud_rate, 10);

                    int baud_rate = atoi(com_baud_rate);

                    hCom = CreateFile( com_port,
                                    GENERIC_READ | GENERIC_WRITE,
                                    0,      //  must be opened with exclusive-access
                                    NULL,   //  default security attributes
                                    OPEN_EXISTING, //  must use OPEN_EXISTING
                                    0,      //  not overlapped I/O
                                    NULL ); //  hTemplate must be NULL for comm devices

                    if (hCom == INVALID_HANDLE_VALUE) 
                    {
                        char text[50] = "";
                        snprintf(text, 50, "Connecting to COM %s failed with error: %d.\n", com_port, GetLastError());
                        SetWindowText(info_message, text);
                    }
                    else
                    {
                        GetCommState(hCom, &dcb);

                        dcb.BaudRate = baud_rate;     //  baud rate
                        dcb.ByteSize = 8;             //  data size, xmit and rcv
                        dcb.Parity   = NOPARITY;      //  parity bit
                        dcb.StopBits = ONESTOPBIT;    //  stop bit

                        SetCommState(hCom, &dcb);

                        // command to stop streaming, in case a stream command has previously persisted on the device
                        const uint8_t data_to_send[5] = { (uint8_t)STOP_STREAMING_DATA, 0U, 0U, 0U, 0U };
                        WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                        PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXCLEAR);

                        SetWindowText(com_connect_btn, "DISCONNECT");
                        SetWindowText(info_message, "COM connection established");

                        EnableWindow(streaming_btn, true);
                        EnableWindow(zero_offset_btn, true);
                        EnableWindow(direction_btn, true);
                        EnableWindow(overrides_toggle_btn, true);
                        EnableWindow(motor_enable_toggle_btn, true);
                        EnableWindow(reset_motor_btn, true);
                        EnableWindow(torque_cmd_btn, true);
                        EnableWindow(duty_override_btn, true);
                        EnableWindow(trigger_fast_logging_btn, true);

                        com_currently_connected = true;
                    }
                }
                else
                {
                    const uint8_t data_to_send[5] = { (uint8_t)STOP_STREAMING_DATA, 0U, 0U, 0U, 0U };
                    WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                    SetWindowText(streaming_btn, "START STREAMING");
                    SetWindowText(info_message, "Stopped streaming");

                    currently_streaming = false;

                    EnableWindow(streaming_btn, false);
                    EnableWindow(zero_offset_btn, false);
                    EnableWindow(direction_btn, false);
                    EnableWindow(overrides_toggle_btn, false);
                    EnableWindow(motor_enable_toggle_btn, false);
                    EnableWindow(reset_motor_btn, false);
                    EnableWindow(torque_cmd_btn, false);
                    EnableWindow(duty_override_btn, false);
                    EnableWindow(trigger_fast_logging_btn, false);

                    CloseHandle(hCom);
                    SetWindowText(com_connect_btn, "CONNECT");
                    SetWindowText(info_message, "COM disconnected");
                    com_currently_connected = false;
                }
            }
            else if (wParam == STREAMING_BTN_ID)
            {
                if(!currently_streaming)
                {
                    const uint8_t data_to_send[5] = { (uint8_t)START_STREAMING_DATA, 0U, 0U, 0U, 0U };
                    WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                    SetWindowText(streaming_btn, "STOP STREAMING");
                    SetWindowText(info_message, "Started streaming");

                    currently_streaming = true;
                }
                else
                {
                    const uint8_t data_to_send[5] = { (uint8_t)STOP_STREAMING_DATA, 0U, 0U, 0U, 0U };
                    WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                    SetWindowText(streaming_btn, "START STREAMING");
                    SetWindowText(info_message, "Stopped streaming");

                    currently_streaming = false;
                }
            }
            else if (wParam == DUTY_OVERRIDE_BTN_ID)
            {
                char string[10] = "";
                GetWindowText(duty_ratio_override_edit, string, 10);
                union uint_to_float_U float_data = { .value = atof(string) };

                const uint8_t data_to_send[5] = { (uint8_t)DUTY_RATIO_OVERRIDE, float_data.raw[0], float_data.raw[1], float_data.raw[2], float_data.raw[3] };
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                SetWindowText(info_message, "Set duty ratio override");
            }
            else if (wParam == TORQUE_CMD_BTN_ID)
            {
                char string[10] = "";
                GetWindowText(torque_cmd_edit, string, 10);
                union uint_to_float_U float_data = { .value = atof(string) };

                const uint8_t data_to_send[5] = { (uint8_t)TORQUE_CMD_OVERRIDE, float_data.raw[0], float_data.raw[1], float_data.raw[2], float_data.raw[3] };
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                SetWindowText(info_message, "Set torque command override");
            }
            else if (wParam == DIRECTION_TOGGLE_BTN_ID)
            {
                direction_override_val = !direction_override_val;
                const uint8_t data_to_send[5] = { (uint8_t)DIRECTION_TOGGLE, direction_override_val, 0U, 0U, 0U};
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                char text[50] = "";
                snprintf(text, 50, "Motor direction changed to %d", direction_override_val);

                SetWindowText(info_message, text);
            }
            else if (wParam == OVERRIDES_TOGGLE_BTN_ID)
            {
                override_enable_val = !override_enable_val;
                const uint8_t data_to_send[5] = { (uint8_t)OVERRIDE_TOGGLE, override_enable_val, 0U, 0U, 0U};
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                if(override_enable_val)
                {
                    SetWindowText(overrides_toggle_btn, "DISABLE OVERRIDES");
                    SetWindowText(info_message, "Overrides enabled");
                }
                else
                {
                    SetWindowText(overrides_toggle_btn, "ENABLE OVERRIDES");
                    SetWindowText(info_message, "Overrides disabled");
                }
            }
            else if (wParam == MOTOR_ENABLE_TOGGLE_BTN_ID)
            {
                motor_enable_val = !motor_enable_val;
                const uint8_t data_to_send[5] = { (uint8_t)MOTOR_ENABLE_TOGGLE, motor_enable_val, 0U, 0U, 0U};
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                if(motor_enable_val)
                {
                    SetWindowText(motor_enable_toggle_btn, "DISABLE MOTOR");
                    SetWindowText(info_message, "Motor enabled");
                }
                else
                {
                    SetWindowText(motor_enable_toggle_btn, "ENABLE MOTOR");
                    SetWindowText(info_message, "Motor disabled");
                }
            }
            else if (wParam == RESET_MOTOR_BTN_ID)
            {
                const uint8_t data_to_send[5] = { (uint8_t)RESET_MOTOR, 0U, 0U, 0U, 0U};
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                SetWindowText(info_message, "Reset motor");
            }
            else if (wParam == TRIGGER_FAST_LOGGING_BTN_ID)
            {
                const uint8_t data_to_send[5] = { (uint8_t)TRIGGER_FAST_LOGGING, 0U, 0U, 0U, 0U};
                WriteFile(hCom, &data_to_send, 5, NULL, NULL);

                fast_logging_triggered = true;

                SetWindowText(info_message, "Fast logging triggered");
            }
            break;
        }
        case WM_CTLCOLORSTATIC:
        {
            HDC hdcStatic = (HDC) wParam;
            SetTextColor(hdcStatic, RGB(0,0,0));
            SetBkMode(hdcStatic, TRANSPARENT);

            if(hbrBkgnd_green == NULL)
            {
                hbrBkgnd_green = CreateSolidBrush(RGB(200,255,200));
            }
            if(hbrBkgnd_red == NULL)
            {
                hbrBkgnd_red = CreateSolidBrush(RGB(255,200,200));
            }
            if(hbrBkgnd_regular == NULL)
            {
                hbrBkgnd_regular = CreateSolidBrush(RGB(220,220,220));
            }

            if(lParam == (LPARAM)measurement_label[0])
            {
                if(meas1_within_bounds)
                {
                    return (INT_PTR)hbrBkgnd_green;
                }
                else
                {
                    return (INT_PTR)hbrBkgnd_regular;
                }
            }
            else if (lParam == (LPARAM)measurement_label[1])
            {
                if(meas2_within_bounds)
                {
                    return (INT_PTR)hbrBkgnd_green; 
                }
                else
                {
                    return (INT_PTR)hbrBkgnd_regular;
                }
            }
            else if (lParam == (LPARAM)measurement_label[2])
            {
                if(meas3_within_bounds)
                {
                    return (INT_PTR)hbrBkgnd_green; 
                }
                else
                {
                    return (INT_PTR)hbrBkgnd_regular;
                }
            }
            else if (lParam == (LPARAM)rls_fault_bitfield_label)
            {
                if(rls_fault_flag)
                {
                    return (INT_PTR)hbrBkgnd_red; 
                }
                else
                {
                    return (INT_PTR)hbrBkgnd_regular;
                }
            }
            else if (lParam == (LPARAM)motor_fault_flag_label)
            {
                if(motor_fault_flag)
                {
                    return (INT_PTR)hbrBkgnd_red; 
                }
                else
                {
                    return (INT_PTR)hbrBkgnd_regular;
                }
            }
			else if (lParam == (LPARAM)controller_state_label)
            {
                if(controller_active)
                {
                    return (INT_PTR)hbrBkgnd_green; 
                }
                else
                {
                    return (INT_PTR)hbrBkgnd_regular;
                }
            }
            else
            {
                return (INT_PTR)hbrBkgnd_regular;
            }
            break;
        }
        case WM_DESTROY:
            PostQuitMessage(0);
        break;
        default:
            return DefWindowProc(hwnd, msg, wParam, lParam);
    }
    return 0;
}

DWORD WINAPI c2000_receive( LPVOID lpParam ) 
{
    for(;;)
    {
        if (currently_streaming || fast_logging_triggered)
        {
            char id;
            ReadFile(hCom, &id, 1, NULL, NULL);

            switch(id)
            {
                case X_HAT_0:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[0], text);

                    break;
                }
                case X_HAT_1:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[1], text);

                    break;
                }
                case X_HAT_2:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[2], text);

                    break;
                }
                case X_HAT_3:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[3], text);

                    break;
                }
                case X_HAT_4:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[4], text);

                    break;
                }
                case X_HAT_5:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[5], text);

                    break;
                }
                case TORQUE_CMD:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(torque_cmd_label, text);

                    break;
                }
                case MEASUREMENTS_0:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[0], text);

                    meas1_within_bounds = fabsf(data.value) < DEG2RAD(ANGLES_WITHIN_BOUNDS_DEG) ? true : false;

                    break;
                }
                case MEASUREMENTS_1:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[1], text);

                    meas2_within_bounds = fabsf(data.value) < DEG2RAD(ANGLES_WITHIN_BOUNDS_DEG) ? true : false;

                    break;
                }
                case MEASUREMENTS_2:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[2], text);

                    meas3_within_bounds = fabsf(data.value) < DEG2RAD(ANGLES_WITHIN_BOUNDS_DEG) ? true : false;

                    break;
                }
                case RLS_ERROR_BITFIELD:
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    rls_fault_flag = data > 0U ? true : false;

                    SetWindowText(rls_fault_bitfield_label, text);

                    break;
                }
                case MOTOR_FAULT_FLAG:
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    motor_fault_flag = (data == 1U) ? true : false;

                    SetWindowText(motor_fault_flag_label, text);

                    break;
                }
				case CONTROLLER_STATE:
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    controller_active = (data == 3U) ? true : false;

                    SetWindowText(controller_state_label, text);

                    break;
                }
				case CURRENT_FB:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(current_fb_label, text);

                    break;
                }
				case V_BRIDGE:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(v_bridge_label, text);

                    break;
                }
				case DUTY_RATIO:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(duty_ratio_label, text);

                    break;
                }
                case FAST_LOGGING_SIGNALS_READY:
                {
                    fast_logging_index = 0U;
                    break;
                }
                case FAST_LOGGING_SIGNAL1:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    fast_logging_signals[0][fast_logging_index] = data.value;

                    break;
                }
                case FAST_LOGGING_SIGNAL2:
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    fast_logging_signals[1][fast_logging_index] = data.value;

                    char text[10] = "";
                    snprintf(text, 10, "%d", fast_logging_index);

                    SetWindowText(buffer_label, text);

                    fast_logging_index++;

                    break;
                }
                case FAST_LOGGING_SIGNALS_DONE:
                {
                    fast_logging_index = 0U;

                    fast_logging_triggered = false;

                    SetWindowText(info_message, "Writing logs to file");

                    write_signals_to_file();

                    break;
                }
				default:
					break;
            }
        }    
    }
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
    LPSTR lpCmdLine, int nCmdShow)
{
    WNDCLASSEX wc;
    MSG Msg;

    const char g_szClassName[] = "SCROLL";

    //Step 1: Registering the Window Class
    wc.cbSize        = sizeof(WNDCLASSEX);
    wc.style         = 0;
    wc.lpfnWndProc   = WndProc;
    wc.cbClsExtra    = 0;
    wc.cbWndExtra    = 0;
    wc.hInstance     = hInstance;
    wc.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
    wc.lpszMenuName  = NULL;
    wc.lpszClassName = g_szClassName;
    wc.hIconSm       = LoadIcon(NULL, IDI_APPLICATION);

    if(!RegisterClassEx(&wc))
    {
        MessageBox(NULL, "Window Registration Failed!", "Error!",
            MB_ICONEXCLAMATION | MB_OK);
        return 0;
    }

    // Step 2: Creating the Window
    hwnd = CreateWindowEx(
        WS_EX_CLIENTEDGE,
        g_szClassName,
        "Furuta Interface GUI",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 820, 590,
        NULL, NULL, hInstance, NULL);

    if(hwnd == NULL)
    {
        MessageBox(NULL, "Window Creation Failed!", "Error!",
            MB_ICONEXCLAMATION | MB_OK);
        return 0;
    }

    int y_position = 10;

    CreateWindow("STATIC", "COM Port:", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    com_port_edit = CreateWindow("EDIT", "COM7", WS_VISIBLE | WS_CHILD, 100, y_position, 60, 20, hwnd, NULL, NULL, NULL);
    com_connect_btn = CreateWindow("BUTTON", "CONNECT", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 220, 10, 200, 30, hwnd, (HMENU) COM_CONNECT_BTN_ID, NULL, NULL);

    y_position += 20;

    CreateWindow("STATIC", "COM Baud:", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    com_port_baud_rate = CreateWindow("EDIT", "115200", WS_VISIBLE | WS_CHILD, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 10;

    for(int i = 0; i < 6; i++)
    {
        y_position += 20;
        char text[20] = "";
        snprintf(text, 20, "x_hat[%d]", i);
        CreateWindow("STATIC", text, WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);

        x_hat_label[i] = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);
    }

    y_position += 30;

    CreateWindow("STATIC", "Torque Cmd", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    torque_cmd_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 10;

    for(int i = 0; i < 3; i++)
    {
        y_position += 20;
        char text[20] = "";
        snprintf(text, 20, "meas[%d]", i);
        CreateWindow("STATIC", text, WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);

        measurement_label[i] = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);
    }

    y_position += 30;

    CreateWindow("STATIC", "RLS Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    rls_fault_bitfield_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 20;

    CreateWindow("STATIC", "Motor Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    motor_fault_flag_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);
	
	y_position += 30;

    CreateWindow("STATIC", "Cntrl State", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    controller_state_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 30;

    CreateWindow("STATIC", "Current Fb", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    current_fb_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 20;

    CreateWindow("STATIC", "Bridge Volt", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    v_bridge_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 20;

    CreateWindow("STATIC", "Duty Ratio", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    duty_ratio_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 40;

    streaming_btn = CreateWindow("BUTTON", "START STREAMING", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 10, y_position, 200, 30, hwnd, (HMENU) STREAMING_BTN_ID, NULL, NULL);
    EnableWindow(streaming_btn, false);

    y_position += 50;

    info_message = CreateWindow("STATIC", "", WS_VISIBLE | WS_CHILD, 10, y_position, 780, 30, hwnd, NULL, NULL, NULL);


    // Right side stuff
    zero_offset_btn = CreateWindow("BUTTON", "SET ZERO OFFSET", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, 10, 200, 30, hwnd, (HMENU) ZERO_OFFSET_BTN_ID, NULL, NULL);
    EnableWindow(zero_offset_btn, false);

    y_position = 70;

    duty_ratio_override_edit = CreateWindow("EDIT", "0.0", WS_VISIBLE | WS_CHILD, 500, y_position, 60, 20, hwnd, NULL, NULL, NULL);
    duty_override_btn = CreateWindow("BUTTON", "SET DUTY", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 570, y_position, 100, 20, hwnd, (HMENU) DUTY_OVERRIDE_BTN_ID, NULL, NULL);
    EnableWindow(duty_override_btn, false);

    y_position += 30;

    torque_cmd_edit = CreateWindow("EDIT", "0.0", WS_VISIBLE | WS_CHILD, 500, y_position, 60, 20, hwnd, NULL, NULL, NULL);
    torque_cmd_btn = CreateWindow("BUTTON", "SET TORQUE", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 570, y_position, 100, 20, hwnd, (HMENU) TORQUE_CMD_BTN_ID, NULL, NULL);
    EnableWindow(torque_cmd_btn, false);

    y_position += 30;

    direction_btn = CreateWindow("BUTTON", "CHANGE DIRECTION", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, y_position, 200, 20, hwnd, (HMENU) DIRECTION_TOGGLE_BTN_ID, NULL, NULL);
    EnableWindow(direction_btn, false);

    y_position += 30;

    overrides_toggle_btn = CreateWindow("BUTTON", "ENABLE OVERRIDES", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, y_position, 200, 20, hwnd, (HMENU) OVERRIDES_TOGGLE_BTN_ID, NULL, NULL);
    EnableWindow(overrides_toggle_btn, false);

    y_position += 30;

    motor_enable_toggle_btn = CreateWindow("BUTTON", "ENABLE MOTOR", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, y_position, 200, 20, hwnd, (HMENU) MOTOR_ENABLE_TOGGLE_BTN_ID, NULL, NULL);
    EnableWindow(motor_enable_toggle_btn, false);

    y_position += 30;

    reset_motor_btn = CreateWindow("BUTTON", "RESET MOTOR", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, y_position, 200, 20, hwnd, (HMENU) RESET_MOTOR_BTN_ID, NULL, NULL);
    EnableWindow(reset_motor_btn, false);

    // Logging trigger stuff
    y_position = 400;

    trigger_fast_logging_btn = CreateWindow("BUTTON", "TRIGGER LOGGING", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, y_position, 200, 20, hwnd, (HMENU) TRIGGER_FAST_LOGGING_BTN_ID, NULL, NULL);
    EnableWindow(trigger_fast_logging_btn, false);

    y_position += 30;

    CreateWindow("STATIC", "Recv Buffer", WS_VISIBLE | WS_CHILD, 500, y_position, 100, 20, hwnd, NULL, NULL, NULL);
    buffer_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD, 610, y_position, 90, 20, hwnd, NULL, NULL, NULL);
    
    // Thread stuff
    DWORD * hThread = CreateThread(NULL, 0, c2000_receive, NULL, 0, NULL);   // returns the thread identifier
    SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);

    if (hThread == NULL) 
    {
        ExitProcess(3);
    }

    ShowWindow(hwnd, nCmdShow);
    UpdateWindow(hwnd);

    // Step 3: The Message Loop
    while(GetMessage(&Msg, NULL, 0, 0) > 0)
    {
        TranslateMessage(&Msg);
        DispatchMessage(&Msg);
    }
    return Msg.wParam;
}