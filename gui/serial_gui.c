#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <tchar.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ZERO_OFFSET_BTN_ID (1)
#define STREAMING_BTN_ID (2)
#define COM_CONNECT_BTN_ID (3)

#define PI (3.1415f)
#define DEG2RAD(x) ((x) * (2.0f * PI / 360.0f ))
#define ANGLES_WITHIN_BOUNDS_DEG (2.0f)

HWND hwnd;

HWND com_port_edit;
HWND com_port_baud_rate;
HWND x_hat_label[6];
HWND torque_cmd_label;
HWND measurement_label[3];
HWND rls_fault_bitfield_label;
HWND motor_fault_flag_label;
HWND streaming_btn;
HWND com_connect_btn;
HWND zero_offset_btn;
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
                const char stream = 'c';
                WriteFile(hCom, &stream, 1, NULL, NULL);
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
                        const char stream = 't';
                        WriteFile(hCom, &stream, 1, NULL, NULL);

                        PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXCLEAR);

                        SetWindowText(com_connect_btn, "DISCONNECT");
                        SetWindowText(info_message, "COM connection established");

                        EnableWindow(streaming_btn, true);
                        EnableWindow(zero_offset_btn, true);

                        com_currently_connected = true;
                    }
                }
                else
                {
                    const char stream = 't';
                    WriteFile(hCom, &stream, 1, NULL, NULL);

                    SetWindowText(streaming_btn, "START STREAMING");
                    SetWindowText(info_message, "Stopped streaming");

                    currently_streaming = false;

                    EnableWindow(streaming_btn, false);
                    EnableWindow(zero_offset_btn, false);

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
                    const char stream = 's';
                    WriteFile(hCom, &stream, 1, NULL, NULL);

                    SetWindowText(streaming_btn, "STOP STREAMING");
                    SetWindowText(info_message, "Started streaming");

                    currently_streaming = true;
                }
                else
                {
                    const char stream = 't';
                    WriteFile(hCom, &stream, 1, NULL, NULL);

                    SetWindowText(streaming_btn, "START STREAMING");
                    SetWindowText(info_message, "Stopped streaming");

                    currently_streaming = false;
                }
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

DWORD WINAPI MyThreadFunction( LPVOID lpParam ) 
{
    for(;;)
    {
        if (currently_streaming)
        {
            char id;
            ReadFile(hCom, &id, 1, NULL, NULL);

            switch(id)
            {
                case 'a':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[0], text);

                    break;
                }
                case 'b':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[1], text);

                    break;
                }
                case 'c':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[2], text);

                    break;
                }
                case 'd':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[3], text);

                    break;
                }
                case 'e':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[4], text);

                    break;
                }
                case 'f':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(x_hat_label[5], text);

                    break;
                }
                case 'g':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[0], text);

                    meas1_within_bounds = fabsf(data.value) < DEG2RAD(ANGLES_WITHIN_BOUNDS_DEG) ? true : false;

                    break;
                }
                case 'h':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[1], text);

                    meas2_within_bounds = fabsf(data.value) < DEG2RAD(ANGLES_WITHIN_BOUNDS_DEG) ? true : false;

                    break;
                }
                case 'i':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[2], text);

                    meas3_within_bounds = fabsf(data.value) < DEG2RAD(ANGLES_WITHIN_BOUNDS_DEG) ? true : false;

                    break;
                }
                case 'j':
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    rls_fault_flag = data > 0U ? true : false;

                    SetWindowText(rls_fault_bitfield_label, text);

                    break;
                }
                case 'k':
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    motor_fault_flag = (data == 1U) ? true : false;

                    SetWindowText(motor_fault_flag_label, text);

                    break;
                }
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
        CW_USEDEFAULT, CW_USEDEFAULT, 820, 500,
        NULL, NULL, hInstance, NULL);

    if(hwnd == NULL)
    {
        MessageBox(NULL, "Window Creation Failed!", "Error!",
            MB_ICONEXCLAMATION | MB_OK);
        return 0;
    }

    int y_position = 10;

    CreateWindow("STATIC", "COM Port:", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    com_port_edit = CreateWindow("EDIT", "COM3", WS_VISIBLE | WS_CHILD, 100, y_position, 60, 20, hwnd, NULL, NULL, NULL);
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

    y_position += 30;

    CreateWindow("STATIC", "Motor Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 20, hwnd, NULL, NULL, NULL);
    motor_fault_flag_label = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 20, hwnd, NULL, NULL, NULL);

    y_position += 40;

    streaming_btn = CreateWindow("BUTTON", "START STREAMING", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 10, y_position, 200, 30, hwnd, (HMENU) STREAMING_BTN_ID, NULL, NULL);
    EnableWindow(streaming_btn, false);

    y_position += 50;

    info_message = CreateWindow("STATIC", "", WS_VISIBLE | WS_CHILD, 10, y_position, 780, 30, hwnd, NULL, NULL, NULL);


    // Right side stuff
    zero_offset_btn = CreateWindow("BUTTON", "SET ZERO OFFSET", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 500, 10, 200, 30, hwnd, (HMENU) ZERO_OFFSET_BTN_ID, NULL, NULL);
    EnableWindow(zero_offset_btn, false);

    // Thread stuff
    DWORD * hThread = CreateThread(NULL, 0, MyThreadFunction, NULL, 0, NULL);   // returns the thread identifier 

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