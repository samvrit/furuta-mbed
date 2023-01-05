#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ZERO_OFFSET_BTN_ID (1)
#define STREAMING_BTN_ID (2)
#define COM_CONNECT_BTN_ID (3)

#define IDT_TIMER1 (1)

HWND hwnd;

HWND com_port_edit;
HWND com_port_baud_rate;
HWND x_hat_label[6];
HWND measurement_label[3];
HWND rls_fault_bitfield;
HWND motor_fault_flag;
HWND streaming_btn;
HWND com_connect_btn;
HWND info_message;

HANDLE hCom;

// Types

union uint_to_float_U
{
    float value;
    uint8_t raw[4];
};

// Global variable declarations
bool currently_streaming = false;
bool com_currently_connected = false;

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

                        SetWindowText(com_connect_btn, "DISCONNECT");
                        SetWindowText(info_message, "COM connection established");

                        EnableWindow(streaming_btn, true);

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
        }
        break;
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

                    break;
                }
                case 'h':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[1], text);

                    break;
                }
                case 'i':
                {
                    union uint_to_float_U data = { .value = 0.0f };
                    ReadFile(hCom, data.raw, 4, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%.4f", data.value);

                    SetWindowText(measurement_label[2], text);

                    break;
                }
                case 'j':
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    SetWindowText(rls_fault_bitfield, text);

                    break;
                }
                case 'k':
                {
                    uint8_t data = 0U;
                    ReadFile(hCom, &data, 1, NULL, NULL);

                    char text[10] = "";
                    snprintf(text, 10, "%u", data);

                    SetWindowText(motor_fault_flag, text);

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

    const char g_szClassName[] = "myWindowClass";

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
        CW_USEDEFAULT, CW_USEDEFAULT, 1000, 1000,
        NULL, NULL, hInstance, NULL);

    if(hwnd == NULL)
    {
        MessageBox(NULL, "Window Creation Failed!", "Error!",
            MB_ICONEXCLAMATION | MB_OK);
        return 0;
    }

    int y_position = 10;

    CreateWindow("STATIC", "COM Port:", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 30, hwnd, NULL, NULL, NULL);
    com_port_edit = CreateWindow("EDIT", "COM3", WS_VISIBLE | WS_CHILD, 100, y_position, 60, 30, hwnd, NULL, NULL, NULL);
    com_connect_btn = CreateWindow("BUTTON", "CONNECT", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 200, 10, 200, 50, hwnd, (HMENU) COM_CONNECT_BTN_ID, NULL, NULL);

    y_position += 40;

    CreateWindow("STATIC", "COM Baud:", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 30, hwnd, NULL, NULL, NULL);
    com_port_baud_rate = CreateWindow("EDIT", "115200", WS_VISIBLE | WS_CHILD, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);


    for(int i = 0; i < 6; i++)
    {
        y_position += 50;
        char text[20] = "";
        snprintf(text, 20, "x_hat[%d]", i);
        CreateWindow("STATIC", text, WS_VISIBLE | WS_CHILD, 10, y_position, 80, 30, hwnd, NULL, NULL, NULL);

        x_hat_label[i] = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);
    }

    y_position += 50;

    for(int i = 0; i < 3; i++)
    {
        y_position += 50;
        char text[20] = "";
        snprintf(text, 20, "meas[%d]", i);
        CreateWindow("STATIC", text, WS_VISIBLE | WS_CHILD, 10, y_position, 80, 30, hwnd, NULL, NULL, NULL);

        measurement_label[i] = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);
    }

    y_position += 50;

    CreateWindow("STATIC", "RLS Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 30, hwnd, NULL, NULL, NULL);
    rls_fault_bitfield = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);

    y_position += 50;

    CreateWindow("STATIC", "Motor Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 80, 30, hwnd, NULL, NULL, NULL);
    motor_fault_flag = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD | TA_RIGHT, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);

    y_position += 50;

    streaming_btn = CreateWindow("BUTTON", "START STREAMING", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 10, y_position, 200, 50, hwnd, (HMENU) STREAMING_BTN_ID, NULL, NULL);
    EnableWindow(streaming_btn, false);

    y_position += 100;

    info_message = CreateWindow("STATIC", "", WS_VISIBLE | WS_CHILD, 10, y_position, 800, 30, hwnd, NULL, NULL, NULL);


    // Right side stuff
    HWND zero_offset_btn = CreateWindow("BUTTON", "SET ZERO OFFSET", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 700, 10, 200, 50, hwnd, (HMENU) ZERO_OFFSET_BTN_ID, NULL, NULL);

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