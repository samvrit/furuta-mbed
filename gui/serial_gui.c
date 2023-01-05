#include <windows.h>
#include <stdio.h>
#include <tchar.h>

#define ZERO_OFFSET_BTN_ID (1)
#define STREAMING_BTN_ID (2)

#define IDT_TIMER1 (1)

HWND hwnd;

HWND x_hat_label[6];
HWND measurement_label[3];
HWND rls_fault_bitfield;
HWND motor_fault_flag;
HWND streaming_btn;
HWND info_message;

HANDLE hCom;

// Step 4: the Window Procedure
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch(msg)
    {
        case WM_CLOSE:
            DestroyWindow(hwnd);
        case WM_COMMAND:
        {
            if (wParam == ZERO_OFFSET_BTN_ID)
            {
                printf("Button click\n");
            }
            else if (wParam == STREAMING_BTN_ID)
            {
                const char stream = 's';
                WriteFile(hCom, &stream, 1, NULL, NULL);

                SetWindowText(streaming_btn, "STOP STREAMING");
                SetWindowText(info_message, "Start streaming");
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

// VOID CALLBACK MyTimerProc(HWND hwnd, UINT message, UINT idTimer, DWORD dwTime)
// { 
//     static int i = 0;

//     i++;

//     char text[100] = "";

//     sprintf(text, "%d", i);

//     SetWindowText(x_hat_label[0], text);
// }

DWORD WINAPI MyThreadFunction( LPVOID lpParam ) 
{
    // SetTimer(hwnd, IDT_TIMER1, 1, (TIMERPROC) MyTimerProc);

    for(;;)
    {
        static int i = 0;

        i++;

        char text[100] = "";

        sprintf(text, "%d", i);

        SetWindowText(x_hat_label[0], text);
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

    int y_position = 50;

    for(int i = 0; i < 6; i++)
    {
        y_position += 50;
        char text[20] = "";
        snprintf(text, 20, "x_hat[%d]", i);
        CreateWindow("STATIC", text, WS_VISIBLE | WS_CHILD, 10, y_position, 60, 30, hwnd, NULL, NULL, NULL);

        x_hat_label[i] = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);
    }

    y_position += 50;

    for(int i = 0; i < 3; i++)
    {
        y_position += 50;
        char text[20] = "";
        snprintf(text, 20, "meas[%d]", i);
        CreateWindow("STATIC", text, WS_VISIBLE | WS_CHILD, 10, y_position, 60, 30, hwnd, NULL, NULL, NULL);

        measurement_label[i] = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);
    }

    y_position += 50;

    CreateWindow("STATIC", "RLS Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 60, 30, hwnd, NULL, NULL, NULL);
    rls_fault_bitfield = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);

    y_position += 50;

    CreateWindow("STATIC", "Motor Fault", WS_VISIBLE | WS_CHILD, 10, y_position, 60, 30, hwnd, NULL, NULL, NULL);
    motor_fault_flag = CreateWindow("STATIC", "0", WS_VISIBLE | WS_CHILD, 100, y_position, 100, 30, hwnd, NULL, NULL, NULL);

    y_position += 50;

    streaming_btn = CreateWindow("BUTTON", "START STREAMING", WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, 10, y_position, 200, 50, hwnd, (HMENU) STREAMING_BTN_ID, NULL, NULL);

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

    // COM port stuff

    DCB dcb;

    TCHAR *pcCommPort = TEXT("COM3");

    hCom = CreateFile( pcCommPort,
                      GENERIC_READ | GENERIC_WRITE,
                      0,      //  must be opened with exclusive-access
                      NULL,   //  default security attributes
                      OPEN_EXISTING, //  must use OPEN_EXISTING
                      0,      //  not overlapped I/O
                      NULL ); //  hTemplate must be NULL for comm devices

    if (hCom == INVALID_HANDLE_VALUE) 
    {
       //  Handle the error.
       printf ("CreateFile failed with error %d.\n", GetLastError());
       return (1);
    }

    GetCommState(hCom, &dcb);

    dcb.BaudRate = CBR_115200;     //  baud rate
    dcb.ByteSize = 8;             //  data size, xmit and rcv
    dcb.Parity   = NOPARITY;      //  parity bit
    dcb.StopBits = ONESTOPBIT;    //  stop bit

    SetCommState(hCom, &dcb);

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