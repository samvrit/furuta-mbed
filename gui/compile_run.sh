cp ../motor_controller_ti/motor_controller/furuta_controller_v2/device_comms/host_comms_shared.h ./include/

gcc -I ./include -o serial_gui serial_gui.c -l gdi32

./serial_gui.exe
