import serial
import struct
import time

def serial_send():
    torqueCommands = [0.01, 0.02, 0.03, 0.04, 0.05, -0.01, -0.03, -0.05, 0.02]
    timeSteps = [5, 5, 5, 5, 5, 5, 5, 5, 5]
    startTime = time.time()
    i = 0
    with serial.Serial('COM16', 115200, timeout=0) as ser:
        while(True):
            line = ser.readline()
            if line:
                print(line.decode('utf-8'))
            currentTime = time.time()
            try:
                if round((currentTime - startTime), 1) == timeSteps[i]:
                    print(f'Sending index: {i}, torque: {torqueCommands[i]}')
                    ba = bytes(struct.pack("f", torqueCommands[i]))
                    ba = ba + b'\r'
                    ser.write(ba)
                    startTime = currentTime
                    i = i + 1
            except IndexError:
                i = 0
                    
if __name__ == '__main__':
    serial_send()