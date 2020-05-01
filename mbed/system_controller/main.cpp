#include "mbed.h"
#include "acquire_feedback.h"
#include "rtos.h"
#include <string>

#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point
#define timoutLimit 100

Thread network_thread;

SPI spi(D11, D12, D13, D10);
CAN can1(PA_11, PA_12, 1000000);

Timer timer;
Ticker ticker;
int dt = 0;

typedef union {
    float value;
    char buffer[sizeof(float)];
} canPacket_t;

volatile canPacket_t canPacket;


uint8_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;

  //the AMT20 requires the release of the CS line after each byte
  spi.select();
  data = spi.write(sendByte);
  spi.deselect();

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  
  wait_us(50);
  
  return data;
}

int main()
{
    uint8_t data = 0;               //this will hold our returned data from the AMT20
    uint8_t timeoutCounter = 0;     //our timeout incrementer
    uint16_t currentPosition = 0;   //this 16 bit variable will hold our 12-bit position

    spi.frequency(500000);

    canPacket.value = -0.5;

    network_thread.start(callback(sensors_receive));

    ThisThread::sleep_for(2000);

    // data = SPIWrite(set_zero_point);

    // while (data != 0x80 && timeoutCounter++ < timoutLimit)
    // {
    //     data = SPIWrite(nop);
    // }

    printf("Sending signal: %02X %02X %02X %02X\r\n", canPacket.buffer[0], canPacket.buffer[1], canPacket.buffer[2], canPacket.buffer[3]);
    can1.write(CANMessage(0x1, (const char *)&canPacket.buffer, sizeof(float)));

    while (1)
    {
        timer.start();

        timeoutCounter = 0;
    
        data = SPIWrite(rd_pos);

        while (data != rd_pos && timeoutCounter++ < timoutLimit)
        {
            data = SPIWrite(nop);
        }

        if (timeoutCounter < timoutLimit) //rd_pos echo received
        {
            currentPosition = (SPIWrite(nop)& 0x0F) << 8;
        
            currentPosition |= SPIWrite(nop);

            printf("Current position: %u\r\n", currentPosition);
        }

        
        ThisThread::sleep_for(100);

        osEvent evt = queue.get(0);
        if (evt.status == osEventMessage)
        {
            timer.stop();
            dt = timer.read_us();
            timer.reset();
            udp_frame *frame = (udp_frame *)evt.value.p;
            printf("\n%d | Received: %d, %d, %d\r\n", dt, frame->pos_rad, frame->vel_sign, frame->vel_rad);

            mpool.free(frame);
        }
    }

    return 0;
}