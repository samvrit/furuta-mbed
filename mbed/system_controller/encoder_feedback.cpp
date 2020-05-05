#include "encoder_feedback.h"

#define nop                     0x00    //no operation
#define rd_pos                  0x10    //read position
#define set_zero_point          0x70    //set zero point
#define timoutLimit             100
#define PI                      3.1415F
#define ANGLE_SCALING_FACTOR    (PI / 2048U)

static SPI encoder(D11, D12, D13); // MOSI, MISO, CLK
DigitalOut cs(D10); // chip select for encoder

void encoder_init(void)
{
    encoder.frequency(500000);
    encoder.format(8, 0, 0);    // 8-bit frame, SPI mode 0, and MSB first bit order
    encoder.clear_transfer_buffer();    // clear transfer buffer

    cs.write(1);    // set default CS pin to be high
}

void set_zero(void)
{
    uint8_t timeoutCounter = 0;     // timeout incrementer

    uint8_t data = SPIWrite(set_zero_point);

    while (data != 0x80 && timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);
    }
}

float get_position(void)
{
    uint8_t timeoutCounter = 0;     //our timeout incrementer

    uint16_t currentPosition = 0;   //this 16 bit variable will hold our 12-bit position

    uint8_t data = SPIWrite(rd_pos);
    
    while (data != rd_pos && timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);   // keep writing nop until command is echoed back, indicating the presence of the new position data
    }

    if (timeoutCounter < timoutLimit)   //rd_pos echo received
    {
        currentPosition = (SPIWrite(nop) & 0x0F) << 8;  // MSB is the first 4 bits of the response
        currentPosition |= SPIWrite(nop);               // Writing nop again will get the LSP
    }
    else
    {
        // handle timeout case
    }

    currentPosition = currentPosition > 2048 ? (currentPosition - 4096) : currentPosition;  // convert range from [0, 4096] to [-2048, 2048]

    return (float)(currentPosition * ANGLE_SCALING_FACTOR); // convert to radians

}

uint8_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;

  //the AMT20 requires the release of the CS line after each byte
  cs.write(0);
  data = encoder.write(sendByte);
  cs.write(1);

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  wait_us(10);
  
  return data;
}