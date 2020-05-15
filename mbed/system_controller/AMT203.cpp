#include "AMT203.h"

#define nop                     0x00    //no operation
#define rd_pos                  0x10    //read position
#define set_zero_point          0x70    //set zero point
#define timoutLimit             100
#define PI                      3.1415F
#define ANGLE_SCALING_FACTOR    (PI / 2048U)

AMT203::AMT203(SPI *spi_handle, DigitalOut *cs, uint32_t frequncy): _spi_handle(spi_handle), _cs(cs), _frequency(frequncy)
{
    _init();
}

uint16_t AMT203::get_position_raw(void)
{
    _timeoutCounter = 0;     //our timeout incrementer

    uint8_t data = SPIWrite(rd_pos);
    
    while (data != rd_pos && _timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);   // keep writing nop until command is echoed back, indicating the presence of the new position data
    }

    if (_timeoutCounter < timoutLimit)   //rd_pos echo received
    {
        _currentPosition = (SPIWrite(nop) & 0x0F) << 8;  // MSB is the first 4 bits of the response
        _currentPosition |= SPIWrite(nop);               // Writing nop again will get the LSP
    }
    else
    {
        // handle timeout case
    }

    return _currentPosition;
}


float AMT203::get_position_minus_pi_to_plus_pi(void)
{
    _timeoutCounter = 0;     //our timeout incrementer

    uint8_t data = SPIWrite(rd_pos);
    
    while (data != rd_pos && _timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);   // keep writing nop until command is echoed back, indicating the presence of the new position data
    }

    if (_timeoutCounter < timoutLimit)   //rd_pos echo received
    {
        _currentPosition = (SPIWrite(nop) & 0x0F) << 8;  // MSB is the first 4 bits of the response
        _currentPosition |= SPIWrite(nop);               // Writing nop again will get the LSP
    }
    else
    {
        // handle timeout case
    }

    _currentPosition = _currentPosition > 2048 ? (_currentPosition - 4096) : _currentPosition;  // convert range from [0, 4096] to [-2048, 2048]

    return (float)(_currentPosition * ANGLE_SCALING_FACTOR); // convert to radians
}

void AMT203::_init(void)
{
    _spi_handle->frequency(_frequency);
    _spi_handle->format(8, 0);
    _spi_handle->clear_transfer_buffer();
    _cs->write(1);
}


void AMT203::set_zero(void)
{
    _timeoutCounter = 0;     // timeout incrementer

    uint8_t data = SPIWrite(set_zero_point);

    while (data != 0x80 && _timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);
    }
}

uint8_t AMT203::SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;

  //the AMT20 requires the release of the CS line after each byte
  _cs->write(0);
  data = _spi_handle->write(sendByte);
  _cs->write(1);

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  wait_us(10);
  
  return data;
}