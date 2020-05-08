#include "mbed.h"

class AMT203
{
public:
    AMT203(SPI *spi_handle, DigitalOut *cs);
    ~AMT203(){};

    uint16_t get_position_raw(void);
    float get_position_minus_pi_to_plus_pi(void);

    void set_zero(void);
    uint8_t SPIWrite(uint8_t sendByte);

private:
    SPI *_spi_handle;
    DigitalOut *_cs;
    uint8_t _timeoutCounter = 0;
    int16_t _currentPosition = 0;
    void _init(void);
};