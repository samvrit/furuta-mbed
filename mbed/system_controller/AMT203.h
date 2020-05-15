#include "mbed.h"

class AMT203
{
public:
    AMT203(SPI *spi_handle, DigitalOut *cs, uint32_t frequency = 1000000L);
    ~AMT203(){};

    uint16_t get_position_raw(void);
    float get_position_minus_pi_to_plus_pi(void);

    void set_zero(void);
    uint8_t SPIWrite(uint8_t sendByte);

private:
    SPI *_spi_handle;
    DigitalOut *_cs;
    uint32_t _frequency = 1000000;
    uint8_t _timeoutCounter = 0;
    int16_t _currentPosition = 0;
    void _init(void);
};