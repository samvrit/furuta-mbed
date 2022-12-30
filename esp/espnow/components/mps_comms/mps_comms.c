#include <math.h>

#include "mps_comms.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)

static const char *MPS_TAG = "MPS";

uint16_t mps_get_angle(spi_device_handle_t * handle)
{
    spi_transaction_t t = { 0 };

    t.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA);
    t.length = 16;
    t.tx_data[0] = 0x0;
    t.tx_data[1] = 0x0;

    spi_device_transmit(*handle, &t);

    const uint16_t angle_little_endian = (t.rx_data[0] << 6U) | (t.rx_data[1] >> 2);

    return angle_little_endian;
}

void mps_set_zero_position_to_current_position(spi_device_handle_t * handle)
{
    spi_transaction_t t = { 0 };

    t.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA);
    t.length = 16;

    // Reset the zero position
    t.tx_data[0] = 0x80U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(25 / portTICK_PERIOD_MS);

    // acknowledge 
    t.tx_data[0] = 0x0U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(1 / portTICK_PERIOD_MS);

    t.tx_data[0] = 0x81U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(25 / portTICK_PERIOD_MS);

    // acknowledge 
    t.tx_data[0] = 0x0U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // read current angle
    const uint16_t current_angle = mps_get_angle(handle);

#if (CONFIG_TRANSMIT_DEVICE1)
    const float angle_to_set = fmodf(((float)current_angle + 8192.0f), 16384.0f);
#else
    const float angle_to_set = (float)current_angle;
#endif // (CONFIG_TRANSMIT_DEVICE1)
    const uint16_t angle_to_set_uint = 65535U - ( ( angle_to_set / 16384.0f ) * 65535U );

    ESP_LOGI(MPS_TAG, "angle: %u, angle_to_set: %u\n", current_angle, angle_to_set_uint);

    const uint8_t angle_to_set_bits_0_7 = (angle_to_set_uint & 0xFFU);
    const uint8_t angle_to_set_bits_8_15 = (angle_to_set_uint >> 8U);

    t.tx_data[0] = 0x80U;
    t.tx_data[1] = angle_to_set_bits_0_7;
    spi_device_transmit(*handle, &t);

    vTaskDelay(25 / portTICK_PERIOD_MS);

    // acknowledge 
    t.tx_data[0] = 0x0U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(1 / portTICK_PERIOD_MS);

    t.tx_data[0] = 0x81U;
    t.tx_data[1] = angle_to_set_bits_8_15;
    spi_device_transmit(*handle, &t);

    vTaskDelay(25 / portTICK_PERIOD_MS);

    // acknowledge 
    t.tx_data[0] = 0x0U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(1 / portTICK_PERIOD_MS);
}

void mps_set_ccw_as_incrementing(spi_device_handle_t * handle)
{
    spi_transaction_t t = { 0 };

    t.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA);
    t.length = 16;

    t.tx_data[0] = 0x89U;
    t.tx_data[1] = 0x80U;
    spi_device_transmit(*handle, &t);

    vTaskDelay(25 / portTICK_PERIOD_MS);

    // acknowledge 
    t.tx_data[0] = 0x0U;
    t.tx_data[1] = 0x0U;
    spi_device_transmit(*handle, &t);
}

#endif // (CONFIG_TRANSMIT_DEVICE1 || CONFIG_TRANSMIT_DEVICE2)
