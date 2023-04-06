/*
 * esp_comms.h
 *
 *  Created on: 18-Dec-2022
 *      Author: Samvrit
 */

#ifndef DEVICE_COMMS_ESP_COMMS_H_
#define DEVICE_COMMS_ESP_COMMS_H_

void esp_get_data(float * angle1, float * angle2, float * velocity1, float * velocity2, const float timestep);

#endif /* DEVICE_COMMS_ESP_COMMS_H_ */
