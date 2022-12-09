/**
 * @file radio_utils.cpp
 * @author Pierre-Marie ANCELE (pm.ancele@catie.fr)
 * @brief
 * @version 0.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "radio_utils.h"

uint32_t float_to_uint(float n)
{
   return (uint32_t)(*(uint32_t*)&n);
}

float uint_to_float(uint32_t n)
{
   return (float)(*(float*)&n);
}

void frame_encoder(com_packet_t *packet)
{
	size_t i = 0;
	uint32_t convert = 0;

	packet->raw[i] = static_cast<uint8_t>(packet->type); i++;
	packet->raw[i] = static_cast<uint8_t>(packet->total_length); i++;

	switch(packet->type) {
		case PACKET_MASTER:
			packet->raw[i] = packet->master.actions; i++;
			packet->raw[i] = (packet->master.x_speed >> 8); i++;
			packet->raw[i] = packet->master.x_speed; i++;
			packet->raw[i] = (packet->master.y_speed >> 8); i++;
			packet->raw[i] = packet->master.y_speed; i++;
			packet->raw[i] = (packet->master.t_speed >> 8); i++;
			packet->raw[i] = packet->master.t_speed; i++;
			packet->raw[i] = packet->master.kickPower; i++;
			break;
		case PACKET_PARAMS:
			convert = float_to_uint(packet->params.kp);
			packet->raw[i] = convert >> 24; i++;
			packet->raw[i] = convert >> 16; i++;
			packet->raw[i] = convert >> 8; i++;
			packet->raw[i] = convert; i++;
			convert = float_to_uint(packet->params.ki);
			packet->raw[i] = convert >> 24; i++;
			packet->raw[i] = convert >> 16; i++;
			packet->raw[i] = convert >> 8; i++;
			packet->raw[i] = convert; i++;
			convert = float_to_uint(packet->params.kd);
			packet->raw[i] = convert >> 24; i++;
			packet->raw[i] = convert >> 16; i++;
			packet->raw[i] = convert >> 8; i++;
			packet->raw[i] = convert; i++;
			break;
		case PACKET_ROBOT:
			packet->raw[i] = packet->robot.id; i++;
			packet->raw[i] = packet->robot.status; i++;
			packet->raw[i] = packet->robot.cap_volt; i++;
			packet->raw[i] = packet->robot.voltage; i++;
			break;
	}
}

void frame_parser(com_packet_t *packet)
{
    uint8_t index = 0;
    uint32_t conv = 0;
    uint32_t *ptr = NULL;

    packet->type = (packet_type_t)packet->raw[index];
    index++;
    packet->total_length = packet->raw[index];
    index++;

    switch (packet->type) {
        case PACKET_MASTER:
            packet->master.actions = packet->raw[index];
            index++;
            packet->master.x_speed = packet->master.x_speed | packet->raw[index] << 8;
            index++;
            packet->master.x_speed = packet->master.x_speed | packet->raw[index];
            index++;
            packet->master.y_speed = packet->master.y_speed | packet->raw[index] << 8;
            index++;
            packet->master.y_speed = packet->master.y_speed | packet->raw[index];
            index++;
            packet->master.t_speed = packet->master.t_speed | packet->raw[index] << 8;
            index++;
            packet->master.t_speed = packet->master.t_speed | packet->raw[index];
            index++;
            packet->master.kickPower = packet->raw[index];
            index++;
            break;
        case PACKET_PARAMS:
            // format kp param
            conv = conv | packet->raw[index] << 24;
            index++;
            conv = conv | packet->raw[index] << 16;
            index++;
            conv = conv | packet->raw[index] << 8;
            index++;
            conv = conv | packet->raw[index];
            index++;
            packet->params.kp = uint_to_float(conv);
            // format ki param
            conv = 0;
            conv = conv | packet->raw[index] << 24;
            index++;
            conv = conv | packet->raw[index] << 16;
            index++;
            conv = conv | packet->raw[index] << 8;
            index++;
            conv = conv | packet->raw[index];
            index++;
            packet->params.ki = uint_to_float(conv);
            // format kd param
            conv = 0;
            conv = conv | packet->raw[index] << 24;
            index++;
            conv = conv | packet->raw[index] << 16;
            index++;
            conv = conv | packet->raw[index] << 8;
            index++;
            conv = conv | packet->raw[index];
            index++;
            packet->params.kd = uint_to_float(conv);
            break;
        case PACKET_ROBOT:
            packet->robot.id = packet->raw[index];
            index++;
            packet->robot.status = packet->raw[index];
            index++;
            packet->robot.cap_volt = packet->raw[index];
            index++;
            packet->robot.voltage = packet->raw[index];
            index++;
            break;
    }
}
