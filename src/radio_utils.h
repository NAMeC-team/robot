/**
 * @file radio_utils.h
 * @author Pierre-Marie ANCELE (pm.ancele@catie.fr)
 * @brief
 * @version 0.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef RADIO_UTILS_H
#define RADIO_UTILS_H

#include "mbed.h"

#define PACKET_SIZE 16

#define ACTION_ON                                                                                  \
    (1 << 0) // The robot should be on (else everything is stopped)
             // Kicking, transition from 0 to 1 trigger kick if IR is present
#define ACTION_KICK1 (1 << 1) // Kick on kicker 1 - chip
#define ACTION_KICK2 (1 << 2) // Kick on kicker 2 - normal
#define ACTION_DRIBBLE (1 << 3) // Enable/disable the dribbler
#define ACTION_CHARGE (1 << 5) // Enable/disable the capacitor charge

#define STATUS_OK (1 << 0) // The robot is alive and ok
#define STATUS_DRIVER_ERR (1 << 1) // Error with drivers
#define STATUS_IR (1 << 2) // The infrared barrier detects the ball

typedef enum {
    PACKET_ROBOT = 1,
    PACKET_MASTER,
    PACKET_PARAMS
} packet_type_t;

// TODO: to be define frame format
typedef struct {
    uint8_t raw[PACKET_SIZE];
    packet_type_t type;
    uint8_t total_length;

    struct {
        uint8_t actions;
        int16_t x_speed; // Kinematic orders [mm/s]
        int16_t y_speed;
        int16_t t_speed; // Rotation in [mrad/s]
        uint8_t kickPower; // Kick power (this is a duration in [x25 uS])
    } master;

    struct {
        float kp;
        float ki;
        float kd; // Servo parameter
    } params;

    struct {
        uint8_t id;
        uint8_t status;
        uint8_t cap_volt; // Kick capcaitor voltage [V]
        uint8_t voltage; // Battery voltage [8th of V]
    } robot;

} com_packet_t;

void frame_encoder(com_packet_t *packet);

void frame_parser(com_packet_t *packet);

uint32_t float_to_uint(float n);

#endif // RADIO_UTILS_H