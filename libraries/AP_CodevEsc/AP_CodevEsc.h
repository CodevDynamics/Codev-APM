/// @file	AP_CodevEsc.h
/// @brief	exectue the codev esc
#pragma once
#include "drv_codev_esc.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Motors/AP_Motors_Class.h>

#define BOARD_TAP_ESC_MODE 2


// allow the board to override the number (or maxiumum number) of LED's it has
#ifndef BOARD_MAX_LEDS
#define BOARD_MAX_LEDS 4
#endif

// Circular from back right in CCW direction
#define ESC_POS {2, 1, 0, 3, 4, 5, 6, 7}
// 0 is CW, 1 is CCW
#define ESC_DIR {0, 1, 1, 0, 1 ,1, 1, 1}
const uint8_t _device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;
const uint8_t _device_dir_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_DIR;

class AP_CodevEsc
{
public:
    AP_CodevEsc(/* args */);
    ~AP_CodevEsc();

    /// Startup initialisation.
    void init();

    static AP_CodevEsc *get_singleton() { return _singleton; }

    bool uart_state() { return uart==nullptr?false:true;}

    void set_output_pwm(uint8_t chan,uint16_t pwm){ motor_out[chan] = pwm;};

    void execute_codev_esc();

private:

    static AP_CodevEsc *_singleton;

    int configure_esc();
    // strobe the corresponding buffer channel
	void select_responder(uint8_t channel);
    void send_esc_outputs();
    uint8_t crc_packet(EscPacket &p);
    uint8_t crc8_esc(uint8_t *p, uint8_t len);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t baudrate = 0;

    uint8_t    	  channels_count = 0; 		///< nnumber of ESC channels
    int8_t 	    responding_esc = -1;
    uint16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
};
