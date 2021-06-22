/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_CodevEsc.h"
#include <AP_Arming/AP_Arming.h>



extern const AP_HAL::HAL &hal;

AP_CodevEsc::AP_CodevEsc(/* args */)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Fence must be singleton");
    }

    _singleton = this;
}

AP_CodevEsc::~AP_CodevEsc()
{
    channels_count = 0;
}

void AP_CodevEsc::init()
{
    channels_count = 4;
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CodevEsc, 0);
    if (uart != nullptr) {
        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_CodevEsc, 0);
        uart->begin(baudrate);

        // configure and initialise the esc
        configure_esc();
    }
}


int AP_CodevEsc::configure_esc()
{

    unsigned long _esc_start_time_us;

    _esc_start_time_us = AP_HAL::micros64();

    if (_esc_start_time_us < MAX_BOOT_TIME_MS * 1000) {
        hal.scheduler->delay_microseconds((MAX_BOOT_TIME_MS * 1000) - _esc_start_time_us);
    }

    /* Issue Basic Config */
    EscPacket packet = {PACKET_HEAD, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
    ConfigInfoBasicRequest   &config = packet.d.reqConfigInfoBasic;
    memset(&config, 0, sizeof(ConfigInfoBasicRequest));
    config.maxChannelInUse = channels_count;
    /* Enable closed-loop control if supported by the board */
    config.controlMode = BOARD_TAP_ESC_MODE;

    /* Asign the id's to the ESCs to match the mux */
	for (uint8_t phy_chan_index = 0; phy_chan_index < channels_count; phy_chan_index++) {
		config.channelMapTable[phy_chan_index] = _device_mux_map[phy_chan_index] &
				ESC_MASK_MAP_CHANNEL;
		config.channelMapTable[phy_chan_index] |= (_device_dir_map[phy_chan_index] << 4) &
				ESC_MASK_MAP_RUNNING_DIRECTION;
	}

    config.maxChannelValue = RPMMAX;
	config.minChannelValue = RPMMIN - 5;

    select_responder(0);
    int packet_len =  crc_packet(packet);
    uart->write(&packet.head,packet_len);

	/* set wait time for tap esc configurate and write flash (0.02696s measure by Saleae logic Analyzer) */
	hal.scheduler->delay_microseconds(3000);


    /* To Unlock the ESC from the Power up state we need to issue 10
    * ESCBUS_MSG_ID_RUN request with all the values 0;
    */
    EscPacket unlock_packet = {PACKET_HEAD, channels_count, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(unlock_packet.d.bytes));

    int unlock_times = 10;

	while (unlock_times--) {
        packet_len = crc_packet(unlock_packet);
        uart->write(&unlock_packet.head,packet_len);

		/* Min Packet to Packet time is 1 Ms so use 2 */
		hal.scheduler->delay_microseconds(1000);
	}
    return 0;
}

void AP_CodevEsc::execute_codev_esc()
{
    if (uart != nullptr) {
        send_esc_outputs();
    }
}

void AP_CodevEsc::send_esc_outputs()
{
    uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM] = {};
    for (uint8_t i = 0;i < 4; i++) {

        AP_Motors *motors = AP_Motors::get_singleton();
        if (motors->armed()) {
            motor_out[i]= constrain_int16(motor_out[i],RPMMIN,RPMMAX);
        }

        rpm[i] = motor_out[i];

        if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) > RPMMAX) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMMAX;

		} else if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) < RPMSTOPPED) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMSTOPPED;
		}

        // apply the led color
        if (i < BOARD_MAX_LEDS) {
            set_led_status(i,control_mode,rpm[i]);
		}
    }

    rpm[_device_mux_map[responding_esc]] |= RUN_FEEDBACK_ENABLE_MASK;

    EscPacket packet = {PACKET_HEAD, channels_count, ESCBUS_MSG_ID_RUN};
	packet.len *= sizeof(packet.d.reqRun.rpm_flags[0]);

	for (uint8_t i = 0; i < channels_count; i++) {
		packet.d.reqRun.rpm_flags[i] = rpm[i];
	}

    if (responding_esc >= 0) {
        select_responder(responding_esc);
    }

    int packet_len = crc_packet(packet);

    uart->write(&packet.head,packet_len);
    if (++responding_esc >= channels_count) {
        responding_esc = 0;
    }
}

void AP_CodevEsc::set_led_status(uint8_t id,uint8_t mode,uint16_t& led_status)
{
    unsigned long _esc_time_now_us = AP_HAL::micros64();
    uint16_t tail_left_led = 0;
    uint16_t tail_right_led = 0;
    bool arm_state = false;
    bool arm_checks_status = false;
    AP_Arming &ap_arm = AP::arming();

    switch ((int8_t)mode)
    {
    case ALT_HOLD:
        tail_left_led = RUN_BLUE_LED_ON_MASK;
        tail_right_led = RUN_BLUE_LED_ON_MASK;
        break;
    case LOITER:
        tail_left_led = RUN_RED_LED_ON_MASK;
        tail_right_led = RUN_GREEN_LED_ON_MASK;
        break;
    case RTL:
        // yellow color
        tail_left_led = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
        tail_right_led = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
        break;

    case AUTO:
    case GUIDED:
    case LAND:
        // purple color
        tail_left_led = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
        tail_right_led = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
        break;

    default:
        tail_left_led = RUN_RED_LED_ON_MASK;
        tail_right_led = RUN_RED_LED_ON_MASK;
        break;
    }

    arm_state = ap_arm.is_armed();

    if (!arm_state) {
        arm_checks_status = ap_arm.get_pre_arm_passed();

        if (!arm_checks_status) {
            tail_left_led = RUN_BLUE_LED_ON_MASK;
            tail_right_led = RUN_BLUE_LED_ON_MASK;
        }
    }

    if (_esc_led_on_time_us == 0) {
        _esc_led_on_time_us = AP_HAL::micros64();
        led_on_off = 0;
    }

    // open led if on
    if (led_on_off == 0) {
        switch (id)
        {
        case 0:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 1:
            led_status |= tail_left_led;
            break;
        case 2:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 3:
            led_status |= tail_right_led;
            break;

        default:
            break;
        }

        if (_esc_time_now_us - _esc_led_on_time_us > LED_ON_TIME_MS * 1000) {
            led_on_off = 1;
            _esc_led_on_time_us = _esc_time_now_us;
        }
    } else {

        // Turn off led
        switch (id)
        {
        case 0:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 1:
            led_status |= RUN_RED_LED_OFF_MASK | RUN_GREEN_LED_OFF_MASK | RUN_BLUE_LED_OFF_MASK;
            break;
        case 2:
            led_status |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
            break;
        case 3:
            led_status |= RUN_RED_LED_OFF_MASK | RUN_GREEN_LED_OFF_MASK | RUN_BLUE_LED_OFF_MASK;
            break;

        default:
            break;
        }

        if (_esc_time_now_us - _esc_led_on_time_us > LED_OFF_TIME_MS * 1000) {
            led_on_off = 0;
            _esc_led_on_time_us = _esc_time_now_us;
        }
    }
}


void AP_CodevEsc::select_responder(uint8_t channel)
{
    #if defined(HAL_ESC_SELECT0_GPIO_PIN)
		hal.gpio->write(HAL_ESC_SELECT0_GPIO_PIN, channel & 1);
		hal.gpio->write(HAL_ESC_SELECT1_GPIO_PIN, channel & 2);
		hal.gpio->write(HAL_ESC_SELECT2_GPIO_PIN, channel & 4);
    #endif
}


uint8_t AP_CodevEsc::crc_packet(EscPacket &p)
{
	/* Calculate the crc over Len,ID,data */
	p.d.bytes[p.len] = crc8_esc(&p.len, p.len + 2);
	return p.len + offsetof(EscPacket, d) + 1;
}

uint8_t AP_CodevEsc::crc8_esc(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = crc_table[crc^*p++];
	}

	return crc;
}


// singleton instance
AP_CodevEsc *AP_CodevEsc::_singleton;

namespace AP {

AP_CodevEsc *AP_CodevEsc()
{
    return AP_CodevEsc::get_singleton();
}

}
