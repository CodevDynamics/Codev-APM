/*
  SToRM32 mount backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"

#define AP_MOUNT_MAVLINK_GIMBAL_RESEND_MS  1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_MAVLINK_GIMBAL_INIT_MS  20000   // search for gimbal for 20 seconds after startup

class AP_Mount_Mavlink : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_Mavlink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override {}

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override;

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

    void mount_orientation_angle(float roll, float pitch, float yaw, float yaw_absolute) override;

    void control_camera(const mavlink_command_long_t &packet) override;

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    // send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
    void send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode);

    void send_attitude();

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal.  Currently hard-coded to Telem2
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    uint32_t _found_gimbal_time = 0;            // system time of last do_mount_control sent to gimbal
    uint32_t _last_mount_update = 0;            // system time of last do_mount_control sent to gimbal

    float _mount_roll = 0.0f;
    float _mount_pitch = 0.0f;
    float _mount_yaw = 0.0f;
    float _mount_yaw_absolute = 0.0f;

    float mount_control_roll = 0.0;
    float mount_control_pitch = 0.0;
    float mount_control_yaw = 0.0;

    uint16_t _count = 0;
};
