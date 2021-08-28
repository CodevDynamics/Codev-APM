#include "AP_Mount_Mavlink.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

AP_Mount_Mavlink::AP_Mount_Mavlink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _chan(MAVLINK_COMM_0)
{}

// update mount position - should be called periodically
void AP_Mount_Mavlink::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    send_attitude();

    const RC_Channel *mount_reset_ch = rc().channel(15 - 1);
    int16_t mount_reset_rc = mount_reset_ch->get_radio_in();

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    float delta_T = 0;
    uint32_t now = AP_HAL::millis();

    // update based on mount mode
    switch(get_mode()) {
        case MAV_MOUNT_MODE_RETRACT:
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            // wait until the gimabl initialise finished
            if (now - _found_gimbal_time < AP_MOUNT_MAVLINK_GIMBAL_INIT_MS) {
                return;
            }

            if (_last_mount_update == 0) {
                _last_mount_update = now;
            }

            delta_T = (now - _last_mount_update) * 1E-3f;

            _last_mount_update = now;

            mount_control_roll += delta_T * ToDeg(_angle_ef_target_rad.x);
            mount_control_pitch += delta_T * ToDeg(_angle_ef_target_rad.y);
            mount_control_yaw += delta_T * ToDeg(_angle_ef_target_rad.z);

            mount_control_yaw = wrap_180(mount_control_yaw);

            // limit the gimbal attitude
            mount_control_pitch = constrain_float(mount_control_pitch,-90.0f,45.0f);
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    bool rc_targeting_mnt = (get_mode() == MAV_MOUNT_MODE_RC_TARGETING) && resend_now;

    if (mount_reset_rc > 1500 && rc_targeting_mnt) {
            mount_control_roll = 0;
            mount_control_pitch = 0;
            mount_control_yaw = 0;
            send_do_mount_control(mount_control_pitch, mount_control_roll, mount_control_yaw, MAV_MOUNT_MODE_MAVLINK_TARGETING);
    } else if (rc_targeting_mnt) {
        // RATES CONTROL
        send_do_mount_control(mount_control_pitch, mount_control_roll, mount_control_yaw, (enum MAV_MOUNT_MODE)get_mode_cfg());
    } else {
        mount_control_yaw = ToDeg(_angle_ef_target_rad.z);
        mount_control_pitch = ToDeg(_angle_ef_target_rad.y);
        send_do_mount_control(ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z), MAV_MOUNT_MODE_MAVLINK_TARGETING);
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Mavlink::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_Mavlink::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

void AP_Mount_Mavlink::mount_orientation_angle(float roll, float pitch, float yaw, float yaw_absolute)
{
    _mount_roll = roll;
    _mount_pitch = pitch;
    _mount_yaw = yaw;
    _mount_yaw_absolute = yaw_absolute;
}

void AP_Mount_Mavlink::control_camera(const mavlink_command_long_t &packet)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(_chan,
                                  packet.target_system,
                                  packet.target_component,
                                  packet.command,
                                  packet.confirmation,
                                  packet.param1,        // confirmation of zero means this is the first time this message has been sent
                                  packet.param2,
                                  packet.param3,
                                  packet.param4,
                                  packet.param5,
                                  packet.param6,
                                  packet.param7);

}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Mavlink::send_mount_status(mavlink_channel_t chan)
{
    // // exit immediately if not initialised
    // if (!_initialised) {
    //     return;
    // }

    // // check we have space for the message
    // if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
    //     return;
    // }
    // // send command_long command containing a do_mount_control command
    // mavlink_msg_command_long_send(_chan,
    //                               _sysid,
    //                               _compid,
    //                               MAV_CMD_DO_MOUNT_CONTROL,
    //                               0,        // confirmation of zero means this is the first time this message has been sent
    //                               ToDeg(_angle_ef_target_rad.y),
    //                               ToDeg(_angle_ef_target_rad.x),
    //                               ToDeg(_angle_ef_target_rad.z),
    //                               0, 0, 0,  // param4 ~ param6 unused
    //                               MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // // store time of send
    // _last_send = AP_HAL::millis();
}


void AP_Mount_Mavlink::send_attitude()
{
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        _chan,
        AP_HAL::millis(),
        ahrs.roll,
        ahrs.pitch,
        -ahrs.yaw,
        omega.x,
        omega.y,
        -omega.z);
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_Mavlink::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    bool findgimbal = GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GENERIC, _sysid, _compid, _chan);
    findgimbal &= (_compid == 154);
    if (findgimbal) {
        _initialised = true;
        _found_gimbal_time = AP_HAL::millis();
    }
}

// send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
void AP_Mount_Mavlink::send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  pitch_deg,
                                  roll_deg,
                                  yaw_deg,
                                  0, 0, 0,  // param4 ~ param6 unused
                                  mount_mode);

    // store time of send
    _last_send = AP_HAL::millis();
}
