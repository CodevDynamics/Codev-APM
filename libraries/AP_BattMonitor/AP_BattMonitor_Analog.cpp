#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

#if HAL_CODEV_ESC_ENABLE
#include <AP_CodevEsc/AP_CodevEsc.h>
#endif
extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _volt_pin_analog_source = hal.analogin->channel(_params._volt_pin);
    _curr_pin_analog_source = hal.analogin->channel(_params._curr_pin);

    // always healthy
    _state.healthy = true;
}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    // this copes with changing the pin at runtime
    _volt_pin_analog_source->set_pin(_params._volt_pin);

    // get voltage
    _state.voltage = _volt_pin_analog_source->voltage_average() * _params._volt_multiplier;

    // read current
    if (has_current()) {
#if HAL_CODEV_ESC_ENABLE
        const AP_CodevEsc *motor_esc = AP::codevesc();
        if (motor_esc == nullptr) {
            return;
        }
        uint32_t tnow = AP_HAL::micros();

        uint16_t current = 0;
        for (int i = 0; i < HAL_ESC_NUM; i++) {
            current  += motor_esc->_esc_status[i].current;
        }
        _state.current_amps = current * 0.01f;
        // record time
        _state.last_time_micros = tnow;
# else
        uint32_t tnow = AP_HAL::micros();
        // calculate time since last current read
        float dt = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _curr_pin_analog_source->set_pin(_params._curr_pin);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average()-_params._curr_amp_offset)*_params._curr_amp_per_volt;

        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            float mah = _state.current_amps * dt * 0.0000002778f;
            _state.consumed_mah += mah;
            _state.consumed_wh  += 0.001f * mah * _state.voltage;
        }

        // record time
        _state.last_time_micros = tnow;
#endif
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog::has_current() const
{
#if HAL_CODEV_ESC_ENABLE
    return true;
#endif
    return (_params.type() == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
}
