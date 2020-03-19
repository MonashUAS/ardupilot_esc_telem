#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

extern volatile int16_t exline1;

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

bool AP_BattMonitor_Analog::check_pointer()
{
    static AP_HAL::AnalogSource* expected_volt_pin_analog_source = (AP_HAL::AnalogSource*)0xdeadedde;
    if (expected_volt_pin_analog_source == (AP_HAL::AnalogSource*)0xdeadedde) {
        expected_volt_pin_analog_source = _volt_pin_analog_source;
    } else {
        if (_volt_pin_analog_source != expected_volt_pin_analog_source) {
            volt_pin_analogue_changed = true;
        }
    }
    return volt_pin_analogue_changed;
}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    exline1 = __LINE__;
    // this copes with changing the pin at runtime

    check_pointer();

    // static uint32_t status_message_last_sent;
    // const uint32_t now = AP_HAL::millis();
    // if (now - status_message_last_sent > 1000) {
    //     status_message_last_sent = now;
    //     gcs().send_text(MAV_SEVERITY_WARNING, "want=%p got=%p", expected_volt_pin_analog_source, _volt_pin_analog_source);
    //     if (volt_pin_analogue_changed) {
    //         gcs().send_text(MAV_SEVERITY_ERROR, "POINTER HAS CHANGED");
    //     }
    // }

    if (!volt_pin_analogue_changed) {
        _volt_pin_analog_source->set_pin(_params._volt_pin);
    }

    exline1 = __LINE__;
    // get voltage
    if (!volt_pin_analogue_changed) {
    _state.voltage = _volt_pin_analog_source->voltage_average() * _params._volt_multiplier;
    }

    exline1 = __LINE__;
    // read current
    if (has_current()) {
        exline1 = __LINE__;
        // calculate time since last current read
        uint32_t tnow = AP_HAL::micros();
        float dt = tnow - _state.last_time_micros;

        exline1 = __LINE__;
        // this copes with changing the pin at runtime
        if (!volt_pin_analogue_changed) {
        _curr_pin_analog_source->set_pin(_params._curr_pin);
        }

        exline1 = __LINE__;
        // read current
        if (!volt_pin_analogue_changed) {
        _state.current_amps = (_curr_pin_analog_source->voltage_average()-_params._curr_amp_offset)*_params._curr_amp_per_volt;
        }

        exline1 = __LINE__;
        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            exline1 = __LINE__;
            float mah = _state.current_amps * dt * 0.0000002778f;
            _state.consumed_mah += mah;
            _state.consumed_wh  += 0.001f * mah * _state.voltage;
        }

        exline1 = __LINE__;
        // record time
        _state.last_time_micros = tnow;
    }
    exline1 = __LINE__;

#if 1
    static bool warning_sent = false;
    if (AP_HAL::millis() > 20000 && !warning_sent) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Resetting pointer in 5 seconds");
        warning_sent = true;
    }
    if (AP_HAL::millis() > 25000) {
        _volt_pin_analog_source = 0x0;
    }
#endif CONFIG_HAL_BOARD == HAL_BOARD_SITL

}

/// return true if battery provides current info
bool AP_BattMonitor_Analog::has_current() const
{
    return (_params.type() == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
}
