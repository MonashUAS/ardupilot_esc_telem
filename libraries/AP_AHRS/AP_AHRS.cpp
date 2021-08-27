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

/*
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS.h"
#include "AP_AHRS_View.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Module/AP_Module.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#ifndef HAL_AHRS_EKF_TYPE_DEFAULT
#define HAL_AHRS_EKF_TYPE_DEFAULT 3
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] = {
    // index 0 and 1 are for old parameters that are no longer not used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
    // @Range: 0.0 1.0
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0f),

    // @Param: GPS_USE
    // @DisplayName: AHRS use GPS for DCM navigation and position-down
    // @Description: This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight. Currently this affects only the DCM-based AHRS: the EKF uses GPS according to its own parameters. A value of 2 means to use GPS for height as well as position - both in DCM estimation and when determining altitude-above-home.
    // @Values: 0:Disabled,1:Use GPS for DCM position,2:Use GPS for DCM position and height
    // @User: Advanced
    AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, float(GPSUse::Enable)),

    // @Param: YAW_P
    // @DisplayName: Yaw P
    // @Description: This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
    // @Range: 0.1 0.4
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.2f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0.1 0.4
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.2f),

    // @Param: WIND_MAX
    // @DisplayName: Maximum wind
    // @Description: This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is. See ARSPD_OPTIONS and ARSPD_MAX_WIND to disable airspeed sensors.
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // NOTE: 7 was BARO_USE

    // @Param: TRIM_X
    // @DisplayName: AHRS Trim Roll
    // @Description: Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
    // @Units: rad
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Standard

    // @Param: TRIM_Y
    // @DisplayName: AHRS Trim Pitch
    // @Description: Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
    // @Units: rad
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Standard

    // @Param: TRIM_Z
    // @DisplayName: AHRS Trim Yaw
    // @Description: Not Used
    // @Units: rad
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TRIM", 8, AP_AHRS, _trim, 0),

    // @Param: ORIENTATION
    // @DisplayName: Board Orientation
    // @Description: Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. The label for each option is specified in the order of rotations for that orientation. This option takes affect on next boot. After changing you will need to re-level your vehicle.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom
    // @User: Advanced
    AP_GROUPINFO("ORIENTATION", 9, AP_AHRS, _board_orientation, 0),

    // @Param: COMP_BETA
    // @DisplayName: AHRS Velocity Complementary Filter Beta Coefficient
    // @Description: This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
    // @Range: 0.001 0.5
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("COMP_BETA",  10, AP_AHRS, beta, 0.1f),

    // @Param: GPS_MINSATS
    // @DisplayName: AHRS GPS Minimum satellites
    // @Description: Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_MINSATS", 11, AP_AHRS, _gps_minsats, 6),

    // NOTE: index 12 was for GPS_DELAY, but now removed, fixed delay
    // of 1 was found to be the best choice

    // 13 was the old EKF_USE

    // @Param: EKF_TYPE
    // @DisplayName: Use NavEKF Kalman filter for attitude and position estimation
    // @Description: This controls which NavEKF Kalman filter version is used for attitude and position estimation
    // @Values: 0:Disabled,2:Enable EKF2,3:Enable EKF3,11:ExternalAHRS
    // @User: Advanced
    AP_GROUPINFO("EKF_TYPE",  14, AP_AHRS, _ekf_type, HAL_AHRS_EKF_TYPE_DEFAULT),

    // @Param: CUSTOM_ROLL
    // @DisplayName: Board orientation roll offset
    // @Description: Autopilot mounting position roll offset. Positive values = roll right, negative values = roll left. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CUSTOM_ROLL", 15, AP_AHRS, _custom_roll, 0),

    // @Param: CUSTOM_PIT
    // @DisplayName: Board orientation pitch offset
    // @Description: Autopilot mounting position pitch offset. Positive values = pitch up, negative values = pitch down. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CUSTOM_PIT", 16, AP_AHRS, _custom_pitch, 0),

    // @Param: CUSTOM_YAW
    // @DisplayName: Board orientation yaw offset
    // @Description: Autopilot mounting position yaw offset. Positive values = yaw right, negative values = yaw left. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CUSTOM_YAW", 17, AP_AHRS, _custom_yaw, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// constructor
AP_AHRS::AP_AHRS(uint8_t flags) :
    _ekf_flags(flags)
{
    _singleton = this;

    // load default values from var_info table
    AP_Param::setup_object_defaults(this, var_info);

#if APM_BUILD_COPTER_OR_HELI() || APM_BUILD_TYPE(APM_BUILD_ArduSub)
    // Copter and Sub force the use of EKF
    _ekf_flags |= AP_AHRS::FLAG_ALWAYS_USE_EKF;
#endif
    _dcm_matrix.identity();

    // initialise the controller-to-autopilot-body trim state:
    _last_trim = _trim.get();
    _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, _last_trim.z);
    _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
}

// init sets up INS board orientation
void AP_AHRS::init()
{
    // EKF1 is no longer supported - handle case where it is selected
    if (_ekf_type.get() == 1) {
        AP_BoardConfig::config_error("EKF1 not available");
    }
#if !HAL_NAVEKF2_AVAILABLE && HAL_NAVEKF3_AVAILABLE
    if (_ekf_type.get() == 2) {
        _ekf_type.set(3);
        ekf3.set_enable(true);
    }
#elif !HAL_NAVEKF3_AVAILABLE && HAL_NAVEKF2_AVAILABLE
    if (_ekf_type.get() == 3) {
        _ekf_type.set(2);
        ekf2.set_enable(true);
    }
#endif

    last_active_ekf_type = (EKFType)_ekf_type.get();

    // init backends
    dcm.init();
#if HAL_EXTERNAL_AHRS_ENABLED
    external.init();
#endif

#if HAL_NMEA_OUTPUT_ENABLED
    _nmea_out = AP_NMEA_Output::probe();
#endif
}

// updates matrices responsible for rotating vectors from vehicle body
// frame to autopilot body frame from _trim variables
void AP_AHRS::update_trim_rotation_matrices()
{
    if (_last_trim == _trim.get()) {
        // nothing to do
        return;
    }

    _last_trim = _trim.get();
    _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, _last_trim.z);
    _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
}

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS::get_gyro(void) const
{
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS::get_rotation_body_to_ned(void) const
{
    return _dcm_matrix;
}

const Vector3f &AP_AHRS::get_gyro_drift(void) const
{
    return _gyro_drift;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS::reset_gyro_drift(void)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    // update DCM
    dcm.reset_gyro_drift();
#if HAL_EXTERNAL_AHRS_ENABLED
    external.reset_gyro_drift();
#endif

    // reset the EKF gyro bias states
#if HAL_NAVEKF2_AVAILABLE
    EKF2.reset_gyro_drift();
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.reset_gyro_drift();
#endif
}

void AP_AHRS::update(bool skip_ins_update)
{
    if (!skip_ins_update) {
        // tell the IMU to grab some data
        AP::ins().update();
    }

    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);

    // see if we have to restore home after a watchdog reset:
    if (!_checked_watchdog_home) {
        load_watchdog_home();
        _checked_watchdog_home = true;
    }

    // drop back to normal priority if we were boosted by the INS
    // calling delay_microseconds_boost()
    hal.scheduler->boost_end();

    // update autopilot-body-to-vehicle-body from _trim parameters:
    update_trim_rotation_matrices();

    update_DCM();

    // update takeoff/touchdown flags
    update_flags();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    update_SITL();
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    update_external();
#endif
    
    if (_ekf_type == 2) {
        // if EK2 is primary then run EKF2 first to give it CPU
        // priority
#if HAL_NAVEKF2_AVAILABLE
        update_EKF2();
#endif
#if HAL_NAVEKF3_AVAILABLE
        update_EKF3();
#endif
    } else {
        // otherwise run EKF3 first
#if HAL_NAVEKF3_AVAILABLE
        update_EKF3();
#endif
#if HAL_NAVEKF2_AVAILABLE
        update_EKF2();
#endif
    }

    update_notify_from_filter_status();

#if AP_MODULE_SUPPORTED
    // call AHRS_update hook if any
    AP_Module::call_hook_AHRS_update(*this);
#endif

    // push gyros if optical flow present
    if (hal.opticalflow) {
        const Vector3f &exported_gyro_bias = get_gyro_drift();
        hal.opticalflow->push_gyro_bias(exported_gyro_bias.x, exported_gyro_bias.y);
    }

    if (_view != nullptr) {
        // update optional alternative attitude view
        _view->update();
    }

    // update AOA and SSA
    update_AOA_SSA();

#if HAL_NMEA_OUTPUT_ENABLED
    // update NMEA output
    if (_nmea_out != nullptr) {
        _nmea_out->update();
    }
#endif

    EKFType active = active_EKF_type();
    if (active != last_active_ekf_type) {
        last_active_ekf_type = active;
        const char *shortname = "???";
        switch ((EKFType)active) {
        case EKFType::NONE:
            shortname = "DCM";
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case EKFType::SIM:
            shortname = "SITL";
            break;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
        case EKFType::EXTERNAL:
            shortname = "External";
            break;
#endif
#if HAL_NAVEKF3_AVAILABLE
        case EKFType::THREE:
            shortname = "EKF3";
            break;
#endif
#if HAL_NAVEKF2_AVAILABLE
        case EKFType::TWO:
            shortname = "EKF2";
            break;
#endif
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS: %s active", shortname);
    }
}

/*
 * copy results from a backend over AP_AHRS canonical results.
 * This updates member variables like roll and pitch, as well as
 * updating derived values like sin_roll and sin_pitch.
 */
void AP_AHRS::copy_estimates_from_backend_estimates(const AP_AHRS_Backend::Estimates &results)
{
    roll = results.roll_rad;
    pitch = results.pitch_rad;
    yaw = results.yaw_rad;

    _dcm_matrix = results.dcm_matrix;

    _gyro_estimate = results.gyro_estimate;
    _gyro_drift = results.gyro_drift;

    // copy earth-frame accelerometer estimates:
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_ef_ekf[i] = results.accel_ef[i];
    }
    _accel_ef_ekf_blended = results.accel_ef_blended;

    update_cd_values();
    update_trig();
}

void AP_AHRS::update_DCM()
{
    dcm.update();
    dcm.get_results(dcm_estimates);

    // we always update the vehicle's canonical roll/pitch/yaw from
    // DCM.  In normal operation this will usually be over-written by
    // an EKF or external AHRS.  This is long-held behaviour, but this
    // really shouldn't be doing this.

    // if (active_EKF_type() == EKFType::NONE) {
        copy_estimates_from_backend_estimates(dcm_estimates);
    // }
}

void AP_AHRS::update_notify_from_filter_status()
{
    nav_filter_status status;
    get_filter_status(status);

    AP_Notify::flags.gps_fusion = status.flags.using_gps; // Drives AP_Notify flag for usable GPS.
    AP_Notify::flags.gps_glitching = status.flags.gps_glitching;
    AP_Notify::flags.have_pos_abs = status.flags.horiz_pos_abs;
}

#if HAL_NAVEKF2_AVAILABLE
void AP_AHRS::update_EKF2(void)
{
    EKF2.update();
    if (!EKF2.started()) {
        return;
    }
    EKF2.get_results(ekf2_estimates);

    if (active_EKF_type() == EKFType::TWO) {
        copy_estimates_from_backend_estimates(ekf2_estimates);
    }
}

void AP_AHRS_NavEKF2::update()
{
    if (!_ekf2_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
        // if we're doing Replay logging then don't allow any data
        // into the EKF yet.  Don't allow it to block us for long.
        if (!hal.util->was_watchdog_reset()) {
            if (AP_HAL::millis() - start_time_ms < 5000) {
                if (!AP::logger().allow_start_ekf()) {
                    return;
                }
            }
        }

        if (AP_HAL::millis() - start_time_ms > startup_delay_ms) {
            _ekf2_started = EKF2.InitialiseFilter();
        }
    }
    if (_ekf2_started) {
        EKF2.UpdateFilter();
    }
}

void AP_AHRS_NavEKF2::get_results(AP_AHRS_Backend::Estimates &results)
{
            Vector3f eulers;
            EKF2.getRotationBodyToNED(results.dcm_matrix);
            dcm_matrix = results.dcm_matrix;
            EKF2.getEulerAngles(-1,eulers);
            results.roll  = eulers.x;
            results.pitch = eulers.y;
            results.yaw   = eulers.z;

            // Use the primary EKF to select the primary gyro
            const int8_t primary_imu = EKF2.getPrimaryCoreIMUIndex();

            const AP_InertialSensor &_ins = AP::ins();

            // get gyro bias for primary EKF and change sign to give gyro drift
            // Note sign convention used by EKF is bias = measurement - truth
            results.gyro_drift.zero();
            EKF2.getGyroBias(-1, results.gyro_drift);
            results.gyro_drift = -results.gyro_drift;

            // calculate corrected gyro estimate for get_gyro()
            if (primary_imu == -1 || !_ins.get_gyro_health(primary_imu)) {
                // the primary IMU is undefined so use an uncorrected default value from the INS library
                results.gyro_estimate = _ins.get_gyro();
            } else {
                // use the same IMU as the primary EKF and correct for gyro drift
                results.gyro_estimate = _ins.get_gyro(primary_imu) + results.gyro_drift;
            }

            // get z accel bias estimate from active EKF (this is usually for the primary IMU)
            float abias = 0;
            EKF2.getAccelZBias(-1,abias);

            // This EKF is currently using primary_imu, and abias applies to only that IMU
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i == primary_imu) {
                    accel.z -= abias;
                }
                if (_ins.get_accel_health(i)) {
                    results.accel_ef[i] = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;
                }
            }
            results.accel_ef_blended = results.accel_ef[primary_imu>=0?primary_imu:_ins.get_primary_accel()];
}
#endif

#if HAL_NAVEKF3_AVAILABLE
void AP_AHRS::update_EKF3(void)
{
    EKF3.update();
    if (!EKF3.started()) {
        return;
    }
    EKF3.get_results(ekf3_estimates);

    if (active_EKF_type() == EKFType::THREE) {
        copy_estimates_from_backend_estimates(ekf3_estimates);
    }
}

void AP_AHRS_NavEKF3::update()
{
    if (!_ekf3_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
        // if we're doing Replay logging then don't allow any data
        // into the EKF yet.  Don't allow it to block us for long.
        if (!hal.util->was_watchdog_reset()) {
            if (AP_HAL::millis() - start_time_ms < 5000) {
                if (!AP::logger().allow_start_ekf()) {
                    return;
                }
            }
        }
        if (AP_HAL::millis() - start_time_ms > startup_delay_ms) {
            _ekf3_started = EKF3.InitialiseFilter();
        }
    }
    if (!_ekf3_started) {
        EKF3.UpdateFilter();
    }
}

void AP_AHRS_NavEKF3::get_results(AP_AHRS_Backend::Estimates &results)
{
            Vector3f eulers;
            EKF3.getRotationBodyToNED(results.dcm_matrix);
            dcm_matrix = results.dcm_matrix;
            EKF3.getEulerAngles(-1,eulers);
            results.roll  = eulers.x;
            results.pitch = eulers.y;
            results.yaw   = eulers.z;

            const AP_InertialSensor &_ins = AP::ins();

            // Use the primary EKF to select the primary gyro
            const int8_t primary_imu = EKF3.getPrimaryCoreIMUIndex();

            // get gyro bias for primary EKF and change sign to give gyro drift
            // Note sign convention used by EKF is bias = measurement - truth
            results.gyro_drift.zero();
            EKF3.getGyroBias(-1, results.gyro_drift);
            results.gyro_drift = -results.gyro_drift;

            // calculate corrected gyro estimate for get_gyro()
            if (primary_imu == -1 || !_ins.get_gyro_health(primary_imu)) {
                // the primary IMU is undefined so use an uncorrected default value from the INS library
                results.gyro_estimate = _ins.get_gyro();
            } else {
                // use the same IMU as the primary EKF and correct for gyro drift
                results.gyro_estimate = _ins.get_gyro(primary_imu) + results.gyro_drift;
            }

            // get 3-axis accel bias festimates for active EKF (this is usually for the primary IMU)
            Vector3f abias;
            EKF3.getAccelBias(-1,abias);

            // This EKF uses the primary IMU
            // Eventually we will run a separate instance of the EKF for each IMU and do the selection and blending of EKF outputs upstream
            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==_ins.get_primary_accel()) {
                    accel -= abias;
                }
                if (_ins.get_accel_health(i)) {
                    results.accel_ef[i] = results.dcm_matrix * accel;
                }
            }
            results.accel_ef_blended = results.accel_ef[_ins.get_primary_accel()];
}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_AHRS::update_SITL(void)
{
    sim.update();

    if (!sim.started()) {
        return;
    }

    sim.get_results(sim_estimates);

    if (active_EKF_type() == EKFType::SIM) {
        copy_estimates_from_backend_estimates(dcm_estimates);
    }
}

void AP_AHRS_SIM::get_results(AP_AHRS_Backend::Estimates &results)
{
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    const AP_InertialSensor &_ins = AP::ins();

        fdm.quaternion.rotation_matrix(results.dcm_matrix);
        results.dcm_matrix = results.dcm_matrix * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body();
        results.dcm_matrix.to_euler(&results.roll, &results.pitch, &results.yaw);

        results.gyro_drift.zero();

        results.gyro_estimate = _ins.get_gyro();

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            const Vector3f &accel = _ins.get_accel(i);
            results.accel_ef[i] = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;
        }
        results.accel_ef_blended = results.accel_ef[0];
}


#if 0 && HAL_NAVEKF3_AVAILABLE
    if (_sitl->odom_enable) {
        // use SITL states to write body frame odometry data at 20Hz
        uint32_t timeStamp_ms = AP_HAL::millis();
        if (timeStamp_ms - _last_body_odm_update_ms > 50) {
            const float quality = 100.0f;
            const Vector3f posOffset(0.0f, 0.0f, 0.0f);
            const float delTime = 0.001f * (timeStamp_ms - _last_body_odm_update_ms);
            _last_body_odm_update_ms = timeStamp_ms;
            timeStamp_ms -= (timeStamp_ms - _last_body_odm_update_ms)/2; // correct for first order hold average delay
            Vector3f delAng = _ins.get_gyro();
            
            delAng *= delTime;
            // rotate earth velocity into body frame and calculate delta position
            Matrix3f Tbn;
            Tbn.from_euler(radians(fdm.rollDeg),radians(fdm.pitchDeg),radians(fdm.yawDeg));
            const Vector3f earth_vel(fdm.speedN,fdm.speedE,fdm.speedD);
            const Vector3f delPos = Tbn.transposed() * (earth_vel * delTime);
            // write to EKF
            EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, 0, posOffset);
        }
    }
#endif // HAL_NAVEKF3_AVAILABLE

#endif // CONFIG_HAL_BOARD

#if HAL_EXTERNAL_AHRS_ENABLED
void AP_AHRS::update_external(void)
{
    external.update();
    external.get_results(external_estimates);

    if (active_EKF_type() == EKFType::EXTERNAL) {
        copy_estimates_from_backend_estimates(external_estimates);
    }
}
#endif // HAL_EXTERNAL_AHRS_ENABLED

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS::get_accel_ef(uint8_t i) const
{
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS::get_accel_ef_blended(void) const
{
    return _accel_ef_ekf_blended;
}

void AP_AHRS::reset()
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);

    dcm.reset();

#if HAL_EXTERNAL_AHRS_ENABLED
    external.reset();
#endif

#if HAL_NAVEKF2_AVAILABLE
    EKF2.reset();
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.reset();
#endif
}

// dead-reckoning support
bool AP_AHRS::get_position(struct Location &loc) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.get_position(loc);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (EKF3.get_position(loc)) {
            return true;
        }
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (EKF3.get_position(loc)) {
            return true;
        }
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        if (sim.get_position(loc)) {
            return true;
        }
        break;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL: {
        return external.get_position(loc);
    }
#endif
    }

    // fall back to position from DCM
    if (!always_use_EKF()) {
        return dcm.get_position(loc);
    }
    return false;
}

// status reporting of estimated errors
float AP_AHRS::get_error_rp(void) const
{
    return dcm.get_error_rp();
}

float AP_AHRS::get_error_yaw(void) const
{
    return dcm.get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS::wind_estimate(void) const
{
    Vector3f wind;
    switch (active_EKF_type()) {
    case EKFType::NONE:
        wind = dcm.wind_estimate();
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        wind = sim.wind_estimate();
        break;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        wind = EKF2.wind_estimate();
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        wind = EKF3.wind_estimate();
        break;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        wind = external.wind_estimate();
        break;
#endif
        
    }
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS::airspeed_estimate(float &airspeed_ret) const
{
    if (airspeed_sensor_enabled()) {
        airspeed_ret = AP::airspeed()->get_airspeed();

        if (_wind_max > 0 && AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
            // constrain the airspeed by the ground speed
            // and AHRS_WIND_MAX
            const float gnd_speed = AP::gps().ground_speed();
            float true_airspeed = airspeed_ret * get_EAS2TAS();
            true_airspeed = constrain_float(true_airspeed,
                                            gnd_speed - _wind_max,
                                            gnd_speed + _wind_max);
            airspeed_ret = true_airspeed / get_EAS2TAS();
        }

        return true;
    }

    if (!get_wind_estimation_enabled()) {
        return false;
    }

    // estimate it via nav velocity and wind estimates

    // get wind estimates
    Vector3f wind_vel;
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.airspeed_estimate(get_active_airspeed_index(), airspeed_ret);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.airspeed_estimate(get_active_airspeed_index(), airspeed_ret);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return dcm.airspeed_estimate(get_active_airspeed_index(), airspeed_ret);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (EKF3.airspeed_estimate(get_active_airspeed_index(), airspeed_ret)) {
            return true;
        }
        return dcm.airspeed_estimate(get_active_airspeed_index(), airspeed_ret);
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.airspeed_estimate(get_active_airspeed_index(), airspeed_ret);
#endif
    }
    return false;
}

bool AP_AHRS::airspeed_estimate_true(float &airspeed_ret) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.airspeed_estimate_true(airspeed_ret);
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
#endif
        break;
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.airspeed_estimate_true(airspeed_ret);
#endif
    }

    if (!airspeed_estimate(airspeed_ret)) {
        return false;
    }
    airspeed_ret *= get_EAS2TAS();
    return true;
}

// return estimate of true airspeed vector in body frame in m/s
// returns false if estimate is unavailable
bool AP_AHRS::airspeed_vector_true(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.airspeed_vector_true(vec);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.airspeed_vector_true(vec);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.airspeed_vector_true(vec);
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    return false;
}

// return a synthetic airspeed estimate (one derived from sensors
// other than an actual airspeed sensor), if available. return
// true if we have a synthetic airspeed.  ret will not be modified
// on failure.
bool AP_AHRS::synthetic_airspeed(float &ret) const
{
    return dcm.synthetic_airspeed(ret);
}

// true if compass is being used
bool AP_AHRS::use_compass(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.use_compass();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.use_compass();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.use_compass();
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    return dcm.use_compass();
}

// return the quaternion defining the rotation from NED to XYZ (body) axes
bool AP_AHRS::get_quaternion(Quaternion &quat) const
{
    // backends always return in autopilot XYZ frame; rotate result
    // according to trim
    switch (active_EKF_type()) {
    case EKFType::NONE:
        if (!dcm.get_quaternion(quat)) {
            return false;
        }
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (!EKF2.get_quaternion(quat)) {
            return false;
        }
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (!EKF3.get_quaternion(quat)) {
            return false;
        }
        break;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        if (!sim.get_quaternion(quat)) {
            return false;
        }
        break;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        // we assume the external AHRS isn't trimmed with the autopilot!
        return external.get_quaternion(quat);
#endif
    }

    quat.rotate(-_trim.get());

    return true;
}

// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS::get_secondary_attitude(Vector3f &eulers) const
{
    EKFType secondary_ekf_type;
    if (!get_secondary_EKF_type(secondary_ekf_type)) {
        return false;
    }

    switch (secondary_ekf_type) {

    case EKFType::NONE:
        // DCM is secondary
        eulers[0] = dcm_estimates.roll_rad;
        eulers[1] = dcm_estimates.pitch_rad;
        eulers[2] = dcm_estimates.yaw_rad;
        return true;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 is secondary
        eulers[0] = ekf2_estimates.roll;
        eulers[1] = ekf2_estimates.pitch;
        eulers[2] = ekf2_estimates.yaw;
        return EKF2.started();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // EKF3 is secondary
        eulers[0] = ekf3_estimates.roll;
        eulers[1] = ekf3_estimates.pitch;
        eulers[2] = ekf3_estimates.yaw;
        return EKF3.started();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        // SITL is secondary (should never happen)
        return false;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL: {
        // External is secondary
        eulers[0] = external_estimates.roll;
        eulers[1] = external_estimates.pitch;
        eulers[2] = external_estimates.yaw;
        return true;
    }
#endif
    }

    // since there is no default case above, this is unreachable
    return false;
}


// return secondary attitude solution if available, as quaternion
bool AP_AHRS::get_secondary_quaternion(Quaternion &quat) const
{
    EKFType secondary_ekf_type;
    if (!get_secondary_EKF_type(secondary_ekf_type)) {
        return false;
    }

    switch (secondary_ekf_type) {

    case EKFType::NONE:
        // DCM is secondary
        if (!dcm.get_quaternion(quat)) {
            return false;
        }
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 is secondary
        if (!EKF2.started()) {
            return false;
        }
        if (!EKF2.get_quaternion(quat)) {
            return false;
        }
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // EKF3 is secondary
        if (!EKF3.started()) {
            return false;
        }
        if (!EKF3.get_quaternion(quat)) {
            return false;
        }
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        // SITL is secondary (should never happen)
        return false;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        // External is secondary
        return external.get_quaternion(quat);
#endif
    }

    quat.rotate(-_trim.get());

    return true;
}

// return secondary position solution if available
bool AP_AHRS::get_secondary_position(struct Location &loc) const
{
    EKFType secondary_ekf_type;
    if (!get_secondary_EKF_type(secondary_ekf_type)) {
        return false;
    }

    switch (secondary_ekf_type) {

    case EKFType::NONE:
        // return DCM position
        dcm.get_position(loc);
        return true;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 is secondary
        if (!EKF2.started()) {
            return false;
        }
        return EKF2.get_position(loc);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // EKF3 is secondary
        if (!EKF3.started()) {
            return false;
        }
        return EKF3.get_position(loc);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        // SITL is secondary (should never happen)
        return false;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        // External is secondary
        return external.get_position(loc);
#endif
    }

    // since there is no default case above, this is unreachable
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS::groundspeed_vector(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.groundspeed_vector();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.groundspeed_vector();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.groundspeed_vector();
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL: {
        return external.groundspeed_vector();
    }
#endif
    }
    return dcm.groundspeed_vector();
}

float AP_AHRS::groundspeed(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.groundspeed();
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.groundspeed();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.groundspeed();
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.groundspeed();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.groundspeed();
#endif
    }
    return groundspeed_vector().length();
}

// set the EKF's origin location in 10e7 degrees.  This should only
// be called when the EKF has no absolute position reference (i.e. GPS)
// from which to decide the origin on its own
bool AP_AHRS::set_origin(const Location &loc)
{
    WITH_SEMAPHORE(_rsem);
#if HAL_NAVEKF2_AVAILABLE
    const bool ret2 = EKF2.set_origin(loc);
#endif
#if HAL_NAVEKF3_AVAILABLE
    const bool ret3 = EKF3.set_origin(loc);
#endif

    // return success if active EKF's origin was set
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return ret2;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return ret3;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.set_origin(loc);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        // don't allow origin set with external AHRS
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return true if inertial navigation is active
bool AP_AHRS::have_inertial_nav(void) const
{
    return active_EKF_type() != EKFType::NONE;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS::get_velocity_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_velocity_NED(vec);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_velocity_NED(vec);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_velocity_NED(vec);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.get_velocity_NED(vec);
#endif
    }
    return dcm.get_velocity_NED(vec);
}

// returns the expected NED magnetic field
bool AP_AHRS::get_mag_field_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_mag_field_NED(vec);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF2.get_mag_field_NED(vec);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_mag_field_NED(vec);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    return false;
}

// returns the estimated magnetic field offsets in body frame
bool AP_AHRS::get_mag_field_correction(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_mag_field_correction(vec);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_mag_field_correction(vec);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_mag_field_correction(vec);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
bool AP_AHRS::get_vert_pos_rate(float &velocity) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.get_vert_pos_rate(velocity);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_vert_pos_rate(velocity);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_vert_pos_rate(velocity);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_vert_pos_rate(velocity);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.get_vert_pos_rate(velocity);
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// get latest height above ground level estimate in metres and a validity flag
bool AP_AHRS::get_hagl(float &height) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_hagl(height);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_hagl(height);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_hagl(height);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL: {
        return false;
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return a relative ground position to the origin in meters
// North/East/Down order.
bool AP_AHRS::get_relative_position_NED_origin(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.get_relative_position_NED_origin(vec);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_relative_position_NED_origin(vec);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_relative_position_NED_origin(vec);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_relative_position_NED_origin(vec);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL: {
        return external.get_relative_position_NED_origin(vec);
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return a relative ground position to the home in meters
// North/East/Down order.
bool AP_AHRS::get_relative_position_NED_home(Vector3f &vec) const
{
    Location originLLH;
    Vector3f originNED;
    if (!get_relative_position_NED_origin(originNED) ||
        !get_origin(originLLH)) {
        return false;
    }

    const Vector3f offset = originLLH.get_distance_NED(_home);

    vec.x = originNED.x - offset.x;
    vec.y = originNED.y - offset.y;
    vec.z = originNED.z - offset.z;
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order
// return true if estimate is valid
bool AP_AHRS::get_relative_position_NE_origin(Vector2f &posNE) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.get_relative_position_NE_origin(posNE);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        return EKF2.get_relative_position_NE_origin(posNE);
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        return EKF3.get_relative_position_NE_origin(posNE);
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_relative_position_NE_origin(posNE);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.get_relative_position_NE_origin(posNE);
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return a relative ground position to the home in meters
// North/East order.
bool AP_AHRS::get_relative_position_NE_home(Vector2f &posNE) const
{
    Location originLLH;
    Vector2f originNE;
    if (!get_relative_position_NE_origin(originNE) ||
        !get_origin(originLLH)) {
        return false;
    }

    const Vector2f offset = originLLH.get_distance_NE(_home);

    posNE.x = originNE.x - offset.x;
    posNE.y = originNE.y - offset.y;
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order


// write a relative ground position to the origin in meters, Down
// return true if the estimate is valid
bool AP_AHRS::get_relative_position_D_origin(float &posD) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return dcm.get_relative_position_D_origin(posD);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_relative_position_D_origin(posD);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_relative_position_D_origin(posD);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_relative_position_D_origin(posD);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.get_relative_position_D_origin(posD);
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// write a relative ground position to home in meters, Down
// will use the barometer if the EKF isn't available
void AP_AHRS::get_relative_position_D_home(float &posD) const
{
    Location originLLH;
    float originD;
    if (!get_relative_position_D_origin(originD) ||
        !get_origin(originLLH)) {
        const auto &gps = AP::gps();
        if (_gps_use == GPSUse::EnableWithHeight &&
            gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            posD = (get_home().alt - gps.location().alt) * 0.01;
        } else {
            posD = -AP::baro().get_altitude();
        }
        return;
    }

    posD = originD - ((originLLH.alt - _home.alt) * 0.01f);
    return;
}
/*
  canonicalise _ekf_type, forcing it to be 0, 2 or 3
  type 1 has been deprecated
 */
AP_AHRS::EKFType AP_AHRS::ekf_type(void) const
{
    EKFType type = (EKFType)_ekf_type.get();
    switch (type) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return type;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return type;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return type;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return type;
#endif
    case EKFType::NONE:
        if (always_use_EKF()) {
#if HAL_NAVEKF2_AVAILABLE
            return EKFType::TWO;
#elif HAL_NAVEKF3_AVAILABLE
            return EKFType::THREE;
#endif
        }
        return EKFType::NONE;
    }
    // we can get to here if the user has mis-set AHRS_EKF_TYPE - any
    // value above 3 will get to here.  TWO is returned here for no
    // better reason than "tradition".
#if HAL_NAVEKF2_AVAILABLE
    return EKFType::TWO;
#elif HAL_NAVEKF3_AVAILABLE
    return EKFType::THREE;
#else
    return EKFType::NONE;
#endif
}

AP_AHRS::EKFType AP_AHRS::active_EKF_type(void) const
{
    EKFType ret = EKFType::NONE;

    switch (ekf_type()) {
    case EKFType::NONE:
        return EKFType::NONE;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        // do we have an EKF2 yet?
        if (!EKF2.started()) {
            return EKFType::NONE;
        }
        if (always_use_EKF()) {
            uint16_t ekf2_faults = 0;
            EKF2.get_filter_faults(-1,ekf2_faults);
            if (ekf2_faults == 0) {
                ret = EKFType::TWO;
            }
        } else if (EKF2.healthy()) {
            ret = EKFType::TWO;
        }
        break;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        // do we have an EKF3 yet?
        if (!EKF3.started()) {
            return EKFType::NONE;
        }
        if (always_use_EKF()) {
            uint16_t ekf3_faults = 0;
            EKF3.get_filter_faults(-1,ekf3_faults);
            if (ekf3_faults == 0) {
                ret = EKFType::THREE;
            }
        } else if (EKF3.healthy()) {
            ret = EKFType::THREE;
        }
        break;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        ret = EKFType::SIM;
        break;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        ret = EKFType::EXTERNAL;
        break;
#endif
    }

    /*
      fixed wing and rover will fall back to DCM if the EKF doesn't
      have GPS. This is the safest option as DCM is very robust. Note
      that we also check the filter status when fly_forward is false
      and we are disarmed. This is to ensure that the arming checks do
      wait for good GPS position on fixed wing and rover
     */
    if (ret != EKFType::NONE &&
        (_vehicle_class == VehicleClass::FIXED_WING ||
         _vehicle_class == VehicleClass::GROUND)) {
        bool should_use_gps = true;
        nav_filter_status filt_state;
#if HAL_NAVEKF2_AVAILABLE
        if (ret == EKFType::TWO) {
            EKF2.get_filter_status(filt_state);
            should_use_gps = EKF2.configured_to_use_gps_for_posxy();
        }
#endif
#if HAL_NAVEKF3_AVAILABLE
        if (ret == EKFType::THREE) {
            EKF3.get_filter_status(filt_state);
            should_use_gps = EKF3.configured_to_use_gps_for_posxy();
        }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (ret == EKFType::SIM) {
            get_filter_status(filt_state);
        }
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
        if (ret == EKFType::EXTERNAL) {
            get_filter_status(filt_state);
            should_use_gps = true;
        }
#endif
        if (hal.util->get_soft_armed() &&
            (!filt_state.flags.using_gps ||
             !filt_state.flags.horiz_pos_abs) &&
            should_use_gps &&
            AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            // if the EKF is not fusing GPS or doesn't have a 2D fix
            // and we have a 3D lock, then plane and rover would
            // prefer to use the GPS position from DCM. This is a
            // safety net while some issues with the EKF get sorted
            // out
            return EKFType::NONE;
        }
        if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
            return EKFType::NONE;
        }
        if (!filt_state.flags.attitude ||
            !filt_state.flags.vert_vel ||
            !filt_state.flags.vert_pos) {
            return EKFType::NONE;
        }
        if (!filt_state.flags.horiz_vel ||
            (!filt_state.flags.horiz_pos_abs && !filt_state.flags.horiz_pos_rel)) {
            if ((!AP::compass().use_for_yaw()) &&
                AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D &&
                AP::gps().ground_speed() < 2) {
                /*
                  special handling for non-compass mode when sitting
                  still. The EKF may not yet have aligned its yaw. We
                  accept EKF as healthy to allow arming. Once we reach
                  speed the EKF should get yaw alignment
                */
                if (filt_state.flags.gps_quality_good) {
                    return ret;
                }
            }
            return EKFType::NONE;
        }
    }
    return ret;
}

// get secondary EKF type.  returns false if no secondary (i.e. only using DCM)
bool AP_AHRS::get_secondary_EKF_type(EKFType &secondary_ekf_type) const
{

    switch (active_EKF_type()) {
    case EKFType::NONE:
        // EKF2, EKF3 or External is secondary
#if HAL_NAVEKF3_AVAILABLE
        if ((EKFType)_ekf_type.get() == EKFType::THREE) {
            secondary_ekf_type = EKFType::THREE;
            return true;
        }
#endif
#if HAL_NAVEKF2_AVAILABLE
        if ((EKFType)_ekf_type.get() == EKFType::TWO) {
            secondary_ekf_type = EKFType::TWO;
            return true;
        }
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
        if ((EKFType)_ekf_type.get() == EKFType::EXTERNAL) {
            secondary_ekf_type = EKFType::EXTERNAL;
            return true;
        }
#endif
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
#endif
        // DCM is secondary
        secondary_ekf_type = EKFType::NONE;
        return true;
    }

    // since there is no default case above, this is unreachable
    return false;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters

    // an internal processing error, but not for bad sensor data.
    if ((_vehicle_class == VehicleClass::FIXED_WING ||
         _vehicle_class == VehicleClass::GROUND) &&
        active_EKF_type() != ekf_type()) {
        return false;
    }

    switch (ekf_type()) {
    case EKFType::NONE:
        return dcm.healthy();

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        return EKF2.healthy();
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        return EKF3.healthy();
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.healthy();
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.healthy();
#endif
    }

    return dcm.healthy();
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
// requires_position should be true if horizontal position configuration should be checked
bool AP_AHRS::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    switch (ekf_type()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.pre_arm_check(requires_position, failure_msg, failure_msg_len);
#endif
    case EKFType::NONE:
        return dcm.pre_arm_check(requires_position, failure_msg, failure_msg_len);

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.pre_arm_check(requires_position, failure_msg, failure_msg_len);
#endif
        
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.pre_arm_check(requires_position, failure_msg, failure_msg_len);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.pre_arm_check(requires_position, failure_msg, failure_msg_len);
#endif
    }

    // if we get here then ekf type is invalid
    hal.util->snprintf(failure_msg, failure_msg_len, "invalid EKF type");
    return false;
}

// true if the AHRS has completed initialisation
bool AP_AHRS::initialised(void) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return true;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.initialised();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.initialised();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return true;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.initialised();
#endif
    }
    return false;
};

// get_filter_status : returns filter status as a series of flags
bool AP_AHRS::get_filter_status(nav_filter_status &status) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return dcm.get_filter_status(status);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_filter_status(status);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_filter_status(status);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_filter_status(status);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.get_filter_status(status);
#endif
    }

    return false;
}

// write optical flow data to EKF
void  AP_AHRS::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
#endif
}

// write body frame odometry measurements to the EKF
void  AP_AHRS::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset)
{
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset);
#endif
}

// Write position and quaternion data from an external navigation system
void AP_AHRS::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
#endif
}

// Writes the default equivalent airspeed and 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
void AP_AHRS::writeDefaultAirSpeed(float airspeed, float uncertainty)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeDefaultAirSpeed(airspeed);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeDefaultAirSpeed(airspeed, uncertainty);
#endif
}

// Write velocity data from an external navigation system
void AP_AHRS::writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
#endif
}

// get speed limit and XY navigation gain scale factor
void AP_AHRS::getControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        // lower gains in VTOL controllers when flying on DCM
        ekfGndSpdLimit = 50.0;
        ekfNavVelGainScaler = 0.5;
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.getControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        // no limit on gains, large vel limit
        ekfGndSpdLimit = 400;
        ekfNavVelGainScaler = 1;
        break;
#endif
    }
}

/*
  get gain factor for Z controllers
 */
float AP_AHRS::getControlScaleZ(void) const
{
    if (active_EKF_type() == EKFType::NONE) {
        // when flying on DCM lower gains by 4x to cope with the high
        // lag
        return 0.25;
    }
    return 1;
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_mag_offsets(mag_idx, magOffsets);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_mag_offsets(mag_idx, magOffsets);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        magOffsets.zero();
        return true;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// Retrieves the NED delta velocity corrected
void AP_AHRS::getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getCorrectedDeltaVelocityNED(ret, dt);
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getCorrectedDeltaVelocityNED(ret, dt);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.getCorrectedDeltaVelocityNED(ret, dt);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    return dcm.getCorrectedDeltaVelocityNED(ret, dt);
}

// check all cores providing consistent attitudes for prearm checks
bool AP_AHRS::attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const
{
    // get primary attitude source's attitude as quaternion
    Quaternion primary_quat;
    get_quat_body_to_ned(primary_quat);
    // only check yaw if compasses are being used
    const bool check_yaw = AP::compass().use_for_yaw();
    uint8_t total_ekf_cores = 0;

#if HAL_NAVEKF2_AVAILABLE
    // check primary vs ekf2
    if (!EKF2.attitudes_consistent(primary_quat, check_yaw, failure_msg, failure_msg_len)) {
        return false;
    }
    total_ekf_cores += EKF2.activeCores();
#endif

#if HAL_NAVEKF3_AVAILABLE
    // check primary vs ekf3
    if (!EKF3.attitudes_consistent(primary_quat, check_yaw, failure_msg, failure_msg_len)) {
        return false;
    }
    total_ekf_cores += EKF3.activeCores();
#endif

    // check primary vs dcm
    if (!always_use_EKF() || (total_ekf_cores == 1)) {
        Quaternion dcm_quat;
        dcm_quat.from_rotation_matrix(get_DCM_rotation_body_to_ned());

        // check roll and pitch difference
        const float rp_diff_rad = primary_quat.roll_pitch_difference(dcm_quat);
        if (rp_diff_rad > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
            hal.util->snprintf(failure_msg, failure_msg_len, "DCM Roll/Pitch inconsistent by %d deg", (int)degrees(rp_diff_rad));
            return false;
        }

        // Check vs DCM yaw if this vehicle could use DCM in flight
        // and if not using an external yaw source (DCM does not support external yaw sources)
        bool using_noncompass_for_yaw = false;
#if HAL_NAVEKF3_AVAILABLE
        using_noncompass_for_yaw = (ekf_type() == EKFType::THREE) && EKF3.using_noncompass_for_yaw();
#endif
        if (!always_use_EKF() && !using_noncompass_for_yaw) {
            Vector3f angle_diff;
            primary_quat.angular_difference(dcm_quat).to_axis_angle(angle_diff);
            const float yaw_diff = fabsf(angle_diff.z);
            if (check_yaw && (yaw_diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
                hal.util->snprintf(failure_msg, failure_msg_len, "DCM Yaw inconsistent by %d deg", (int)degrees(yaw_diff));
                return false;
            }
        }
    }

    return true;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastYawResetAngle(float &yawAng)
{
    switch (active_EKF_type()) {

    case EKFType::NONE:
        return dcm.getLastYawResetAngle(yawAng);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastYawResetAngle(yawAng);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastYawResetAngle(yawAng);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.getLastYawResetAngle(yawAng);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.getLastYawResetAngle(yawAng);
#endif
    }
    return 0;
}

// return the amount of NE position change in metres due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastPosNorthEastReset(Vector2f &pos)
{
    switch (active_EKF_type()) {

    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastPosNorthEastReset(pos);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastPosNorthEastReset(pos);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.getLastPosNorthEastReset(pos);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return 0;
#endif
    }
    return 0;
}

// return the amount of NE velocity change in metres/sec due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastVelNorthEastReset(Vector2f &vel) const
{
    switch (active_EKF_type()) {

    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastVelNorthEastReset(vel);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastVelNorthEastReset(vel);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return 0;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return 0;
#endif
    }
    return 0;
}


// return the amount of vertical position change due to the last reset in meters
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastPosDownReset(float &posDelta)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastPosDownReset(posDelta);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastPosDownReset(posDelta);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return 0;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return 0;
#endif
    }
    return 0;
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool AP_AHRS::resetHeightDatum(void)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    switch (ekf_type()) {

    case EKFType::NONE:
#if HAL_NAVEKF3_AVAILABLE
        UNUSED_RESULT(EKF3.resetHeightDatum());
#endif
#if HAL_NAVEKF2_AVAILABLE
        UNUSED_RESULT(EKF2.resetHeightDatum());
#endif
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#if HAL_NAVEKF3_AVAILABLE
        UNUSED_RESULT(EKF3.resetHeightDatum());
#endif
        return EKF2.resetHeightDatum();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#if HAL_NAVEKF2_AVAILABLE
        UNUSED_RESULT(EKF2.resetHeightDatum());
#endif
        return EKF3.resetHeightDatum();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return false;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    return false;
}

// send a EKF_STATUS_REPORT for current EKF
void AP_AHRS::send_ekf_status_report(mavlink_channel_t chan) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        dcm.send_ekf_status_report(chan);
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.send_ekf_status_report(chan);
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL: {
        external.send_ekf_status_report(chan);
        break;
    }
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.send_ekf_status_report(chan);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.send_ekf_status_report(chan);
#endif

    }
}

// passes a reference to the location of the inertial navigation origin
// in WGS-84 coordinates
// returns a boolean true when the inertial navigation origin has been set
bool AP_AHRS::get_origin(Location &ret) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return dcm.get_origin(ret);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_origin(ret);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_origin(ret);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_origin(ret);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return external.get_origin(ret);
#endif
    }

    return false;
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return false when no limiting is required
bool AP_AHRS::get_hgt_ctrl_limit(float& limit) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no limiting applies
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_hgt_ctrl_limit(limit);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_hgt_ctrl_limit(limit);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_hgt_ctrl_limit(limit);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// this is not related to terrain following
void AP_AHRS::set_terrain_hgt_stable(bool stable)
{
    // avoid repeatedly setting variable in NavEKF objects to prevent
    // spurious event logging
    switch (terrainHgtStableState) {
    case TriState::UNKNOWN:
        break;
    case TriState::True:
        if (stable) {
            return;
        }
        break;
    case TriState::False:
        if (!stable) {
            return;
        }
        break;
    }
    terrainHgtStableState = (TriState)stable;

#if HAL_NAVEKF2_AVAILABLE
    EKF2.set_terrain_hgt_stable(stable);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.set_terrain_hgt_stable(stable);
#endif
}

// return the innovations for the primariy EKF
// boolean false is returned if innovations are not available
bool AP_AHRS::get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no data
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // use EKF to get innovations
        return EKF2.get_innovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // use EKF to get innovations
        return EKF3.get_innovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_innovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

// returns true when the state estimates are significantly degraded by vibration
bool AP_AHRS::is_vibration_affected() const
{
    switch (ekf_type()) {
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.is_vibration_affected();
#endif
    case EKFType::NONE:
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
#endif
        return false;
    }
    return false;
}

// get_variances - provides the innovations normalised using the innovation variance where a value of 0
// indicates prefect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
// inconsistency that will be accpeted by the filter
// boolean false is returned if variances are not available
bool AP_AHRS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no data
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_variances(velVar, posVar, hgtVar, magVar, tasVar);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_variances(velVar, posVar, hgtVar, magVar, tasVar);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        return sim.get_variances(velVar, posVar, hgtVar, magVar, tasVar);
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

// get a source's velocity innovations.  source should be from 0 to 7 (see AP_NavEKF_Source::SourceXY)
// returns true on success and results are placed in innovations and variances arguments
bool AP_AHRS::get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no data
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 does not support source level variances
        return false;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // use EKF to get variance
        return EKF3.get_vel_innovations_and_variances_for_source(source, innovations, variances);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        // SITL does not support source level variances
        return false;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

//get the index of the active airspeed sensor, wrt the primary core
uint8_t AP_AHRS::get_active_airspeed_index() const
{
// we only have affinity for EKF3 as of now
#if HAL_NAVEKF3_AVAILABLE
    if (active_EKF_type() == EKFType::THREE) {
        return EKF3.get_active_airspeed_index();
    }
#endif
    // for the rest, let the primary airspeed sensor be used
    const AP_Airspeed * _airspeed = AP::airspeed();
    if (_airspeed != nullptr) {
        return _airspeed->get_primary();
    }
    return 0;
}

// get the index of the current primary IMU
uint8_t AP_AHRS::get_primary_IMU_index() const
{
    int8_t imu = -1;
    switch (ekf_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // let EKF2 choose primary IMU
        imu = EKF2.get_primary_core_IMU_index();
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // let EKF2 choose primary IMU
        imu = EKF3.get_primary_core_IMU_index();
        break;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        break;
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    if (imu == -1) {
        imu = AP::ins().get_primary_accel();
    }
    return imu;
}

// get earth-frame accel vector for primary IMU
const Vector3f &AP_AHRS::get_accel_ef() const
{
    return get_accel_ef(get_primary_accel_index());
}

// return the index of the primary core or -1 if no primary core selected
int8_t AP_AHRS::get_primary_core_index() const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
#endif
        // we have only one core
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_primary_core_index();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_primary_core_index();
#endif
    }

    // we should never get here
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return -1;
}

// get the index of the current primary accelerometer sensor
uint8_t AP_AHRS::get_primary_accel_index(void) const
{
    if (ekf_type() != EKFType::NONE) {
        return get_primary_IMU_index();
    }
    return AP::ins().get_primary_accel();
}

// get the index of the current primary gyro sensor
uint8_t AP_AHRS::get_primary_gyro_index(void) const
{
    if (ekf_type() != EKFType::NONE) {
        return get_primary_IMU_index();
    }
    return AP::ins().get_primary_gyro();
}

// see if EKF lane switching is possible to avoid EKF failsafe
void AP_AHRS::check_lane_switch(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        break;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
        
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.check_lane_switch();
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.check_lane_switch();
        break;
#endif
    }
}

// request EKF yaw reset to try and avoid the need for an EKF lane switch or failsafe
void AP_AHRS::request_yaw_reset(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
        break;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
        
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.request_yaw_reset();
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.request_yaw_reset();
        break;
#endif
    }
}

// set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
void AP_AHRS::set_posvelyaw_source_set(uint8_t source_set_idx)
{
#if HAL_NAVEKF3_AVAILABLE
    EKF3.set_posvelyaw_source_set(source_set_idx);
#endif
}

void AP_AHRS::Log_Write()
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.Log_Write();
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.Log_Write();
#endif
}

// check if non-compass sensor is providing yaw.  Allows compass pre-arm checks to be bypassed
bool AP_AHRS::using_noncompass_for_yaw(void) const
{
    switch (active_EKF_type()) {
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.is_ext_nav_used_for_yaw();
#endif
    case EKFType::NONE:
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.using_noncompass_for_yaw();
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
#endif
        return false; 
    }
    // since there is no default case above, this is unreachable
    return false;
}

// check if external nav is providing yaw
bool AP_AHRS::using_extnav_for_yaw(void) const
{
    switch (active_EKF_type()) {
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.isExtNavUsedForYaw();
#endif
    case EKFType::NONE:
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.using_extnav_for_yaw();
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SIM:
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case EKFType::EXTERNAL:
#endif
        return false;
    }
    // since there is no default case above, this is unreachable
    return false;
}

// set and save the alt noise parameter value
void AP_AHRS::set_alt_measurement_noise(float noise)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.set_baro_alt_noise(noise);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.set_baro_alt_noise(noise);
#endif
}

/*
  get the current view's rotation, or ROTATION_NONE
 */
enum Rotation AP_AHRS::get_view_rotation(void) const
{
    return _view?_view->get_rotation():ROTATION_NONE;
}

// singleton instance
AP_AHRS *AP_AHRS::_singleton;

namespace AP {

AP_AHRS &ahrs()
{
    return *AP_AHRS::get_singleton();
}

}
