#pragma once

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
 *  Simulated perfect AHRS (Attitude Heading Reference System)
 *  interface for ArduPilot
 *
 */

#include "AP_AHRS_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <SITL/SITL.h>

class AP_AHRS_SIM : public AP_AHRS_Backend {
public:

    using AP_AHRS_Backend::AP_AHRS_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_SIM);

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override {
        // always perfect anyway
    }

    bool initialised() const override {
        return AP::sitl() != nullptr;
    }

    void update() override {
        // nothing to do
    }

    void get_results(Estimates &results) override;
    void reset() override {
        // nothing to do
    }

    // dead-reckoning support
    bool get_position(struct Location &loc) const override;

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate() const override {
        return Vector3f{};
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const override;

    // return a synthetic airspeed estimate (one derived from sensors
    // other than an actual airspeed sensor), if available. return
    // true if we have a synthetic airspeed.  ret will not be modified
    // on failure.
    bool synthetic_airspeed(float &ret) const override WARN_IF_UNUSED {
        return false;
    }

    // return a ground vector estimate in meters/second, in North/East order
    Vector2f groundspeed_vector() override;

    bool use_compass() override {
        return true;  // sure, why not?
    }

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    bool healthy() const override {
        return true;
    }

    bool get_velocity_NED(Vector3f &vec) const override;

    // Get a derivative of the vertical position in m/s which is
    // kinematically consistent with the vertical position is required
    // by some control loops.
    // This is different to the vertical velocity from the EKF which
    // is not always consistent with the vertical position due to the
    // various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const override;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override {
        return AP::sitl() != nullptr;
    }

    // relative-origin functions for fallback in AP_InertialNav
    bool set_origin(const Location &loc) override {
        // never allow origin set in SITL. The origin is set by the
        // simulation backend
        return false;
    }
    bool get_relative_position_NED_origin(Vector3f &vec) const override;

    bool get_origin(Location &ret) const override;
    bool get_relative_position_NE_origin(Vector2f &posNE) const override;
    bool get_relative_position_D_origin(float &posD) const override;
    bool get_hagl(float &height) const override;

    bool get_filter_status(nav_filter_status &status) const override;

    void send_ekf_status_report(mavlink_channel_t chan) const override;

    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override;

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    void getControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const {
        // same as EKF2 for no optical flow
        ekfGndSpdLimit = 400.0f;
        ekfNavVelGainScaler = 1.0f;
    }
};

#endif
