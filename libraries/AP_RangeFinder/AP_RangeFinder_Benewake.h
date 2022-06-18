#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    virtual float model_dist_max_cm() const = 0;
    virtual bool has_signal_byte() const { return false; }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    // this structure is sufficient to extract data for all the
    // backends.  See description of packet format in .cpp file.
    struct BenewakePacket {
        uint8_t headermagic1;
        uint8_t headermagic2;
        uint8_t DIST_L;
        uint8_t DIST_H;
        uint8_t STRENGTH_L;  // reserved on TF03
        uint8_t STRENGTH_H;  // reserved on TF03
        uint8_t SIG;  // only on TF02
        uint8_t TIME;  // only on TF02
        uint8_t checksum;

        uint16_t dist() const {
            return DIST_H<<8 | DIST_L;
        }
    };

    union BenewakeUnion {
        uint8_t linebuf[10];
        struct BenewakePacket packet;
    } u;

    uint8_t linebuf_len;

    void move_header_in_buffer(uint8_t search_start_pos);
};
