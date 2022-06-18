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

#include "AP_RangeFinder_Benewake.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9
#define BENEWAKE_DIST_MAX_CM 32768
#define BENEWAKE_OUT_OF_RANGE_ADD_CM 100

// format of serial packets received from benewake lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x59
// byte 1               Frame header    0x59
// byte 2               DIST_L          Distance (in cm) low 8 bits
// byte 3               DIST_H          Distance (in cm) high 8 bits
// byte 4               STRENGTH_L      Strength low 8 bits
// bute 4 (TF03)        (Reserved)
// byte 5               STRENGTH_H      Strength high 8 bits
// bute 5 (TF03)        (Reserved)
// byte 6 (TF02)        SIG             Reliability in 8 levels, 7 & 8 means reliable
// byte 6 (TFmini)      Distance Mode   0x02 for short distance (mm), 0x07 for long distance (cm)
// byte 6 (TF03)        (Reserved)
// byte 7 (TF02 only)   TIME            Exposure time in two levels 0x03 and 0x06
// byte 8               Checksum        Checksum byte, sum of bytes 0 to bytes 7

// distance returned in reading_m, signal_ok is set to true if sensor reports a strong signal

// searches the buffer for an instance of the header byte, starting at
// search_start_pos.  If found, moves that bytes and all bytes past it
// to the start of the buffer.
void AP_RangeFinder_Benewake::move_header_in_buffer(uint8_t search_start_pos)
{
    const char *header = (char*)memchr(&u.linebuf[search_start_pos],
                                       BENEWAKE_FRAME_HEADER,
                                       linebuf_len - search_start_pos);
    if (header == nullptr) {
        // header byte not found; empty the buffer
        linebuf_len = 0;
        return;
    }
    const uint16_t delta = (char*)header - (char*)u.linebuf;
    if (delta == 0) {
        // header byte already at start of buffer
        return;
    }

    memmove(u.linebuf, &u.linebuf[delta], linebuf_len-delta);
    linebuf_len -= delta;
}

bool AP_RangeFinder_Benewake::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // process at most 100 packets.  get_reading is called at 50Hz, so
    // this is a massive packet rate.
    for (uint8_t packetcount=0; packetcount<100; packetcount++) {
        // fill buffer with any bytes available from the uart:
        const uint32_t nbytes = uart->read(&u.linebuf[linebuf_len], ARRAY_SIZE(u.linebuf)-linebuf_len);
        if (nbytes == 0) {
            break;
        }

        linebuf_len += nbytes;

        move_header_in_buffer(0);

        // ensure we have a complete packet:
        if (linebuf_len < BENEWAKE_FRAME_LENGTH) {
            continue;
        }

        // two indentical header bytes:
        if (u.linebuf[1] != BENEWAKE_FRAME_HEADER) {
            // second header byte not found; discard both bytes:
            move_header_in_buffer(2);
            continue;
        }

        // calculate checksum
        uint8_t checksum = 0;
        for (uint8_t i=0; i<BENEWAKE_FRAME_LENGTH-1; i++) {
            checksum += u.linebuf[i];
        }

        // if checksum does not match then discard this header byte
        // and try again
        if (checksum != u.packet.checksum) {
            move_header_in_buffer(1);
            continue;
        }

        // calculate distance
        const uint16_t dist = u.packet.dist();
        if (dist >= BENEWAKE_DIST_MAX_CM) {
            // this reading is out of range
            count_out_of_range++;
        } else if (!has_signal_byte()) {
            // no signal byte from TFmini so add distance to sum
            sum_cm += dist;
            count++;
        } else {
            // TF02 provides signal reliability (good = 7 or 8)
            if (u.packet.SIG >= 7) {
                // add distance to sum
                sum_cm += dist;
                count++;
            } else {
                // this reading is out of range
                count_out_of_range++;
            }
        }

        // consume this packet:
        move_header_in_buffer(BENEWAKE_FRAME_LENGTH);
    }

    if (count > 0) {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(model_dist_max_cm(), max_distance_cm() + BENEWAKE_OUT_OF_RANGE_ADD_CM) * 0.01f;
        return true;
    }

    // no readings so return false
    return false;
}
