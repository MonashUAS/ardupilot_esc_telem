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

#include "AP_RangeFinder_JRE_Serial.h"

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#include <AP_Math/AP_Math.h>

#define FRAME_HEADER_1   'R'    // 0x52
#define FRAME_HEADER_2   'A'    // 0x41

#define DIST_MAX_CM 50000
#define OUT_OF_RANGE_ADD_CM   100

bool AP_RangeFinder_JRE_Serial::get_reading(float &reading_m)
{
    // uart instance check
    if (uart == nullptr) {
        return false;  // not update
    }
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings
    float sum = 0;
    // max distance the sensor can reliably measure - read from parameters
    const int16_t distance_cm_max = max_distance_cm();

    // buffer read
    const ssize_t num_read = uart->read(read_buff, ARRAY_SIZE(read_buff));
    read_buff_idx = 0; // read_buff start data index
    while (read_buff_idx < num_read) {

        if (data_buff_idx == 0) { // header first byte check
            // header data search
            for (; read_buff_idx < num_read; read_buff_idx++) {
                if (read_buff[read_buff_idx] == FRAME_HEADER_1) {
                    data_buff[0] = FRAME_HEADER_1;
                    data_buff_idx = 1; // next data_buff
                    read_buff_idx++;   // next read_buff
                    break;
                }
            }

        } else if (data_buff_idx == 1) { // header second byte check
            if (read_buff[read_buff_idx] == FRAME_HEADER_2) {
                data_buff[1] = FRAME_HEADER_2;
                data_buff_idx = 2;    // next data_buff
            } else {
                data_buff_idx = 0;    // data index clear
            }
            read_buff_idx++;          // next read_buff

        } else { // data set
            if (data_buff_idx >= ARRAY_SIZE(data_buff)) {  // 1 data set complete
                // crc check
                uint16_t crc = crc16_ccitt_r(data_buff, ARRAY_SIZE(data_buff) - 2, 0xffff, 0xffff);
                if ((HIGHBYTE(crc) == data_buff[15]) && (LOWBYTE(crc) == data_buff[14])) {
                    // status check
                    if (data_buff[13] & 0x02) { // NTRK
                        invalid_count++;
                    } else { // TRK
                        reading_m = (data_buff[4] * 256 + data_buff[5]) * 0.01f;
                        sum += reading_m;
                        valid_count++;
                    }
                }
                data_buff_idx = 0; // data index clear
            } else {
                data_buff[data_buff_idx++] = read_buff[read_buff_idx++];
            }
        }
    }

    if (valid_count > 0) {
        no_signal = false;
        reading_m = sum / valid_count;
        return true;
    }

    if (invalid_count > 0) {
        no_signal = true;
        reading_m = MIN(MAX(DIST_MAX_CM, distance_cm_max + OUT_OF_RANGE_ADD_CM), UINT16_MAX) * 0.01f;
        return true;
    }

    return false;
}
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
