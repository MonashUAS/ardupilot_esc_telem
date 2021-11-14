#pragma once

#include "SIM_JEDEC.h"

namespace SITL {

class JEDEC_MX25L3206E : public JEDEC
{
protected:

    void fill_rdid(uint8_t *buf, uint8_t len) override;

    uint8_t id1() { return family << 5 | density; }
    uint8_t id2() { return sub << 5 | rev << 2; }

    const char *filename() const override { return "JEDEC-MX25L3206E.dat"; }

    uint32_t storage_size() const override {
        return 32768;
    }

private:

    const uint8_t manufacturer[7] { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2 };
    static const uint8_t family = 1;
    static const uint8_t density = 2;
    static const uint8_t sub = 0;
    static const uint8_t rev = 0;
    static const uint8_t rsvd = 0;

};

}
