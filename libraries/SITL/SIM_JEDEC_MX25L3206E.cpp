#include "SIM_JEDEC_MX25L3206E.h"

using namespace SITL;

void JEDEC_MX25L3206E::fill_rdid(uint8_t *buffer, uint8_t len)
{
    memcpy(buffer, &manufacturer, ARRAY_SIZE(manufacturer));
    buffer[7] = id1();
    buffer[8] = id2();
}
