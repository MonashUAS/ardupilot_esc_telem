/*
  ESC Telemetry for HobbyWing Platinum v4
 */

#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_HOBBYWING_PLATINUM_V4_ENABLED

#include "AP_HobbyWing_ESC.h"

class AP_HobbyWing_Platinum_v4 : public AP_HobbyWing_ESC {
public:

    using AP_HobbyWing_ESC::AP_HobbyWing_ESC;

    void update() override;

};

#endif  // AP_HOBBYWING_PLATINUM_V4_ENABLED