#pragma once

class VehicleTracking {
    enum class DataType : uint8_t {
        POS = 0,
        VEL = 1,
    };

    class Vehicle {
    public:
        Vector3f pos;
        Vector3f vel;
        Vector3f acc;
        uint32_t last_update_ms;

        uint8_t populated_data_types;  // bitmask from DataType
    };

private:
};
