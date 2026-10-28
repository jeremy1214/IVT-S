#pragma once

#include <cstdint>
#include <string>

struct CANFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

struct IVTDecodedData {
    uint8_t mux_id;
    std::string channel_name;
    uint8_t msg_count;

    enum class StatusFlag : uint8_t {
        NONE = 0x00,
        OCS_TRIGGERED = 0x01,
        OUT_OF_RANGE = 0x02,
        ANY_ERROR = 0x04,
        SYSTEM_ERROR = 0x08
    };

    StatusFlag status_flags;

    int32_t raw_value;
    double physical_value;
    std::string unit;
};

bool decodeIVTFrame(const CANFrame& frame, IVTDecodedData& result);
void printIVTData(const IVTDecodedData& data);