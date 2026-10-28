#include "EM_IVT_Decode.h"

#include <iostream>
#include <iomanip>

namespace {

int32_t parseBigEndianInt32(const uint8_t* bytes) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(bytes[0]) << 24) |
        (static_cast<uint32_t>(bytes[1]) << 16) |
        (static_cast<uint32_t>(bytes[2]) << 8)  |
        (static_cast<uint32_t>(bytes[3]))
    );
}

}

bool decodeIVTFrame(const CANFrame& frame, IVTDecodedData& result) {
    // 檢查 CAN ID 是否在預設的常態刷新訊息範圍內 (0x521 ~ 0x528)
    if (frame.id < 0x521 || frame.id > 0x528) {
        return false;
    }

    // IVT-S 的常態刷新結果封包長度固定為 6 Bytes (DB0 ~ DB5)
    if (frame.dlc < 6) {
        return false;
    }

    // --- 解析 DB0: MuxID ---
    result.mux_id = frame.data[0];

    // --- 解析 DB1: Low Nibble (Rolling Counter) 與 High Nibble (Status Flags) ---
    result.msg_count = frame.data[1] & 0x0F;
    
    uint8_t status_flags = (frame.data[1] >> 4) & 0x0F;
    result.status_flags = static_cast<IVTDecodedData::StatusFlag>(status_flags);

    // --- 解析 DB2..DB5: 32-bit Signed Integer (Big-Endian) ---
    result.raw_value = parseBigEndianInt32(&frame.data[2]);

    // --- 根據 MuxID / CAN ID 轉換為實際物理量 ---
    switch (result.mux_id) {
        case 0x00: // 電流 Current
            result.channel_name   = "Current (I)";
            result.physical_value = result.raw_value / 1000.0; // 1 mA/LSB -> A
            result.unit           = "A";
            break;

        case 0x01: // 電壓 Voltage 1
            result.channel_name   = "Voltage 1 (U1)";
            result.physical_value = result.raw_value / 1000.0; // 1 mV/LSB -> V
            result.unit           = "V";
            break;

        case 0x02: // 電壓 Voltage 2
            result.channel_name   = "Voltage 2 (U2)";
            result.physical_value = result.raw_value / 1000.0; // 1 mV/LSB -> V
            result.unit           = "V";
            break;

        case 0x03: // 電壓 Voltage 3
            result.channel_name   = "Voltage 3 (U3)";
            result.physical_value = result.raw_value / 1000.0; // 1 mV/LSB -> V
            result.unit           = "V";
            break;

        case 0x04: // 溫度 Temperature
            result.channel_name   = "Temperature (T)";
            result.physical_value = result.raw_value / 10.0;   // 0.1 °C/LSB -> °C
            result.unit           = "°C";
            break;

        case 0x05: // 即時功率 Power
            result.channel_name   = "Power (W)";
            result.physical_value = static_cast<double>(result.raw_value); // 1 W/LSB
            result.unit           = "W";
            break;

        case 0x06: // 電量累計 Ah Count
            result.channel_name   = "Ampere-Seconds (As)";
            result.physical_value = static_cast<double>(result.raw_value); // 1 As/LSB
            result.unit           = "As";
            break;

        case 0x07: // 能量累計 Wh Count
            result.channel_name   = "Watt-Hours (Wh)";
            result.physical_value = static_cast<double>(result.raw_value); // 1 Wh/LSB
            result.unit           = "Wh";
            break;

        default:
            return false; // 未知的 MuxID
    }

    return true;
}

void printIVTData(const IVTDecodedData& data) {
    std::cout << "==========================================" << std::endl;
    std::cout << data.channel_name << " (MuxID: 0x" 
              << std::hex << (int)data.mux_id << std::dec << ")" << std::endl;
    std::cout << "IVT_MsgCount: " << (int)data.msg_count << " (0~15)" << std::endl;
    std::cout << "Raw value: " << data.raw_value << std::endl;
    std::cout << "Physical value: " << std::fixed << std::setprecision(3) 
              << data.physical_value << " " << data.unit << std::endl;
    
    // 印出狀態與警告旗標
    std::cout << "Status flags: [";
    const uint8_t status_flags = static_cast<uint8_t>(data.status_flags);
    const bool system_error = (status_flags & static_cast<uint8_t>(IVTDecodedData::StatusFlag::SYSTEM_ERROR)) != 0;
    const bool out_of_range = (status_flags & static_cast<uint8_t>(IVTDecodedData::StatusFlag::OUT_OF_RANGE)) != 0;
    const bool ocs_triggered = (status_flags & static_cast<uint8_t>(IVTDecodedData::StatusFlag::OCS_TRIGGERED)) != 0;
    const bool any_error = (status_flags & static_cast<uint8_t>(IVTDecodedData::StatusFlag::ANY_ERROR)) != 0;
    if (system_error)  std::cout << " [Fatal system error!]";
    if (out_of_range)  std::cout << " [Out of range / measurement error]";
    if (ocs_triggered) std::cout << " [Overcurrent protection triggered (OCS)]";
    if (any_error)     std::cout << " [Error detected on one or more channels]";
    if (!system_error && !out_of_range && !ocs_triggered && !any_error) {
        std::cout << " Normal (OK) ";
    }
    std::cout << "]" << std::endl;
    std::cout << "==========================================" << std::endl << std::endl;
}