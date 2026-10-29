#include "Setting_EM_IVT.h"
#include <iomanip>
#include <iostream>

static std::string channelToString(IVTChannel channel) {
    switch (channel) {
        case IVTChannel::Current:  return "Current (I)";
        case IVTChannel::Voltage1: return "Voltage 1 (U1)";
        case IVTChannel::Voltage2: return "Voltage 2 (U2)";
        case IVTChannel::Voltage3: return "Voltage 3 (U3)";
        case IVTChannel::Temp:     return "Temperature (T)";
        case IVTChannel::Power:    return "Power (W)";
        case IVTChannel::As:       return "As Counter";
        case IVTChannel::Wh:       return "Wh Counter";
        default:                   return "Unknown Channel";
    }
}

static std::string getErrorDescription(IVTLogType type, uint8_t index) {
    if (type == IVTLogType::MeasurementErrors) {
        switch (index) {
            case 0x00: return "Bitmask of occurred measurement errors";
            case 0x01: return "Error ADC interrupt";
            case 0x02: return "Error Overflow ADC channel 1";
            case 0x03: return "Error Underflow ADC channel 1";
            case 0x04: return "Error Overflow ADC channel 2";
            case 0x05: return "Error Underflow ADC channel 2";
            case 0x06: return "Error Vref";
            case 0x07: return "Error current measurement implausible I1-I2";
            case 0x08: return "Error thermal EMF correction";
            case 0x09: return "Error current measurement I1 open circuit";
            case 0x0A: return "Error voltage measurement U1 open circuit";
            case 0x0B: return "Error voltage measurement U2 open circuit";
            case 0x0C: return "Error voltage measurement U3 open circuit";
            case 0x0D: return "Error ntc-h open circuit";
            case 0x0E: return "Error ntc-l open circuit";
            case 0x0F: return "Error calibration data";
            default:   return "Measurement Error Index 0x" + std::to_string(index);
        }
    } else if (type == IVTLogType::SystemErrors) {
        switch (index) {
            case 0x00: return "Bitmask of occurred system errors";
            case 0x01: return "Error Code CRC";
            case 0x02: return "Error Parameter CRC";
            case 0x03: return "Error CAN bus receive Data";
            case 0x04: return "Error CAN bus transmit Data";
            case 0x05: return "Error overtemp";
            case 0x06: return "Error undertemp";
            case 0x07: return "Error power failure";
            case 0x08: return "Error system clock";
            case 0x09: return "Error system init";
            case 0x0A: return "Error configuration";
            case 0x0B: return "Error overcurrent detection";
            case 0x0C: return "Error eeprom r/w";
            case 0x0D: return "Error ADC Clock";
            case 0x0E: return "Error Reset illegal opcode";
            case 0x0F: return "Error Reset Watchdog";
            case 0x10: return "Error Reset EMC";
            default:   return "System Error Index 0x" + std::to_string(index);
        }
    } else {
        switch (index) {
            case 0x01: return "Ampere hour counter overall (As)";
            case 0x04: return "Energy counter overall (Wh)";
            case 0x10: return "Runtime overall (s)";
            case 0x21: return "Current maximum (A)";
            case 0x22: return "Current minimum (A)";
            case 0x23: return "U1 maximum (V)";
            case 0x24: return "U1 minimum (V)";
            case 0x29: return "Temperature maximum (C)";
            case 0x2A: return "Temperature minimum (C)";
            default:   return "Log Item 0x" + std::to_string(index);
        }
    }
}

CANCommandFrame IVTCommandBuilder::buildGetErrorRequest(
    IVTLogType log_type,
    uint8_t log_index,
    uint32_t cmd_id) {

    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = static_cast<uint8_t>(log_type);
    frame.data[1] = log_index;

    for (int i = 2; i < 8; ++i) {
        frame.data[i] = 0x00;
    }

    frame.description = "Get Log/Error Request (CMD 0x" + 
                        std::to_string(static_cast<int>(log_type)) + 
                        ", Index 0x" + std::to_string(log_index) + ") [" + 
                        getErrorDescription(log_type, log_index) + "]";
    return frame;
}

bool IVTCommandBuilder::decodeErrorResponse(
    const CANCommandFrame& rx_frame,
    IVTErrorLogResult& result) {

    if (rx_frame.dlc < 3) {
        result.is_success = false;
        return false;
    }

    uint8_t resp_code = rx_frame.data[0];
    if (resp_code < 0x80 || resp_code > 0x83) {
        result.is_success = false;
        return false;
    }

    result.response_code = resp_code;
    result.log_index     = rx_frame.data[1];
    
    IVTLogType type = static_cast<IVTLogType>(resp_code - 0x40);
    result.description  = getErrorDescription(type, result.log_index);

    result.raw_value = 0;
    for (int i = 2; i < rx_frame.dlc; ++i) {
        result.raw_value = (result.raw_value << 8) | rx_frame.data[i];
    }

    result.is_success = true;
    return true;
}

CANCommandFrame IVTCommandBuilder::configChannel(
    IVTChannel channel,
    bool enable_cyclic,
    uint16_t period_ms,
    bool is_little_endian,
    uint32_t cmd_id) {
    
    uint8_t mux_id = static_cast<uint8_t>(channel);
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x20 | (mux_id & 0x0F);

    uint8_t mode_bits = enable_cyclic ? 0x02 : 0x00;
    uint8_t flags = is_little_endian ? (1 << 6) : 0x00;
    
    frame.data[1] = (flags & 0xF0) | (mode_bits & 0x0F);
    frame.data[2] = static_cast<uint8_t>((period_ms >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(period_ms & 0xFF);

    for (int i = 4; i < 8; ++i) frame.data[i] = 0x00;

    frame.description = "Config " + channelToString(channel) +
                        " [Mode: " + (enable_cyclic ? "Cyclic" : "Disabled") +
                        ", Period: " + std::to_string(period_ms) + "ms]";
    return frame;
}

CANCommandFrame IVTCommandBuilder::setPositiveThreshold(
    int16_t threshold_A,
    int16_t reset_threshold_A,
    uint32_t cmd_id) {
    
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x35; // SET_THRESHOLD_POS
    
    uint16_t set_val = static_cast<uint16_t>(threshold_A);
    uint16_t reset_val = static_cast<uint16_t>(reset_threshold_A);

    frame.data[1] = static_cast<uint8_t>((set_val >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(set_val & 0xFF);
    frame.data[3] = static_cast<uint8_t>((reset_val >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(reset_val & 0xFF);

    for (int i = 5; i < 8; ++i) frame.data[i] = 0x00;

    frame.description = "Set Pos OCS Threshold: Set=" + std::to_string(threshold_A) + "A, Reset=" + std::to_string(reset_threshold_A) + "A";
    return frame;
}

CANCommandFrame IVTCommandBuilder::setNegativeThreshold(
    int16_t threshold_A,
    int16_t reset_threshold_A,
    uint32_t cmd_id) {
    
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x36; // SET_THRESHOLD_NEG
    
    uint16_t set_val = static_cast<uint16_t>(threshold_A);
    uint16_t reset_val = static_cast<uint16_t>(reset_threshold_A);

    frame.data[1] = static_cast<uint8_t>((set_val >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(set_val & 0xFF);
    frame.data[3] = static_cast<uint8_t>((reset_val >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(reset_val & 0xFF);

    for (int i = 5; i < 8; ++i) frame.data[i] = 0x00;

    frame.description = "Set Neg OCS Threshold: Set=" + std::to_string(threshold_A) + "A, Reset=" + std::to_string(reset_threshold_A) + "A";
    return frame;
}

CANCommandFrame IVTCommandBuilder::buildSetMode(uint8_t mode, uint8_t default_mode, uint32_t cmd_id) {
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x34; // SET_MODE
    frame.data[1] = mode;
    frame.data[2] = default_mode;
    for (int i = 3; i < 8; ++i) frame.data[i] = 0x00;

    const std::string mode_str = (mode == 0x00) ? "STOP" : "RUN";
    frame.description = "Set Mode to " + mode_str;
    return frame;
}

CANCommandFrame IVTCommandBuilder::buildStoreCommand(uint32_t cmd_id) {
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x32; // STORE
    for (int i = 1; i < 8; ++i) frame.data[i] = 0x00;
    frame.description = "STORE Configuration to EEPROM";
    return frame;
}

// 【修正】遵循官方手冊 8.4 節，RESTART 指令碼為 0x3F
CANCommandFrame IVTCommandBuilder::buildRestartCommand(uint32_t cmd_id) {
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x3F; // RESTART
    for (int i = 1; i < 8; ++i) frame.data[i] = 0x00;
    frame.description = "RESTART Sensor (0x3F)";
    return frame;
}

std::vector<CANCommandFrame> IVTCommandBuilder::buildFullConfigSequence(
    const std::vector<CANCommandFrame>& custom_configs,
    uint32_t cmd_id) {
    
    std::vector<CANCommandFrame> sequence;

    // 1. 切換至 STOP Mode
    sequence.push_back(buildSetMode(0x00, 0x00, cmd_id));

    // 2. 寫入自訂參數
    for (const auto& config : custom_configs) {
        sequence.push_back(config);
    }

    // 3. 儲存參數至 EEPROM
    sequence.push_back(buildStoreCommand(cmd_id));

    // 4. 設定回 RUN Mode
    sequence.push_back(buildSetMode(0x01, 0x01, cmd_id));

    // 5. 固化預設啟動模式 (STORE)
    sequence.push_back(buildStoreCommand(cmd_id));

    // 6. 重啟感測器 (發送 0x3F)
    sequence.push_back(buildRestartCommand(cmd_id));

    return sequence;
}

void printCANFrame(const CANCommandFrame& frame) {
    std::cout << "[" << frame.description << "]" << std::endl;
    std::cout << "  CAN ID : 0x" << std::hex << std::uppercase << frame.id << std::dec << std::endl;
    std::cout << "  DLC    : " << static_cast<int>(frame.dlc) << std::endl;
    std::cout << "  Payload: ";
    for (uint8_t i = 0; i < frame.dlc; ++i) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl << std::endl;
}