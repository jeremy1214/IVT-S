#include "Setting_EM_IVT.h"
#include <iomanip>
#include <iostream>
#include <chrono>
#include <ctime>

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

bool IVTCommandBuilder::decodeCommandResponse(
    const CANCommandFrame& rx_frame,
    IVTCommandResponseResult& result) {

    if (rx_frame.dlc < 2) {
        result.is_success = false;
        return false;
    }

    uint8_t resp_code = rx_frame.data[0];
    if (resp_code < 0x90 || resp_code > 0xBF) {
        result.is_success = false;
        return false;
    }

    result.response_code = resp_code;
    result.result_status = rx_frame.data[1];

    switch (resp_code) {
        case 0x94: result.description = "Set Mode Response (0x94)"; break;
        case 0xB2: result.description = "Store EEPROM Response (0xB2)"; break;
        case 0xBF: result.description = "Restart Sensor Response (0xBF)"; break;
        case 0xB5: result.description = "Set Pos OCS Threshold Response (0xB5)"; break;
        case 0xB6: result.description = "Set Neg OCS Threshold Response (0xB6)"; break;
        default:
            if (resp_code >= 0xA0 && resp_code <= 0xA7) {
                result.description = "Config Channel Response (0x" + std::to_string(static_cast<int>(resp_code)) + ")";
            } else {
                result.description = "Command Response (0x" + std::to_string(static_cast<int>(resp_code)) + ")";
            }
            break;
    }

    result.is_success = (result.result_status == 0x00);
    return true;
}

// 【新增】解析週期性發送的實時數據 (CAN ID: 0x521 ~ 0x528)
bool IVTCommandBuilder::decodeDataResponse(
    const CANCommandFrame& rx_frame,
    IVTDataResult& result) {

    if (rx_frame.dlc < 6) {
        result.is_success = false;
        return false;
    }

    // 可從 CAN ID (0x521 ~ 0x528) 或 DB0 (Mux ID: 0x00 ~ 0x07) 判斷通道
    uint8_t mux_id = rx_frame.data[0];
    if (rx_frame.id >= 0x521 && rx_frame.id <= 0x528) {
        mux_id = rx_frame.id - 0x521;
    }

    if (mux_id > 0x07) {
        result.is_success = false;
        return false;
    }

    result.mux_id       = mux_id;
    result.channel_name = channelToString(static_cast<IVTChannel>(mux_id));
    result.status_flags = rx_frame.data[1];

    // DB2..DB5 為 32-bit Big-Endian 有符號整數 (int32_t)
    int32_t raw_32 = (static_cast<int32_t>(rx_frame.data[2]) << 24) |
                     (static_cast<int32_t>(rx_frame.data[3]) << 16) |
                     (static_cast<int32_t>(rx_frame.data[4]) << 8)  |
                     (static_cast<int32_t>(rx_frame.data[5]));

    result.raw_value = raw_32;

    // 依據手冊 8.1 節 (Page 21) 的物理數值轉換比例
    switch (static_cast<IVTChannel>(mux_id)) {
        case IVTChannel::Current:
            result.physical_value = raw_32 / 1000.0; // 1 mA/LSB -> A
            result.unit = "A";
            break;
        case IVTChannel::Voltage1:
        case IVTChannel::Voltage2:
        case IVTChannel::Voltage3:
            result.physical_value = raw_32 / 1000.0; // 1 mV/LSB -> V
            result.unit = "V";
            break;
        case IVTChannel::Temp:
            result.physical_value = raw_32 / 10.0;   // 0.1 °C/LSB -> °C
            result.unit = "°C";
            break;
        case IVTChannel::Power:
            result.physical_value = raw_32;          // 1 W/LSB -> W
            result.unit = "W";
            break;
        case IVTChannel::As:
            result.physical_value = raw_32;          // 1 As/LSB -> As
            result.unit = "As";
            break;
        case IVTChannel::Wh:
            result.physical_value = raw_32;          // 1 Wh/LSB -> Wh
            result.unit = "Wh";
            break;
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

void IVTCommandBuilder::logCANMessage(
    std::ofstream& log_file,
    const std::string& direction,
    uint32_t can_id,
    uint8_t dlc,
    const uint8_t* data,
    const std::string& description) {
    
    if (!log_file.is_open()) return;

    // 取得精確時間戳記 (毫秒級)
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    char time_str[100];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time));

    log_file << "[" << time_str << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
             << "[" << direction << "] "
             << "ID: 0x" << std::hex << std::uppercase << can_id << std::dec << " | "
             << "DLC: " << static_cast<int>(dlc) << " | Data: ";

    for (int i = 0; i < dlc; ++i) {
        log_file << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                 << static_cast<int>(data[i]) << " ";
    }
    log_file << std::dec;

    if (!description.empty()) {
        log_file << " | (" << description << ")";
    }
    log_file << std::endl;
}

void printCANFrame(const CANCommandFrame& frame) {
    std::cout << "[" << frame.description << "]" << std::endl;
    std::cout << "  ID   : 0x" << std::hex << std::uppercase << frame.id << std::dec << std::endl;
    std::cout << "  DLC  : " << static_cast<int>(frame.dlc) << std::endl;
    std::cout << "  Data : ";
    for (uint8_t i = 0; i < frame.dlc; ++i) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl << std::endl;
}