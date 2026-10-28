#include "Setting_EM_IVT.h"

#include <iomanip>
#include <iostream>

// 輔助函式：轉換 Channel 列舉為可讀字串
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

static std::string getErrorDescription(uint8_t index) {
    switch (index) {
        case 0x00: return "All occurred errors (Bitmask)";
        case 0x01: return "Error ADC interrupt / System Error Hardware";
        case 0x02: return "Error Overflow ADC channel 1 / System Error Software";
        case 0x03: return "Error Underflow ADC channel 1";
        case 0x04: return "Error Overflow ADC channel 2";
        case 0x05: return "Error Underflow ADC channel 2 / Error Overtemperature";
        case 0x06: return "Error Vref / Error Undertemperature";
        case 0x07: return "Error current measurement implausible I1 - I2";
        case 0x08: return "Error thermal EMF correction";
        case 0x09: return "Error current measurement I1 open circuit / Runtime Overcurrent Pos";
        case 0x0A: return "Error voltage measurement U1 open circuit / Runtime Overcurrent Neg";
        case 0x0B: return "Error voltage measurement U2 open circuit";
        case 0x0C: return "Error voltage measurement U3 open circuit";
        case 0x0D: return "Error ntc-h open circuit";
        case 0x0E: return "Error ntc-l open circuit";
        case 0x0F: return "Error calibration data (offset/gain too high)";
        default:   return "Custom Log Index (0x" + std::to_string(index) + ")";
    }
}

// 1. 生成 Get Error / Log 請求封包
CANCommandFrame IVTCommandBuilder::buildGetErrorRequest(
    IVTLogType log_type,
    uint8_t log_index,
    uint32_t cmd_id) {

    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = static_cast<uint8_t>(log_type); // 0x40 或 0x42
    frame.data[1] = log_index;                      // 指定 Log Index (0x00 ~ 0x1F)

    // DB2 ~ DB7 保留填 0x00
    for (int i = 2; i < 8; ++i) {
        frame.data[i] = 0x00;
    }

    std::string type_str = (log_type == IVTLogType::SinceReset) ? "Since Reset" : "Overall";
    frame.description = "Get Error/Log (" + type_str + ") for Index 0x" + 
                        std::to_string(log_index) + " [" + getErrorDescription(log_index) + "]";
    return frame;
}

// 2. 解譯 Response 回傳封包 (接收 CAN ID: 0x511)
bool IVTCommandBuilder::decodeErrorResponse(
    const CANCommandFrame& rx_frame,
    IVTErrorLogResult& result) {

    // 檢查 DLC 長度（標準應為 8）
    if (rx_frame.dlc < 6) {
        result.is_success = false;
        return false;
    }

    // 檢查 DB0 回應代碼：必須為 0xC0 (SinceReset Response) 或 0xC2 (Overall Response)
    uint8_t resp_code = rx_frame.data[0];
    if (resp_code != 0xC0 && resp_code != 0xC2) {
        result.is_success = false;
        return false;
    }

    result.response_code = resp_code;
    result.log_index     = rx_frame.data[1]; // DB1 為感測器回應的 Log Index
    result.error_name    = getErrorDescription(result.log_index);

    // DB2..DB5 為 32-bit Big-Endian 的 數據值/計數器
    result.raw_value = (static_cast<uint32_t>(rx_frame.data[2]) << 24) |
                       (static_cast<uint32_t>(rx_frame.data[3]) << 16) |
                       (static_cast<uint32_t>(rx_frame.data[4]) << 8)  |
                       (static_cast<uint32_t>(rx_frame.data[5]));

    result.is_success = true;

    std::cout << "[Decode Error Response] Code: 0x" << std::hex << std::uppercase
              << static_cast<int>(result.response_code) << std::dec
              << ", Log Index: 0x" << std::hex << std::uppercase
              << static_cast<int>(result.log_index) << std::dec
              << " (" << result.error_name << ")"
              << ", Raw Value: " << result.raw_value
              << std::endl;
    return true;
}

// 高階通道配置 API
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

    if (is_little_endian) {
        frame.data[1] |= (1 << 6);
    }
    if (enable_cyclic) {
        frame.data[1] |= (1 << 1);
    }

    frame.data[2] = static_cast<uint8_t>((period_ms >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(period_ms & 0xFF);

    frame.description = "Config " + channelToString(channel) +
                        " [Cyclic: " + (enable_cyclic ? "ON" : "OFF") +
                        ", Period: " + std::to_string(period_ms) + "ms]";
    return frame;
}

// 高階過流門檻設定 API
CANCommandFrame IVTCommandBuilder::setPositiveThreshold(
    int16_t threshold_current_A,
    uint32_t cmd_id) {
    
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x35;
    const uint16_t encoded_threshold = static_cast<uint16_t>(threshold_current_A);
    frame.data[1] = static_cast<uint8_t>((encoded_threshold >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(encoded_threshold & 0xFF);
    frame.description = "Set Positive OCS Threshold to " + std::to_string(threshold_current_A) + " A";
    return frame;
}

// 基礎指令實作
CANCommandFrame IVTCommandBuilder::buildSetMode(uint8_t mode, uint8_t default_mode, uint32_t cmd_id) {
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x34;
    frame.data[1] = mode;
    frame.data[2] = default_mode;

    const std::string mode_str = (mode == 0x00) ? "STOP" : "RUN";
    frame.description = "Set Mode to " + mode_str;
    return frame;
}

CANCommandFrame IVTCommandBuilder::buildStoreCommand(uint32_t cmd_id) {
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x32;
    frame.description = "STORE Configuration to EEPROM";
    return frame;
}

CANCommandFrame IVTCommandBuilder::buildRestartCommand(uint32_t cmd_id) {
    CANCommandFrame frame{};
    frame.id = cmd_id;
    frame.dlc = 8;
    frame.data[0] = 0x34;
    frame.data[1] = 0x01;
    frame.data[2] = 0x01;
    frame.description = "RESTART Sensor";
    return frame;
}

// --- 核心必要流程 Function 實作 ---
std::vector<CANCommandFrame> IVTCommandBuilder::buildFullConfigSequence(
    const std::vector<CANCommandFrame>& custom_configs,
    uint32_t cmd_id) {
    
    std::vector<CANCommandFrame> sequence;

    // 1. 必要前置步驟：切換至 STOP Mode 以允許修改參數
    sequence.push_back(buildSetMode(0x00, 0x00, cmd_id));

    // 2. 插入使用者自訂的設定集
    for (const auto& config : custom_configs) {
        sequence.push_back(config);
    }

    // 3. 必要後置步驟 A：儲存所有設定至 EEPROM
    sequence.push_back(buildStoreCommand(cmd_id));

    // 4. 必要後置步驟 B：設定回 RUN Mode
    sequence.push_back(buildSetMode(0x01, 0x01, cmd_id));

    // 5. 必要後置步驟 C：將預設 RUN Mode 固化進 EEPROM
    sequence.push_back(buildStoreCommand(cmd_id));

    // 6. 必要後置步驟 D：重啟使新參數正式生效
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