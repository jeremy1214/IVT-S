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