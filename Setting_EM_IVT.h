#pragma once

#include <cstdint>
#include <string>
#include <vector>

// 1. 定義語意化的通道名稱，不需要人工記憶 MuxID
enum class IVTChannel : uint8_t {
    Current  = 0x00, // 電流 I
    Voltage1 = 0x01, // 電壓 U1
    Voltage2 = 0x02, // 電壓 U2
    Voltage3 = 0x03, // 電壓 U3
    Temp     = 0x04, // 溫度 T
    Power    = 0x05, // 功率 W
    As       = 0x06, // 安培秒 As
    Wh       = 0x07  // 瓦時 Wh
};

struct CANCommandFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    std::string description;
};

class IVTCommandBuilder {
public:
    static constexpr uint32_t DEFAULT_CMD_ID = 0x411;

    // --- 高階設定 API (免記指令碼與 MuxID) ---

    // 1. 設定通道刷新週期與開關 (使用 IVTChannel)
    static CANCommandFrame configChannel(
        IVTChannel channel,
        bool enable_cyclic,
        uint16_t period_ms,
        bool is_little_endian = false,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // 2. 設定正向過流保護門檻 (安培 A)
    static CANCommandFrame setPositiveThreshold(
        int16_t threshold_current_A,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 底層基礎命令建構器 ---
    static CANCommandFrame buildSetMode(uint8_t mode, uint8_t default_mode, uint32_t cmd_id = DEFAULT_CMD_ID);
    static CANCommandFrame buildStoreCommand(uint32_t cmd_id = DEFAULT_CMD_ID);
    static CANCommandFrame buildRestartCommand(uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 核心：自動包裝必要流程的 Function ---
    // 自動生成：STOP -> [custom_configs...] -> STORE -> RUN -> STORE -> RESTART
    static std::vector<CANCommandFrame> buildFullConfigSequence(
        const std::vector<CANCommandFrame>& custom_configs,
        uint32_t cmd_id = DEFAULT_CMD_ID);
};

void printCANFrame(const CANCommandFrame& frame);