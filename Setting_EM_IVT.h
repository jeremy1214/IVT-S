#pragma once

#include <cstdint>
#include <string>
#include <vector>

// 1. 手冊 8.5 節：Log / Error 請求類型
enum class IVTLogType : uint8_t {
    MeasurementErrors = 0x40, // 讀取量測錯誤 (DB0 = 0x40)
    SystemErrors      = 0x41, // 讀取系統錯誤 (DB0 = 0x41)
    OverallLog        = 0x42, // 讀取 Overall 統計數據 (DB0 = 0x42)
    SinceResetLog     = 0x43  // 讀取 Since Reset 統計數據 (DB0 = 0x43)
};

// 2. 解譯 Get Error / Log Response 的結果結構
struct IVTErrorLogResult {
    uint8_t     response_code; // 0x80 (Measurement), 0x81 (System), 0x82 (Overall), 0x83 (SinceReset)
    uint8_t     log_index;     // 查詢的 Index/Sub-code
    std::string description;   // 名稱描述
    uint64_t    raw_value;     // DB2..DB7 解析出來的數據 (可長達 6 頁位元組)
    bool        is_success;    // 是否解析成功
};

// 3. 通道名稱 (Mux ID)
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

    // --- 發送 Get Error / Log 請求 ---
    static CANCommandFrame buildGetErrorRequest(
        IVTLogType log_type,
        uint8_t log_index,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 解譯感測器回傳的 Response (CAN ID: 0x511) ---
    static bool decodeErrorResponse(
        const CANCommandFrame& response_frame,
        IVTErrorLogResult& result);

    // --- 高階通道配置 API ---
    static CANCommandFrame configChannel(
        IVTChannel channel,
        bool enable_cyclic,
        uint16_t period_ms,
        bool is_little_endian = false,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 過流保護門檻設定 API ---
    // threshold_A: 觸發門檻, reset_threshold_A: 復歸門檻
    static CANCommandFrame setPositiveThreshold(
        int16_t threshold_A,
        int16_t reset_threshold_A = 0,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    static CANCommandFrame setNegativeThreshold(
        int16_t threshold_A,
        int16_t reset_threshold_A = 0,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 底層基礎命令建構器 ---
    static CANCommandFrame buildSetMode(uint8_t mode, uint8_t default_mode, uint32_t cmd_id = DEFAULT_CMD_ID);
    static CANCommandFrame buildStoreCommand(uint32_t cmd_id = DEFAULT_CMD_ID);
    
    // 【修正】手冊 8.4 節標準 RESTART 命令字碼為 0x3F
    static CANCommandFrame buildRestartCommand(uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 核心：自動包裝完整配置流程 ---
    static std::vector<CANCommandFrame> buildFullConfigSequence(
        const std::vector<CANCommandFrame>& custom_configs,
        uint32_t cmd_id = DEFAULT_CMD_ID);
};

void printCANFrame(const CANCommandFrame& frame);