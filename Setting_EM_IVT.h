#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <fstream>

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
    uint64_t    raw_value;     // DB2..DB7 解析出來的數據
    bool        is_success;    // 是否解析成功
};

// 3. 一般設定指令 Response 的結果結構 (例如 0x94 SET_MODE, 0xB2 STORE, 0xBF RESTART, 0xA0~0xA7 Config)
struct IVTCommandResponseResult {
    uint8_t     response_code; // 例如 0x94, 0xB2, 0xBF, 0xA0~0xA7 等
    uint8_t     result_status; // 0x00: SUCCESS, 非 0: Error Code
    std::string description;   // 語意描述
    bool        is_success;    // 是否成功
};

// 4. 週期性數據發送 (Cyclic Result Message) 解析結果結構
struct IVTDataResult {
    uint8_t     mux_id;         // 通道 ID (0x00: Current, 0x01: U1, ..., 0x07: Wh)
    std::string channel_name;   // 通道名稱
    uint8_t     status_flags;   // DB1 狀態位元 (包含 Error 位元、System 狀態等)
    int64_t     raw_value;      // 原始 32-bit 整數數據 (有符號)
    double      physical_value; // 計算後的物理值 (A, V, °C, W, As, Wh)
    std::string unit;           // 物理單位
    bool        is_success;     // 是否解析成功
};

// 5. 通道名稱 (Mux ID)
enum class IVTChannel : uint8_t {
    Current  = 0x00, // 電流 I (521h)
    Voltage1 = 0x01, // 電壓 U1 (522h)
    Voltage2 = 0x02, // 電壓 U2 (523h)
    Voltage3 = 0x03, // 電壓 U3 (524h)
    Temp     = 0x04, // 溫度 T (525h)
    Power    = 0x05, // 功率 W (526h)
    As       = 0x06, // 安培秒 As (527h)
    Wh       = 0x07  // 瓦時 Wh (528h)
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

    // --- 解譯感測器回傳的 Response (CAN ID: 0x511 / 0x521~0x528) ---
    // 解譯 Error / Log Response (0x80 ~ 0x83)
    static bool decodeErrorResponse(
        const CANCommandFrame& response_frame,
        IVTErrorLogResult& result);

    // 解譯設定指令 Response (0x90 ~ 0xBF, 如 SET_MODE 回傳 0x94, STORE 回傳 0xB2, RESTART 回傳 0xBF)
    static bool decodeCommandResponse(
        const CANCommandFrame& response_frame,
        IVTCommandResponseResult& result);

    // 【新增】解譯感測器週期性發送的量測數據 (CAN ID 0x521 ~ 0x528)
    static bool decodeDataResponse(
        const CANCommandFrame& rx_frame,
        IVTDataResult& result);

    // --- 高階通道配置 API ---
    static CANCommandFrame configChannel(
        IVTChannel channel,
        bool enable_cyclic,
        uint16_t period_ms,
        bool is_little_endian = false,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 過流保護門檻設定 API ---
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
    static CANCommandFrame buildRestartCommand(uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- 核心：自動包裝完整配置流程 ---
    static std::vector<CANCommandFrame> buildFullConfigSequence(
        const std::vector<CANCommandFrame>& custom_configs,
        uint32_t cmd_id = DEFAULT_CMD_ID);

    // --- CAN 傳輸 Log 檔案寫入工具 ---
    static void logCANMessage(
        std::ofstream& log_file,
        const std::string& direction, // "TX" 或 "RX"
        uint32_t can_id,
        uint8_t dlc,
        const uint8_t* data,
        const std::string& description = "");
};

void printCANFrame(const CANCommandFrame& frame);