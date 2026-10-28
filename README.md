# EM-IVT

這個專案提供 IVT-S 感測器的 CAN 封包解碼與設定指令建構功能：

- `EM_IVT_Decode.h/.cpp`：解碼 IVT-S 常態刷新回傳資料，並可將結果輸出至終端機。
- `Setting_EM_IVT.h/.cpp`：建立通道刷新、過流保護及完整設定流程所需的 CAN 指令。

這些模組只負責解析及建立 CAN frame，不包含 CAN 介面初始化、封包傳送或接收功能。請由應用程式搭配 CAN 裝置／函式庫收發封包。

## 需求

- 支援 C++11 或更新版本的編譯器。
- 使用時將對應的 `.cpp` 檔加入應用程式的建置設定。

## 解碼回傳資料

使用 `decodeIVTFrame` 解碼 CAN frame。函式成功時回傳 `true` 並填入 `IVTDecodedData`；CAN ID 不在 `0x521`～`0x528` 範圍內、DLC 小於 6，或 MuxID 不支援時回傳 `false`。

常態刷新封包使用 DB0～DB5：

| 位元組 | 內容 |
| --- | --- |
| DB0 | MuxID |
| DB1 低 4 位元 | 訊息計數器（0～15） |
| DB1 高 4 位元 | 狀態旗標 |
| DB2～DB5 | Big-endian、有正負號的 32 位元原始值 |

支援的 MuxID 與物理量換算如下：

| MuxID | 通道 | 換算方式 | 單位 |
| --- | --- | --- | --- |
| `0x00` | 電流 Current (I) | 原始值 ÷ 1000 | A |
| `0x01` | 電壓 Voltage 1 (U1) | 原始值 ÷ 1000 | V |
| `0x02` | 電壓 Voltage 2 (U2) | 原始值 ÷ 1000 | V |
| `0x03` | 電壓 Voltage 3 (U3) | 原始值 ÷ 1000 | V |
| `0x04` | 溫度 Temperature (T) | 原始值 ÷ 10 | °C |
| `0x05` | 功率 Power | 原始值 | W |
| `0x06` | 電量累計 | 原始值 | As |
| `0x07` | 能量累計 | 原始值 | Wh |

狀態旗標使用 DB1 高 4 位元，對應 `IVTDecodedData::StatusFlag`：

| 旗標值 | 意義 |
| --- | --- |
| `0x01` | OCS 過流保護觸發 |
| `0x02` | 超出範圍／量測錯誤 |
| `0x04` | 一個或多個通道發生錯誤 |
| `0x08` | 系統錯誤 |

範例：

```cpp
#include "EM_IVT_Decode.h"

#include <iostream>

int main() {
    CANFrame frame{};
    frame.id = 0x521;
    frame.dlc = 6;
    frame.data[0] = 0x00; // MuxID：電流
    frame.data[1] = 0x03; // 狀態旗標為 0，訊息計數器為 3
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8; // 原始值 1000，換算後為 1.000 A

    IVTDecodedData decoded{};
    if (!decodeIVTFrame(frame, decoded)) {
        std::cerr << "無法解碼此 CAN frame\n";
        return 1;
    }

    printIVTData(decoded);
    return 0;
}
```

`printIVTData` 會輸出通道、MuxID、訊息計數器、原始值、換算後的物理量及狀態旗標。

## 建立設定指令

`IVTCommandBuilder` 提供高階 API，並以 `IVTChannel` 列舉表示通道，預設指令 CAN ID 為 `0x411`。

### 設定通道刷新

`configChannel` 可設定通道是否啟用循環刷新、刷新週期（毫秒）及資料位元組序。預設為 Big-endian，且使用預設指令 ID。

```cpp
#include "Setting_EM_IVT.h"

#include <iostream>

int main() {
    CANCommandFrame current_config = IVTCommandBuilder::configChannel(
        IVTChannel::Current,
        true,   // 啟用循環刷新
        100     // 刷新週期：100 ms
    );

    printCANFrame(current_config);
    return 0;
}
```

`is_little_endian` 設為 `true` 時會在指令中設定 little-endian 旗標；`cmd_id` 可用來覆寫預設 CAN ID。

### 設定正向過流保護門檻

`setPositiveThreshold` 接受以安培為單位的 `int16_t` 門檻值：

```cpp
CANCommandFrame threshold_config =
    IVTCommandBuilder::setPositiveThreshold(100); // 100 A
```

### 建立完整設定流程

將個別設定 frame 放入 `custom_configs`，再呼叫 `buildFullConfigSequence`。函式會按照以下順序建立 frame：

1. 切換至 STOP。
2. 插入自訂設定指令。
3. 儲存設定至 EEPROM。
4. 切換至 RUN。
5. 再次儲存設定至 EEPROM。
6. 重啟感測器。

```cpp
#include "Setting_EM_IVT.h"

#include <vector>

int main() {
    std::vector<CANCommandFrame> custom_configs;
    custom_configs.push_back(IVTCommandBuilder::configChannel(
        IVTChannel::Current, true, 100));
    custom_configs.push_back(IVTCommandBuilder::setPositiveThreshold(100));

    const std::vector<CANCommandFrame> sequence =
        IVTCommandBuilder::buildFullConfigSequence(custom_configs);

    for (const CANCommandFrame& frame : sequence) {
        printCANFrame(frame);
        // 在此使用 CAN 裝置／函式庫傳送 frame。
    }

    return 0;
}
```

`buildFullConfigSequence` 只負責組合 frame，不會自行傳送指令。若使用自訂 `cmd_id`，請將相同 ID 傳給完整流程及各個自訂設定的建構函式，例如：

```cpp
const uint32_t custom_cmd_id = 0x412;
CANCommandFrame config = IVTCommandBuilder::configChannel(
    IVTChannel::Current, true, 100, false, custom_cmd_id);
const std::vector<CANCommandFrame> sequence =
    IVTCommandBuilder::buildFullConfigSequence({config}, custom_cmd_id);
```

各自訂 frame 的 ID 不會由完整流程自動改寫。

## 編譯範例

使用 GCC／Clang 編譯上方範例時，請將所需模組的 `.cpp` 一併編譯。例如解碼範例：

```sh
g++ -std=c++11 -Wall -Wextra -pedantic main.cpp EM_IVT_Decode.cpp -o ivt_decode
```

設定範例則改為：

```sh
g++ -std=c++11 -Wall -Wextra -pedantic main.cpp Setting_EM_IVT.cpp -o ivt_setting
```
