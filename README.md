# EM-IVT

This project contains C++ utilities for decoding IVT-S periodic measurement frames and constructing CAN command frames for sensor configuration.

Included files:

- `EM_IVT_Decode.h` / `EM_IVT_Decode.cpp`: decode live measurement frames from the IVT-S sensor
- `Setting_EM_IVT.h` / `Setting_EM_IVT.cpp`: build request/command frames for configuration, status queries, and setup flows
- `README.md`: usage notes and examples

These modules do not contain any CAN bus driver or hardware access layer. They only parse frames and build payloads. You still need to send/receive data through a CAN interface library or hardware adapter in your application.

## Requirements

- C++11 or later compiler
- Add the relevant `.cpp` file to your project build when using the library

## API Summary

### Decoding live measurement data

`decodeIVTFrame(const CANFrame&, IVTDecodedData&)`

- Valid CAN IDs: `0x521` through `0x528`
- Minimum DLC: `6`
- Supported Mux IDs: `0x00` to `0x07`
- Returns `false` for unsupported IDs, short frames, or unsupported Mux IDs

`printIVTData(const IVTDecodedData&)`

- Prints channel name, message count, raw value, converted value, and status summary

### Building command frames

`IVTCommandBuilder::buildGetErrorRequest(...)`

- Generates the request to query error/log data
- Supported log types:
  - `IVTLogType::SinceReset` (`0x40`)
  - `IVTLogType::Overall` (`0x42`)
- Default CAN command ID: `0x411`

`IVTCommandBuilder::decodeErrorResponse(...)`

- Parses the response packet from the sensor
- Accepts response codes:
  - `0xC0` for SinceReset
  - `0xC2` for Overall

`IVTCommandBuilder::configChannel(...)`

- Builds the channel refresh configuration frame
- Parameters:
  - channel
  - enable_cyclic
  - period_ms
  - is_little_endian
  - cmd_id

`IVTCommandBuilder::setPositiveThreshold(...)`

- Builds the positive overcurrent threshold frame
- Value is given in amps as `int16_t`

`IVTCommandBuilder::buildFullConfigSequence(...)`

- Automatically creates the full sequence:
  1. STOP
  2. custom config(s)
  3. STORE
  4. RUN
  5. STORE
  6. RESTART

`printCANFrame(const CANCommandFrame&)`

- Prints the CAN ID, DLC, and payload bytes

## Decode Format

The periodic refresh packet uses DB0 to DB5.

| Byte | Description |
| --- | --- |
| DB0 | MuxID |
| DB1 low 4 bits | Message counter (0–15) |
| DB1 high 4 bits | Status flags |
| DB2–DB5 | 32-bit signed raw value in big-endian format |

### Supported Mux IDs and conversion rules

| MuxID | Channel | Conversion | Unit |
| --- | --- | --- | --- |
| `0x00` | Current (I) | raw / 1000 | A |
| `0x01` | Voltage 1 (U1) | raw / 1000 | V |
| `0x02` | Voltage 2 (U2) | raw / 1000 | V |
| `0x03` | Voltage 3 (U3) | raw / 1000 | V |
| `0x04` | Temperature (T) | raw / 10 | °C |
| `0x05` | Power | raw | W |
| `0x06` | Ampere-seconds (As) | raw | As |
| `0x07` | Watt-hours (Wh) | raw | Wh |

### Status bits

The code stores status flags as a bitmask in `IVTDecodedData::status_flags` and checks them individually using the enum values:

| Value | Meaning |
| --- | --- |
| `0x01` | OCS overcurrent protection triggered |
| `0x02` | Out-of-range / measurement error |
| `0x04` | Error on one or more channels |
| `0x08` | System error |

These bits are checked by `printIVTData()` using bitwise operations. The status field can contain more than one flag at the same time.

## Example: Decode a measurement packet

```cpp
#include "EM_IVT_Decode.h"

#include <iostream>

int main() {
    CANFrame frame{};
    frame.id = 0x521;
    frame.dlc = 6;
    frame.data[0] = 0x00; // MuxID: Current
    frame.data[1] = 0x03; // message counter = 3, status = 0
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8; // raw = 1000 => 1.000 A

    IVTDecodedData decoded{};
    if (!decodeIVTFrame(frame, decoded)) {
        std::cerr << "Unable to decode this CAN frame\n";
        return 1;
    }

    printIVTData(decoded);
    return 0;
}
```

## Example: Build a configuration command

```cpp
#include "Setting_EM_IVT.h"

#include <iostream>

int main() {
    CANCommandFrame current_config = IVTCommandBuilder::configChannel(
        IVTChannel::Current,
        true,
        100
    );

    printCANFrame(current_config);
    return 0;
}
```

## Example: Query error/log history

```cpp
#include "Setting_EM_IVT.h"

#include <iostream>

int main() {
    CANCommandFrame req = IVTCommandBuilder::buildGetErrorRequest(
        IVTLogType::SinceReset,
        0x02
    );

    printCANFrame(req);
    return 0;
}
```

## Example: Build a full configuration sequence

```cpp
#include "Setting_EM_IVT.h"

#include <vector>

int main() {
    std::vector<CANCommandFrame> custom_configs;
    custom_configs.push_back(IVTCommandBuilder::configChannel(
        IVTChannel::Current,
        true,
        100
    ));
    custom_configs.push_back(IVTCommandBuilder::setPositiveThreshold(100));

    const auto sequence = IVTCommandBuilder::buildFullConfigSequence(custom_configs);

    for (const auto& frame : sequence) {
        printCANFrame(frame);
        // Send the frame with your CAN transport layer here.
    }

    return 0;
}
```

## Build examples

Use your own example source file instead of `main.cpp` when compiling. For example:

```sh
g++ -std=c++11 -Wall -Wextra -pedantic example_decode.cpp EM_IVT_Decode.cpp -o ivt_decode
```

```sh
g++ -std=c++11 -Wall -Wextra -pedantic example_setting.cpp Setting_EM_IVT.cpp -o ivt_setting
```

If you need a custom CAN ID, pass `cmd_id` to the builder functions and to `buildFullConfigSequence` so the resulting packet IDs remain consistent.

## Notes

- `result.status_flags` is stored as an enum value but is interpreted as a bitmask in the implementation.
- `buildFullConfigSequence()` does not send frames automatically; it only returns a ready-to-send sequence.
- `decodeErrorResponse()` expects the sensor response frame to have the correct response code and enough data bytes to decode the 32-bit value.
