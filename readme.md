# STM32duino_CAN_Library
CanBus library for STM32F103 running on STM32duino, tested on F103C8T6 variant.
Key advantages:
* Object-oriented programming style
* Support for inline Can::Frame initialization
* Support for 33.3KBPS CAN (GMLAN/SWCAN)

Inner workings based on [nopnop2002](https://github.com/nopnop2002/Arduino-STM32-CAN/tree/master/stm32f103)'s sketch.

## Supported board / Can-Bus parameters:

STM32F103 pinouts:
```cpp
enum Can::Pinout {
  RX_PA11_TX_PA12,  // 36/48/64/100/144 pin packages
  RX_PB8_TX_PB9,    // 48/64/100/144 pin packages
  RX_PD0_TX_PD1,    // 100/144 pin packages
};
```

Bitrates:

```cpp
enum Can::Bitrate {
  CAN_33K3BPS,
  CAN_50KBPS,
  CAN_100KBPS,
  CAN_125KBPS,
  CAN_250KBPS,
  CAN_500KBPS,
  CAN_1000KBPS,
};
```

## Can-Bus frame:

```cpp
class Can::Frame {
public:
  uint32_t id;
  uint8_t length;
  uint8_t data[8];
  FrameFormat format;
  FrameType type;
}
```
Types and formats:

```cpp
enum Can::FrameFormat {
  Standard,
  Extended, 
};

enum Can::FrameType {
  Data,
  Remote,
};
```

## Example usage:

```cpp
#include "CanBus.hpp"
using namespace Can;
CanBus can0;

void setup() {
  Serial.begin(9600);

  bool ret = can0.begin(Bitrate::CAN_33K3BPS, Pinout::RX_PB8_TX_PB9);
  Serial.println(ret ? "BOOT OK" : "BOOT FAILURE");
  if (!ret) while(true);
}

Frame rx;
Frame tx = { 0x7FF, 3, {0x01, 0x02, 0x03} };

void loop() {

  if(can0.checkReceive()) {
    can0.receive(&rx);
    rx.print();
  }

  tx = { 0x17FF, 3, {0x01, 0x02, 0x03}, FrameFormat::Extended, FrameType::Data };
  can0.send(&tx);
}
```
