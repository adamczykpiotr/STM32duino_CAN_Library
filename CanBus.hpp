#pragma once

#include <Arduino.h>

#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU

namespace Can {

	enum Pinout {
		RX_PA11_TX_PA12,	// 36/48/64/100/144 pin packages
		RX_PB8_TX_PB9,		// 48/64/100/144 pin packages
		RX_PD0_TX_PD1,		// 100/144 pin packages
	};

	enum Bitrate {
		CAN_33K3BPS,
		CAN_50KBPS,
		CAN_100KBPS,
		CAN_125KBPS,
		CAN_250KBPS,
		CAN_500KBPS,
		CAN_1000KBPS,
	};

	class BitrateConfig {
	public:
		uint8_t TS1;
		uint8_t TS2;
		uint8_t BRP;
	};
	extern Can::BitrateConfig canBitrateConfig[];

	enum FrameFormat {
		Standard,
		Extended, 
	};

	enum FrameType {
		Data,
		Remote,
	};

	class Frame {
	public:
		uint32_t id;
		uint8_t length;
		uint8_t data[8];
		FrameFormat format;
		FrameType type;

		bool isStandardFrame();
		bool isExtendedFrame();
		bool isDataFrame();
		bool isRemoteFrame();
		void print();
	};

	class CanBus {
	public:
		bool begin(Bitrate bitrate, Pinout pinout);
		void setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);
		bool checkReceive();

		void receive(Frame* frame);
		bool send(Frame* frame);
	};
}