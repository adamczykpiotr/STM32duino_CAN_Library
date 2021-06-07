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
Frame tx =  { 0x7FF, 3, {0x01, 0x02, 0x03} };

void loop() {

	if(can0.checkReceive()) {
		can0.receive(&rx);
        rx.print();
	}

	tx = { 0x17FF, 3, {0x01, 0x02, 0x03}, FrameFormat::Extended, FrameType::Data };

    can0.send(&tx);

}