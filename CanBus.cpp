#include "CanBus.hpp"

/*
 * Calculated using http://www.bittiming.can-wiki.info/
 * 
 * CAN clock rate:	36MHz
 * Sample point:	87.5%
 * SJW:				1
 */
Can::BitrateConfig Can::canBitrateConfig[] = {
	{7,  1, 120},	//CAN_33K3BPS
	{13, 2, 45},	//CAN_50KBPS
	{15, 2, 20},	//CAN_100KBPS
	{13, 2, 18},	//CAN_125KBPS
	{13, 2, 9},		//CAN_250KBPS
	{15, 2, 4},		//CAN_500KBPS
	{15, 2, 2},		//CAN_1000KBPS
};

/**
 * Initializes CAN interface with provided bitrate on selected pins 
 * TODO: Refactor
 */
bool Can::CanBus::begin(Can::Bitrate bitrate, Can::Pinout pinout) {

	RCC->APB1ENR |= 0x2000000UL;	// Enable CAN clock 
	RCC->APB2ENR |= 0x1UL;			// Enable AFIO clock
	AFIO->MAPR   &= 0xFFFF9FFF;		// reset CAN remap

	if( pinout == Can::Pinout::RX_PA11_TX_PA12 ) {
		//todo reformat

		RCC->APB2ENR |= 0x4UL;           // Enable GPIOA clock
		GPIOA->CRH   &= ~(0xFF000UL);    // Configure PA12(0b0000) and PA11(0b0000)
		// 0b0000
		//   MODE=00(Input mode)
		//   CNF=00(Analog mode)

		GPIOA->CRH   |= 0xB8FFFUL;       // Configure PA12(0b1011) and PA11(0b1000)
		// 0b1011
		//   MODE=11(Output mode, max speed 50 MHz) 
		//   CNF=10(Alternate function output Push-pull
		// 0b1000
		//   MODE=00(Input mode)
		//   CNF=10(Input with pull-up / pull-down)

		GPIOA->ODR |= 0x1UL << 12;       // PA12 Upll-up

	} else if ( pinout == Can::Pinout::RX_PB8_TX_PB9 ) {
		//todo reformat
		AFIO->MAPR   |= 0x00004000;      // set CAN remap
		// CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)

		RCC->APB2ENR |= 0x8UL;           // Enable GPIOB clock
		GPIOB->CRH   &= ~(0xFFUL);       // Configure PB9(0b0000) and PB8(0b0000)
		// 0b0000
		//   MODE=00(Input mode)
		//   CNF=00(Analog mode)

		GPIOB->CRH   |= 0xB8UL;          // Configure PB9(0b1011) and PB8(0b1000)
		// 0b1011
		//   MODE=11(Output mode, max speed 50 MHz) 
		//   CNF=10(Alternate function output Push-pull
		// 0b1000
		//   MODE=00(Input mode)
		//   CNF=10(Input with pull-up / pull-down)

		GPIOB->ODR |= 0x1UL << 8;        // PB8 Upll-up

	} else if ( pinout == Can::Pinout::RX_PD0_TX_PD1 ) {
		//todo reformat
		AFIO->MAPR   |= 0x00005000;      // set CAN remap
		// CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)

		RCC->APB2ENR |= 0x20UL;          // Enable GPIOD clock
		GPIOD->CRL   &= ~(0xFFUL);       // Configure PD1(0b0000) and PD0(0b0000)
		// 0b0000
		//   MODE=00(Input mode)
		//   CNF=00(Analog mode)

		GPIOD->CRH   |= 0xB8UL;          // Configure PD1(0b1011) and PD0(0b1000)
		// 0b1000
		//   MODE=00(Input mode)
		//   CNF=10(Input with pull-up / pull-down)
		// 0b1011
		//   MODE=11(Output mode, max speed 50 MHz) 
		//   CNF=10(Alternate function output Push-pull

		GPIOD->ODR |= 0x1UL << 0;        // PD0 Upll-up
	}

	CAN1->MCR |= 0x1UL;                   // Require CAN1 to Initialization mode 
	while (!(CAN1->MSR & 0x1UL));         // Wait for Initialization mode

	//CAN1->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
	CAN1->MCR = 0x41UL;                   // Hardware initialization(With automatic retransmission)

	// Set bit rates 
	Can::BitrateConfig bitrateConfig = Can::canBitrateConfig[bitrate];

	CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
	CAN1->BTR |=  (((bitrateConfig.TS2-1) & 0x07) << 20) | (((bitrateConfig.TS1-1) & 0x0F) << 16) | ((bitrateConfig.BRP-1) & 0x1FF);

	// Configure Filters to default values
	CAN1->FMR  |=   0x1UL;                // Set to filter initialization mode
	CAN1->FMR  &= 0xFFFFC0FF;             // Clear CAN2 start bank

	// bxCAN has 28 filters.
	// These filters are used for both CAN1 and CAN2.
	// STM32F103 has only CAN1, so all 28 are used for CAN1
	CAN1->FMR  |= 0x1C << 8;              // Assign all filters to CAN1

	// Set filter 0
	// Single 32-bit scale configuration 
	// Two 32-bit registers of filter bank x are in Identifier Mask mode
	// Filter assigned to FIFO 0 
	// Filter bank register to all 0
	setFilter(0, 1, 0, 0, 0x0UL, 0x0UL);  //OK

	CAN1->FMR   &= ~(0x1UL);              // Deactivate initialization mode

	uint16_t TimeoutMilliseconds = 1000;
	CAN1->MCR   &= ~(0x1UL);              // Require CAN1 to normal mode 

	// Wait for normal mode
	// If the connection is not correct, it will not return to normal mode.
	for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
		if ((CAN1->MSR & 0x1UL) == 0) return true;
		delayMicroseconds(1000);
	}

	return false;
}

/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 * 
 * TODO: Refactor
 */
void Can::CanBus::setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
	if (index > 27) return;
	
	//Deactivate filter
	CAN1->FA1R &= ~(0x1UL<<index);
	
	//Set filter to dual 16-bit or single 32-bit config
	(scale == 0) ? CAN1->FS1R &= ~(0x1UL<<index) : CAN1->FS1R |= (0x1UL<<index);

	//Set filter to mask / list mode
	(mode == 0) ? CAN1->FM1R &= ~(0x1UL<<index) : CAN1->FM1R |= (0x1UL<<index);

	//Set filter assigned to FIFO 0 / 1
	(fifo == 0) ? CAN1->FFA1R &= ~(0x1UL<<index) : CAN1->FFA1R |= (0x1UL<<index);

	//Set filter bank registers 1 & 2
	CAN1->sFilterRegister[index].FR1 = bank1;
	CAN1->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

	//Activate filter
	CAN1->FA1R |= (0x1UL<<index);               
}

bool Can::CanBus::checkReceive() {
	//Check for pending FIFO 0 messages
	return CAN1->RF0R & 0x3UL;
}

/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 * TODO: Refactor
 */
void Can::CanBus::receive(Can::Frame* frame) {
	uint32_t id = CAN1->sFIFOMailBox[0].RIR;

	if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
		frame->format = FrameFormat::Standard;
		frame->id = (CAN_STD_ID_MASK & (id >> 21U));
	} else {                               // Extended frame format
		frame->format = FrameFormat::Extended;
		frame->id = (CAN_EXT_ID_MASK & (id >> 3U));
	}

	if ((id & STM32_CAN_RIR_RTR) == 0) { // Data frame
		frame->type = FrameType::Data;
	} else {                               // Remote frame
		frame->type = FrameType::Remote;
	}


	frame->length = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;

	frame->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
	frame->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
	frame->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
	frame->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
	frame->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
	frame->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
	frame->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
	frame->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);

	// Release FIFO 0 output mailbox.
	// Make the next incoming message available.
	CAN1->RF0R |= 0x20UL;
}

/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @params frame - CAN message structure for transmission
 * 
 * TODO: Refactor
 */
bool Can::CanBus::send(Can::Frame* frame) {

	//Construct arbitration field + IDE
	uint32_t out = (frame->format == Standard) 
		? ((frame->id & CAN_STD_ID_MASK) << 21U)
		: ((frame->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
	if (frame->type == Remote) out |= STM32_CAN_TIR_RTR;

	//Set control field
	CAN1->sTxMailBox[0].TDTR &= ~(0xF);
	CAN1->sTxMailBox[0].TDTR |= frame->length & 0xFUL;

	//Construct first 4 bytes of data field
	CAN1->sTxMailBox[0].TDLR = (
		((uint32_t) frame->data[3] << 24) |
		((uint32_t) frame->data[2] << 16) |
		((uint32_t) frame->data[1] <<  8) |
		((uint32_t) frame->data[0]		)
	);

	//Construct last 4 bytes of data field
	CAN1->sTxMailBox[0].TDHR = (
		((uint32_t) frame->data[7] << 24) |
		((uint32_t) frame->data[6] << 16) |
		((uint32_t) frame->data[5] <<  8) |
		((uint32_t) frame->data[4]		)
	);

	//Send
	CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

	//Wait until the mailbox is empty
	volatile int attempt = 0;
	while(CAN1->sTxMailBox[0].TIR & 0x1UL && attempt++ < 1000000);

	//Detect whether mailbox has been emptied
	if (CAN1->sTxMailBox[0].TIR & 0x1UL) return false;

	//Success
	return true;
}

/**
 * Returns whether frame is of standard type
 */
bool Can::Frame::isStandardFrame() {
	return (this->type == FrameFormat::Standard);
}

/**
 * Returns whether frame is extended
 */
bool Can::Frame::isExtendedFrame() {
	return (this->type == FrameFormat::Extended);
}

/**
 * Returns whether frame is a data frame
 */
bool Can::Frame::isDataFrame() {
	return (this->format == Can::FrameType::Data);
}

/**
 * Returns whether frame is a remote frame
 */
bool Can::Frame::isRemoteFrame() {
	return (this->format == Can::FrameType::Remote);
}

/**
 * Prints frame details
 */
void Can::Frame::print() {
	Serial.print("Frame (");
	Serial.print(this->isStandardFrame() ? "Standard" : "Extended");
	Serial.print(") ");

	Serial.print("Id: 0x");
	if(this->isExtendedFrame()) {
		if (this->id < 0x10000000) Serial.print("0");
		if (this->id < 0x1000000) Serial.print("00");
		if (this->id < 0x100000) Serial.print("000");
		if (this->id < 0x10000) Serial.print("0000");
	} else {
		if (this->id < 0x100) Serial.print("0");
		if (this->id < 0x10) Serial.print("00");
	}

	Serial.print(this->id, HEX);
	Serial.print(" ");

	if(this->isRemoteFrame()) {
		Serial.println("Remote frame");
		return;
	}

	Serial.print("[");
	Serial.print(this->length);
	Serial.print("] ");

	for(uint8_t i = 0 ; i < this->length; i++ ) {
		Serial.print("0x"); 
		if(this->data[i] < 0x10) Serial.print("0");
		Serial.print(this->data[i], HEX);
		Serial.print(' '); 
	}
	Serial.println();
}