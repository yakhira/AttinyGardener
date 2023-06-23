#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <Arduino.h>

#include "nrf24/projdefs.h"
#include "nrf24/nRF24L01.h"
#include "utils/utils.h"


#define CHANNEL 76 // 0-125

#define PUMP1 PB0
#define PUMP2 PB1
#define PUMP_DATA_DIRECTION DDRB
#define PUMP_WORK_TIME 120 //2 minutes

#define SENSOR1 PD3
#define SENSOR2 PD4
#define SENSOR3 PD5
#define SENSOR_DATA_DIRECTION DDRD
#define SENSOR_PORT PORTD

#define DELAY	3	// (== 250us * (n+1))
#define RETRY	10	// 12
#define FIVE_BYTES	5

volatile bool sleep = true;
volatile unsigned long pump_work_count = 0;

const uint8_t PIPE0_ADDRESS_PGM[] PROGMEM = "1Node";
const uint8_t PIPE1_ADDRESS_PGM[] PROGMEM = "2Node";

void receive() {
	uint8_t payload[NRF24_MAX_SIZE];

	nrf24_writeReg(W_REGISTER | EN_AA,      NRF24_PIPE_0);  // en autoack
	nrf24_writeReg(W_REGISTER | NRF_STATUS, NRF24_STATUS_CLEAR_ALL);  // clear status
	
	// set RX mode, enable CRC with 2 bytes, mask all IRQs, power on nRF radio (POWER DOWN ==> STANDBY-1)
	nrf24_writeReg(W_REGISTER | NRF_CONFIG,
		NRF24_CFG_PWR_UP | NRF24_CFG_RX_MODE | NRF24_CFG_CRC_2B | NRF24_CFG_CRC_EN | NRF24_CFG_IRQ_MASK_ALL);

	_delay_ms(DELAY_POWER_UP_MILLIS);

	uint8_t status = nrf24_readReg(R_REGISTER | NRF_STATUS);

	if( status & NRF24_STATUS_RX_DR ) {
		// read everything what has been received from RX FIFO
		// (make sure you read everything from RX FIFO otherwise RX FIFO will overflow eventually)
		// (if you are lazy, you can always do FLUSH_RX after each read but you loose unprocessed frames, RX has 3 FIFOs
		uint8_t bytes = nrf24_readReg(R_RX_PL_WID);

		nrf24_readRegs(R_RX_PAYLOAD, payload, bytes);	// read payload into a buffer
		nrf24_writeReg(W_REGISTER | NRF_STATUS, NRF24_STATUS_CLEAR_ALL); // clear received flag

		payload[bytes] = 0;

		if (strcmp((char*)payload, "P1_ON") == 0) {
			digitalWrite_B(PUMP1, HIGH);
			pump_work_count = 0;
			sleep = false;
		} else if (strcmp((char*)payload, "P2_ON") == 0) {
			digitalWrite_B(PUMP2, HIGH);
			pump_work_count = 0;
			sleep = false;
		} else if (strcmp((char*)payload, "P1_OFF") == 0) {
			digitalWrite_B(PUMP1, LOW);
			pump_work_count = 0;
			sleep = true;
		} else if (strcmp((char*)payload, "P2_OFF") == 0) {
			digitalWrite_B(PUMP2, LOW);
			pump_work_count = 0;
			sleep = true;
		}
	}

	nrf24_cmd(FLUSH_TX); // clean TX FIFOs thoroughly
	nrf24_cmd(FLUSH_RX); // clean RX FIFOs thoroughly

	nrf24_writeReg(W_REGISTER | NRF_CONFIG,
		NRF24_CFG_PWR_DOWN | NRF24_CFG_RX_MODE | NRF24_CFG_CRC_2B | NRF24_CFG_CRC_EN | NRF24_CFG_IRQ_MASK_ALL);
}

void wakeUp() {
	nrf24_enableCE();
	receive();
	nrf24_disableCE();

	if (!sleep) {
		if ((pump_work_count/140) > PUMP_WORK_TIME) {
			digitalWrite_B(PUMP1, LOW);
			digitalWrite_B(PUMP2, LOW);
			pump_work_count = 0;
			sleep = true;
		} else {
			pump_work_count++;
		}
	}
}

void setup() {
	PUMP_DATA_DIRECTION |= _BV(PUMP1);
	PUMP_DATA_DIRECTION |= _BV(PUMP2);

	SENSOR_DATA_DIRECTION &= ~_BV(SENSOR1);
	SENSOR_DATA_DIRECTION &= ~_BV(SENSOR2);
	SENSOR_DATA_DIRECTION &= ~_BV(SENSOR3);

	SENSOR_PORT |= _BV(SENSOR1); 
	SENSOR_PORT |= _BV(SENSOR2); 
	SENSOR_PORT |= _BV(SENSOR3); 

	nrf24_init(); // initialize radio (UNDEFINED ==> POWER ON RESET) 
	
	_delay_ms(20); // it takes 10.3ms to switch from POWER ON RESET ==> POWER DOWN
	
	nrf24_writeReg(W_REGISTER | RF_CH,    CHANNEL);
	nrf24_writeReg(W_REGISTER | RF_SETUP, NRF24_PWR_MAX | NRF24_SPEED_250kbps); // 0dbm TX power, 250kbps
	
	nrf24_writeReg(W_REGISTER | EN_RXADDR, NRF24_PIPE_0);	// enable RX in pipe 0 for ACK packet
	nrf24_writeReg(W_REGISTER | DYNPD,     NRF24_PIPE_0);   // enable dynamic payload in pipe 0	
	nrf24_writeReg(W_REGISTER | FEATURE,   NRF24_FEATURE_EN_DPL); // enable dynamic payload length
	
	// Target pipe 0 address from PROGMEM. Because we read value from PROGMEM, we have to add flag NRF24_PROGMEM_MASK to the size.
	nrf24_writeRegs(W_REGISTER | TX_ADDR, PIPE0_ADDRESS_PGM, FIVE_BYTES | NRF24_PROGMEM_MASK); 	
	// RX address on pipe 0 from PROGMEM. Because we read value from PROGMEM, we have to add flag NRF24_PROGMEM_MASK to the size.
	nrf24_writeRegs(W_REGISTER | RX_ADDR_P0, PIPE1_ADDRESS_PGM, FIVE_BYTES | NRF24_PROGMEM_MASK); 

	nrf24_cmd(FLUSH_TX); // clean TX FIFOs thoroughly
	nrf24_cmd(FLUSH_RX); // clean RX FIFOs thoroughly

	setSleep(SLEEP_250MS);
	attachInterrupt(0, wakeUp, LOW);
}

void loop(){
	if (sleep) {
		sleep_mode();
	}
}
