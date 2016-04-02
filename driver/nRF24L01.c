/*
 * nRF24L01.c
 *
 * Created: 01.11.2013 15:09:29
 *  Author: Vadim Kulakov, vad7 @ yahoo.com
 */

#include "nrf24l01.h"


uint8_t ICACHE_FLASH_ATTR SPI_WriteReadByte(uint8_t data) {
	USIDR = data;
	USISR = (1<<USIOIF); // clear overflow flag
	do {
		USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	} while((USISR & (1<<USIOIF)) == 0);
	return USIDR;
}

uint8_t ICACHE_FLASH_ATTR NRF24_ReadRegister(uint8_t reg)
{
	NRF24_SET_CSN_LOW;
	SPI_WriteReadByte(NRF24_CMD_R_REGISTER | (NRF24_CMD_REGISTER_MASK & reg));
	uint8_t result = SPI_WriteReadByte(NRF24_CMD_NOP);
	NRF24_SET_CSN_HI;
	return result;
}

void ICACHE_FLASH_ATTR NRF24_ReadArray(uint8_t cmd, uint8_t *array, uint8_t len)
{
	NRF24_SET_CSN_LOW;
	SPI_WriteReadByte(cmd);
	while(len-- > 0) *array++ = SPI_WriteReadByte(NRF24_CMD_NOP);
	NRF24_SET_CSN_HI;
}

void ICACHE_FLASH_ATTR NRF24_WriteByte(uint8_t cmd, uint8_t value)
{
	NRF24_SET_CSN_LOW;
	SPI_WriteReadByte(cmd);
	SPI_WriteReadByte(value);
	NRF24_SET_CSN_HI;
}

void ICACHE_FLASH_ATTR NRF24_WriteArray(int8_t cmd, uint8_t *array, uint8_t len)
{
	NRF24_SET_CSN_LOW;
	SPI_WriteReadByte(cmd);
	while(len-- > 0) SPI_WriteReadByte(*array++);
	NRF24_SET_CSN_HI;
}

uint8_t ICACHE_FLASH_ATTR NRF24_SendCommand(uint8_t cmd) // Send command & receive status
{
	NRF24_SET_CSN_LOW;
	uint8_t result = SPI_WriteReadByte(cmd);
	NRF24_SET_CSN_HI;
	return result;
}

void ICACHE_FLASH_ATTR NRF24_SetMode(uint8_t mode) // Set mode in CONFIG reg
{
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_CONFIG, NRF24_CONFIG | (1<<NRF24_BIT_PWR_UP) | mode);
	if(mode & NRF24_ReceiveMode) { // Receive mode
		//NRF24_SendCommand(NRF24_CMD_FLUSH_RX);
		NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_STATUS, (1<<NRF24_BIT_RX_DR) | (1<<NRF24_BIT_TX_DS) | (1<<NRF24_BIT_MAX_RT)); // clear status
		NRF24_SET_CE_HI; // start receiving
	}
}

uint8_t ICACHE_FLASH_ATTR NRF24_Receive(uint8_t *payload) // Receive in payload, return data pipe number + 1 if success
{
	uint8_t pipe = 0, st;
	if((st = NRF24_SendCommand(NRF24_CMD_NOP)) & (1<<NRF24_BIT_RX_DR))
	{
		NRF24_ReadArray(NRF24_CMD_R_RX_PAYLOAD, payload, NRF24_PAYLOAD_LEN);
		NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_STATUS, (1<<NRF24_BIT_RX_DR) | (1<<NRF24_BIT_TX_DS) | (1<<NRF24_BIT_MAX_RT)); // clear status
		pipe = ((st >> NRF24_BIT_RX_P_NO) & 0b111) + 1;
	}
	return pipe;
}

uint8_t ICACHE_FLASH_ATTR NRF24_Transmit(uint8_t *payload) // Transmit payload, return 0 if success, 1 - max retransmit count reached, 2 - module not response.
{
	NRF24_SET_CE_LOW;
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_STATUS, (1<<NRF24_BIT_RX_DR) | (1<<NRF24_BIT_TX_DS) | (1<<NRF24_BIT_MAX_RT)); // clear status
	NRF24_SendCommand(NRF24_CMD_FLUSH_TX);
	NRF24_WriteArray(NRF24_CMD_W_TX_PAYLOAD, payload, NRF24_PAYLOAD_LEN);
	NRF24_SET_CE_HI; // Start transmission
	uint8_t st = 0, i;
	for(i = 1; i != 0; i++)
	{
		Delay10us(10);
		st = NRF24_SendCommand(NRF24_CMD_NOP);
		if(st & ((1<<NRF24_BIT_MAX_RT) | (1<<NRF24_BIT_TX_DS))) break; // stop if sent or max retransmit reached
	}
	return i == 0 ? 2 : !(st & (1<<NRF24_BIT_TX_DS));
}

uint8_t ICACHE_FLASH_ATTR NRF24_SetAddresses(uint8_t addr_LSB) // Set addresses: NRF24_BASE_ADDR + addr_LSB, return 1 if success
{
	NRF24_Buffer[0] = addr_LSB;
	memcpy_P(NRF24_Buffer + 1, NRF24_BASE_ADDR, sizeof(NRF24_BASE_ADDR)/sizeof(NRF24_BASE_ADDR[0]));
	NRF24_WriteArray(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0, NRF24_Buffer, NRF24_ADDRESS_LEN);
	NRF24_WriteArray(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR, NRF24_Buffer, NRF24_ADDRESS_LEN);
	NRF24_ReadArray(NRF24_CMD_R_REGISTER | NRF24_REG_TX_ADDR, NRF24_Buffer, NRF24_ADDRESS_LEN);
	return NRF24_Buffer[0] == addr_LSB;
}

void ICACHE_FLASH_ATTR NRF24_Powerdown(void)
{
	NRF24_SET_CE_LOW;
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_CONFIG, NRF24_CONFIG); // Power down
}

void ICACHE_FLASH_ATTR NRF24_init(uint8_t channel) // After init transmit must be delayed
{
	NRF24_SET_CSN_HI;
	uint8_t i = 0, c, v;
	do {
		c = pgm_read_byte(&NRF24_INIT_DATA[i++]);
		v = pgm_read_byte(&NRF24_INIT_DATA[i++]);
		NRF24_WriteByte(c, v); 
	} while(i < sizeof(NRF24_INIT_DATA));
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_RF_CH,	channel);
}
