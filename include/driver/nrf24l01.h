/*
 * nRF24L01.c
 *
 * Created: 01.11.2013 15:09:29, updated for esp8266: 04.2016
 * Written by Vadim Kulakov, vad7 @ yahoo.com
 */ 
#include "hw/esp8266.h"

/* Register map table */
#define NRF24_REG_CONFIG		0x00
#define NRF24_REG_EN_AA			0x01
#define NRF24_REG_EN_RXADDR		0x02
#define NRF24_REG_SETUP_AW		0x03
#define NRF24_REG_SETUP_RETR	0x04
#define NRF24_REG_RF_CH			0x05
#define NRF24_REG_RF_SETUP		0x06
#define NRF24_REG_STATUS		0x07
#define NRF24_REG_OBSERVE_TX	0x08
#define NRF24_REG_RPD			0x09
#define NRF24_REG_RX_ADDR_P0	0x0A
#define NRF24_REG_RX_ADDR_P1	0x0B
#define NRF24_REG_RX_ADDR_P2	0x0C
#define NRF24_REG_RX_ADDR_P3	0x0D
#define NRF24_REG_RX_ADDR_P4	0x0E
#define NRF24_REG_RX_ADDR_P5	0x0F
#define NRF24_REG_TX_ADDR		0x10
#define NRF24_REG_RX_PW_P0		0x11
#define NRF24_REG_RX_PW_P1		0x12
#define NRF24_REG_RX_PW_P2		0x13
#define NRF24_REG_RX_PW_P3		0x14
#define NRF24_REG_RX_PW_P4		0x15
#define NRF24_REG_RX_PW_P5		0x16
#define NRF24_REG_FIFO_STATUS	0x17
#define NRF24_REG_DYNPD			0x1C
#define NRF24_REG_FEATURE		0x1D

/* Bit Mnemonics */
#define NRF24_BIT_MASK_RX_DR	6
#define NRF24_BIT_MASK_TX_DS	5
#define NRF24_BIT_MASK_MAX_RT	4
#define NRF24_BIT_EN_CRC		3
#define NRF24_BIT_CRCO			2
#define NRF24_BIT_PWR_UP		1
#define NRF24_BIT_PRIM_RX		0
#define NRF24_BIT_ARD			4
#define NRF24_BIT_ARC			0
#define NRF24_BIT_CONT_WAVE		7
#define NRF24_BIT_RF_DR_LOW		5
#define NRF24_BIT_PLL_LOCK		4
#define NRF24_BIT_RF_DR_HIGH	3
#define NRF24_BIT_RX_DR			6
#define NRF24_BIT_TX_DS			5
#define NRF24_BIT_MAX_RT		4
#define NRF24_BIT_RX_P_NO		1
#define NRF24_BIT_F_TX_FULL		0
#define NRF24_BIT_PLOS_CNT		4
#define NRF24_BIT_ARC_CNT		0
#define NRF24_BIT_TX_REUSE		6
#define NRF24_BIT_TX_FULL		5
#define NRF24_BIT_TX_EMPTY		4
#define NRF24_BIT_RX_FULL		1
#define NRF24_BIT_RX_EMPTY		0
#define NRF24_BIT_EN_DPL		2
#define NRF24_BIT_EN_ACK_PAY	1
#define NRF24_BIT_EN_DYN_ACK	0

/* SPI commands */
#define NRF24_CMD_R_REGISTER         0x00
#define NRF24_CMD_W_REGISTER         0x20
#define NRF24_CMD_REGISTER_MASK      0x1F
#define NRF24_CMD_R_RX_PAYLOAD       0x61
#define NRF24_CMD_W_TX_PAYLOAD       0xA0
#define NRF24_CMD_FLUSH_TX           0xE1
#define NRF24_CMD_FLUSH_RX           0xE2
#define NRF24_CMD_REUSE_TX_PL        0xE3
#define NRF24_CMD_R_RX_PL_WID        0x60
#define NRF24_CMD_W_ACK_PAYLOAD      0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_CMD_NOP                0xFF

#define NRF24_ReceiveMode			(1<<NRF24_BIT_PRIM_RX)
#define NRF24_TransmitMode			0

typedef enum
{
	NRF24_Transmitting = 0,
	NRF24_Transmit_Ok,
	NRF24_Transmit_Error,
	NRF24_Transmit_Timeout,
} NRF25_TRANSMIT_STATUS;

uint16_t		NRF24_transmit_cnt;
volatile uint8_t NRF24_transmit_status; // 1 - ok, 2 - max retransmit count reached, 3 - module is not responses.

#ifdef SPI_BLOCK

uint8_t NRF24_SendCommand(uint8_t cmd) ICACHE_FLASH_ATTR; // Send command & receive status
void NRF24_WriteByte(uint8_t cmd, uint8_t value) ICACHE_FLASH_ATTR;
#define NRF24_ReadArray(cmd, array, len) spi_write_read_block(SPI_RECEIVE, cmd, array, len)
#define NRF24_WriteArray(cmd, array, len) spi_write_read_block(SPI_SEND, cmd, array, len)


#else

uint8_t NRF24_ReadRegister(uint8_t reg) ICACHE_FLASH_ATTR;
void NRF24_ReadArray(uint8_t cmd, uint8_t *array, uint8_t len) ICACHE_FLASH_ATTR;
void NRF24_WriteByte(uint8_t cmd, uint8_t value) ICACHE_FLASH_ATTR;
void NRF24_WriteArray(int8_t cmd, uint8_t *array, uint8_t len) ICACHE_FLASH_ATTR;
uint8_t NRF24_SendCommand(uint8_t cmd) ICACHE_FLASH_ATTR; // Send command & receive status

#endif

void NRF24_SetMode(uint8_t mode) ICACHE_FLASH_ATTR; // Set mode in CONFIG reg
uint8_t NRF24_Receive(uint8_t *payload) ICACHE_FLASH_ATTR; // Receive in payload, return data pipe number + 1 if success
void NRF24_Transmit(uint8_t *payload) ICACHE_FLASH_ATTR; // Transmit payload, return 0 if success, 1 - max retransmit count reached, 2 - module not response.
uint8_t NRF24_SetAddresses(uint8_t addr_LSB) ICACHE_FLASH_ATTR; // Set addresses: NRF24_BASE_ADDR + addr_LSB, return 1 if success
void NRF24_Powerdown(void) ICACHE_FLASH_ATTR;
void NRF24_init(void) ICACHE_FLASH_ATTR; // After init transmit must be delayed
