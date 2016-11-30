/*
 * nRF24L01.c
 *
 * Created: 01.11.2013 15:09:29, updated for esp8266: 04.2016
 * Written by Vadim Kulakov, vad7 @ yahoo.com
 *
 */
#include "sdk/add_func.h"
#include "driver/spi.h"
#include "driver/nrf24l01.h"

uint8_t NRF24_Buffer[NRF24_PAYLOAD_LEN]; // MUST be EQUAL or GREATER than Address field width!!!

uint8_t __attribute__((aligned(2))) NRF24_INIT_DATA[] = {
	NRF24_CMD_W_REGISTER | NRF24_REG_RF_SETUP,	(0<<NRF24_BIT_RF_DR_LOW) | (0<<NRF24_BIT_RF_DR_HIGH) | 0b111, // Data rate: 1Mbps, Max power (0b111)
	NRF24_CMD_W_REGISTER | NRF24_REG_SETUP_AW,	NRF24_ADDRESS_LEN - 2, // address length
	NRF24_CMD_W_REGISTER | NRF24_REG_SETUP_RETR,(0b0111<<NRF24_BIT_ARD) | (0b0011<<NRF24_BIT_ARC), // Auto Retransmit Delay +1*250uS, 3 Re-Transmit on fail (must be > 0 for ACK)
//	NRF24_CMD_W_REGISTER | NRF24_REG_RF_CH,		NRF24_RF_CHANNEL, // RF channel
//	NRF24_CMD_W_REGISTER | NRF24_REG_RX_PW_P0,	NRF24_PAYLOAD_LEN,
	NRF24_CMD_W_REGISTER | NRF24_REG_FEATURE,	(1<<NRF24_BIT_EN_DPL) | (1<<NRF24_BIT_EN_ACK_PAY), // Dynamic Payload Length, Enables Payload with ACK
	NRF24_CMD_W_REGISTER | NRF24_REG_EN_AA,		0b000001, // Enable ‘Auto Acknowledgment’ for pipes 0
	NRF24_CMD_W_REGISTER | NRF24_REG_EN_RXADDR,	0b000001, // Enable data pipes: 0
	NRF24_CMD_W_REGISTER | NRF24_REG_DYNPD,		0b000001  // Dynamic payload
};
uint8_t NRF24_BASE_ADDR[] = { 0xC8, 0xC8 }; // Address MSBs: 2..3

os_timer_t 	NRF24_timer DATA_IRAM_ATTR;

#ifdef SPI_BLOCK

#if DEBUGSOO > 4
uint32 system_get_time(void);
#endif

// Send command & receive status
uint8_t ICACHE_FLASH_ATTR NRF24_SendCommand(uint8_t cmd)
{
	uint8 buf[1];
	buf[0] = cmd;
	spi_write_read_block(SPI_SEND | SPI_RECEIVE, 0, buf, 1);
	return buf[0];
}

void ICACHE_FLASH_ATTR NRF24_WriteByte(uint8_t cmd, uint8_t value)
{
	uint8 buf[1];
	buf[0] = value;
	spi_write_read_block(SPI_SEND, cmd, buf, 1);
}

#else

uint8_t ICACHE_FLASH_ATTR NRF24_ReadRegister(uint8_t reg)
{
	NRF24_SET_CSN_LOW;
	spi_write_read_byte(NRF24_CMD_R_REGISTER | (NRF24_CMD_REGISTER_MASK & reg));
	uint8_t result = spi_write_read_byte(NRF24_CMD_NOP);
	NRF24_SET_CSN_HI;
	return result;
}

void ICACHE_FLASH_ATTR NRF24_ReadArray(uint8_t cmd, uint8_t *array, uint8_t len)
{
	NRF24_SET_CSN_LOW;
	spi_write_read_byte(cmd);
	while(len-- > 0) *array++ = spi_write_read_byte(NRF24_CMD_NOP);
	NRF24_SET_CSN_HI;
}

void ICACHE_FLASH_ATTR NRF24_WriteByte(uint8_t cmd, uint8_t value)
{
	NRF24_SET_CSN_LOW;
	spi_write_read_byte(cmd);
	spi_write_read_byte(value);
	NRF24_SET_CSN_HI;
}

void ICACHE_FLASH_ATTR NRF24_WriteArray(int8_t cmd, uint8_t *array, uint8_t len)
{
	NRF24_SET_CSN_LOW;
	spi_write_read_byte(cmd);
	while(len-- > 0) spi_write_read_byte(*array++);
	NRF24_SET_CSN_HI;
}

// Send command & receive status
uint8_t ICACHE_FLASH_ATTR NRF24_SendCommand(uint8_t cmd)
{
	NRF24_SET_CSN_LOW;
	uint8_t result = spi_write_read_byte(cmd);
	NRF24_SET_CSN_HI;
	return result;
}

#endif

// Set mode in CONFIG reg
void ICACHE_FLASH_ATTR NRF24_SetMode(uint8_t mode)
{
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_CONFIG, NRF24_CONFIG | (1<<NRF24_BIT_PWR_UP) | mode);
	if(mode & NRF24_ReceiveMode) { // Receive mode
		//NRF24_SendCommand(NRF24_CMD_FLUSH_RX);
#ifdef NRF24_CE_GPIO
		NRF24_SET_CE_HI; // start receiving
#endif
	}
}

// Receive in payload, return data pipe number + 1 if success
// payload must be 32 bytes size.
uint8_t ICACHE_FLASH_ATTR NRF24_Receive(uint8_t *payload)
{
	uint8_t pipe = NRF24_RX_FIFO_EMPTY, st;
	st = NRF24_SendCommand(NRF24_CMD_NOP);
	if((st & (1<<NRF24_BIT_RX_DR)) || (st & (0b111<<NRF24_BIT_RX_P_NO)) != (0b111<<NRF24_BIT_RX_P_NO)) // flag RX set or RX payload ready
	{
		#if DEBUGSOO > 4
			os_printf("NRF Received ST = %X\n", st);
		#endif
#ifdef NRF24_USE_DYNAMIC_PAYLOAD
		NRF24_ReadArray(NRF24_CMD_R_RX_PL_WID, NRF24_Buffer, 1); // get RX payload len
		if(NRF24_Buffer[0] > 32) return pipe;
		NRF24_ReadArray(NRF24_CMD_R_RX_PAYLOAD, payload, NRF24_Buffer[0]);
#else
		NRF24_ReadArray(NRF24_CMD_R_RX_PAYLOAD, payload, NRF24_PAYLOAD_LEN);
#endif
		pipe = ((st >> NRF24_BIT_RX_P_NO) & 0b111);
		// Clear RX status, TX, MAX_RT status - for ACK payload
		NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_STATUS, (1<<NRF24_BIT_RX_DR));
	} else {
		if(st & ((1<<NRF24_BIT_TX_DS)|(1<<NRF24_BIT_MAX_RT))) NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_STATUS, st & ((1<<NRF24_BIT_TX_DS)|(1<<NRF24_BIT_MAX_RT)));
	}
	return pipe;
}

void NRF24_timer_handler(void)
{
	NRF24_transmit_cnt--;
	uint8 st = NRF24_SendCommand(NRF24_CMD_NOP);
	if(NRF24_transmit_cnt == 0 || (st & ((1<<NRF24_BIT_MAX_RT) | (1<<NRF24_BIT_TX_DS)))) { // stop if sent or max retransmit reached
		if(st & (1<<NRF24_BIT_TX_DS)) { // ok
			NRF24_transmit_status = NRF24_Transmit_Ok;
		} else if(st & (1<<NRF24_BIT_MAX_RT)) {
			NRF24_transmit_status = NRF24_Transmit_Error;
		} else
			NRF24_transmit_status = NRF24_Transmit_Timeout;
		ets_timer_disarm(&NRF24_timer);
		#if DEBUGSOO > 4
			os_printf("TRE:%u,%u\n", NRF24_transmit_cnt, system_get_time());
		#endif
	}
}

// Start transmitting payload, see - NRF24_transmit_status
void ICACHE_FLASH_ATTR NRF24_Transmit(uint8_t *payload)
{
	NRF24_SendCommand(NRF24_CMD_FLUSH_TX);
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_STATUS, (1<<NRF24_BIT_TX_DS) | (1<<NRF24_BIT_MAX_RT)); // Clear TX status
	NRF24_WriteArray(NRF24_CMD_W_TX_PAYLOAD, payload, NRF24_PAYLOAD_LEN);
#ifdef NRF24_CE_GPIO
	NRF24_SET_CE_HI; // Start transmission
#endif
	NRF24_transmit_status = NRF24_Transmitting;
	NRF24_transmit_cnt = 50; // ms
	ets_timer_disarm(&NRF24_timer);
	os_timer_setfn(&NRF24_timer, (os_timer_func_t *)NRF24_timer_handler, NULL);
	ets_timer_arm_new(&NRF24_timer, 1, 1, 1); // 1 ms, repeat
}

// Set TX address: NRF24_BASE_ADDR + addr_LSB, return 1 if success
uint8_t ICACHE_FLASH_ATTR NRF24_SetTXAddress(uint8_t addr_LSB)
{
	NRF24_Buffer[0] = addr_LSB;
	os_memcpy(NRF24_Buffer + 1, NRF24_BASE_ADDR, sizeof(NRF24_BASE_ADDR)/sizeof(NRF24_BASE_ADDR[0]));
	NRF24_WriteArray(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR, NRF24_Buffer, NRF24_ADDRESS_LEN);
	NRF24_ReadArray(NRF24_CMD_R_REGISTER | NRF24_REG_TX_ADDR, NRF24_Buffer, NRF24_ADDRESS_LEN);
#if DEBUGSOO > 4
	os_printf("NRF TX_Addr: ");
	uint8 i;
	for(i = 0; i < NRF24_ADDRESS_LEN; i++) os_printf("%X ", NRF24_Buffer[i]);
	os_printf("\n");
#endif
	return NRF24_Buffer[0] == addr_LSB;
}

// Set RX address to pipe: NRF24_BASE_ADDR + addr_LSB, return 1 if success
uint8_t ICACHE_FLASH_ATTR NRF24_SetRXAddress(uint8_t pipe, uint8_t addr_LSB)
{
	NRF24_Buffer[0] = addr_LSB;
	os_memcpy(NRF24_Buffer + 1, NRF24_BASE_ADDR, sizeof(NRF24_BASE_ADDR)/sizeof(NRF24_BASE_ADDR[0]));
	NRF24_WriteArray(NRF24_CMD_W_REGISTER | (NRF24_REG_RX_ADDR_P0 + pipe), NRF24_Buffer, NRF24_ADDRESS_LEN);
	NRF24_ReadArray(NRF24_CMD_R_REGISTER | (NRF24_REG_RX_ADDR_P0 + pipe), NRF24_Buffer, NRF24_ADDRESS_LEN);
#if DEBUGSOO > 4
	os_printf("NRF RX_Addr: ");
	uint8 i;
	for(i = 0; i < NRF24_ADDRESS_LEN; i++) os_printf("%X ", NRF24_Buffer[i]);
	os_printf("\n");
#endif
	return NRF24_Buffer[0] == addr_LSB;
}

/*
void ICACHE_FLASH_ATTR NRF24_Powerdown(void)
{
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_CONFIG, NRF24_CONFIG); // Power down
	NRF24_SET_CE_LOW;
}
*/
// After init transmit must be delayed
void ICACHE_FLASH_ATTR NRF24_init(void)
{
	ets_intr_lock();
	spi_init(4000); // SPI clock, kHz
#ifdef NRF24_CE_GPIO
	SET_PIN_FUNC(NRF24_CE_GPIO, (MUX_FUN_IO_PORT(NRF24_CE_GPIO) )); // установить функцию GPIOx в режим порта i/o
	SET_PIN_PULLUP_DIS(NRF24_CE_GPIO);
	GPIO_ENABLE_W1TS = (1<<NRF24_CE_GPIO); // Configure GPIO port to output
	NRF24_SET_CE_HI;
#endif
#ifdef NRF24_CSN_GPIO
	SET_PIN_FUNC(NRF24_CSN_GPIO, (MUX_FUN_IO_PORT(NRF24_CSN_GPIO) )); // установить функцию GPIOx в режим порта i/o
	SET_PIN_PULLUP_DIS(NRF24_CSN_GPIO);
	GPIO_ENABLE_W1TS = (1<<NRF24_CSN_GPIO); // Configure GPIO port to output
	NRF24_SET_CSN_HI;
#endif
	ets_intr_unlock();
	#if DEBUGSOO > 4
		uint32 r = READ_PERI_REG(SPI_CLOCK(0));
		os_printf("SPI0 Clk: %uMHz(%X=%u,%u,%u,%u)\n", ets_get_cpu_frequency()/(((r>>SPI_CLKDIV_PRE_S)&SPI_CLKDIV_PRE)+1)/(((r>>SPI_CLKCNT_N_S)&SPI_CLKCNT_N)+1),
				r, (r>>SPI_CLKDIV_PRE_S)&SPI_CLKDIV_PRE, (r>>SPI_CLKCNT_N_S)&SPI_CLKCNT_N, (r>>SPI_CLKCNT_H_S)&SPI_CLKCNT_H, (r>>SPI_CLKCNT_L_S)&SPI_CLKCNT_L);
	#endif
	uint16_t i;
	for(i = 0; i < sizeof(NRF24_INIT_DATA) / 2; i++) {
		uint16_t d = ((uint16 *)NRF24_INIT_DATA)[i]; // array must be: aligned(2)
		NRF24_WriteByte(d & 0xFF, d / 256);
	}
}
