# Esp8266 HSPI driver for nRF24L01
---

Schematic: 
![SCH](https://github.com/vad7/nrf24l01_esp8266/blob/master/esp8266-nrf24l01.jpg)

![alt tag](https://github.com/vad7/nrf24l01_esp8266/blob/master/nRF24L01.jpg)

[nRF24L01 doc](https://github.com/vad7/nrf24l01_esp8266.git/blob/master/nRF24L01P_Product_Specification_1_0.pdf) 

Размер передачи (NRF24_PAYLOAD_LEN) и длина адреса (NRF24_ADDRESS_LEN) устанавливается при инициализации

Используется HSPI (GPIO12..15), драйвер HSPI поддерживает блочное чтение, запись (максимум 64 байт), дуплекс. 
Можно включить режим одновременного приема - передачи по 1 байту (SPI_TINY).

Пример: [example](https://github.com/vad7/WirelessCO2_esp8266/blob/master/app/user/wireless_co2.c)
