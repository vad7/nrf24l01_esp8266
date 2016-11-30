# Esp8266 HSPI driver for nRF24L01
---

Used two mode: SPI and SPI OVERLAP.

Примеры: 
[example, HSPI]([http://vad-7.blogspot.ru/2016/04/esp8266.html])
[example, SPI OVERLAP]([http://vad-7.blogspot.ru/2016/11/aeropac-sn-co2-az-7798-uart.html])

Schematic HSPI: 
![SCH](https://github.com/vad7/nrf24l01_esp8266/blob/master/esp8266-nrf24l01.jpg)

Schematic SPI OVERLAP: 
![SCH](https://github.com/vad7/nrf24l01_esp8266/blob/master/nRF24L01_SPI_OVERLAP.jpg)


![alt tag](https://github.com/vad7/nrf24l01_esp8266/blob/master/nRF24L01.jpg)

[nRF24L01 doc](https://github.com/vad7/nrf24l01_esp8266/blob/master/nRF24L01P_Product_Specification_1_0.pdf) 

Размер передачи (NRF24_PAYLOAD_LEN) и длина адреса (NRF24_ADDRESS_LEN) устанавливается при инициализации

Используется HSPI (GPIO12..15), драйвер HSPI поддерживает блочное чтение, запись (максимум 64 байт), дуплекс. 
Можно включить режим одновременного приема - передачи по 1 байту (SPI_TINY).


Еще здесь: ([http://vad-7.blogspot.ru](http://vad-7.blogspot.ru))
