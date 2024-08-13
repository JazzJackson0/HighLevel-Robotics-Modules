#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> // Functions for serial interface config
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

// i2cget -y [i2cnum, e.g. 3] CHIP-ADDDRESS DATA-ADDRESS
// i2cget -y 3 0x40 05 (0x40 + 5 = 0x45)

//i2cset -y 3 0x40 255 ...

// Linux handles all of the single bit operations for you, Start/Stop, R/W, Ack/Nack

#define PIN_IN 0
#define PIN_OUT 1
#define PIN_LOW 0
#define PIN_HIGH 1
#define GPIO_PATH "/sys/class/gpio"

class Serial {

    private:
        int buffer_size;

        // SPI Settings
        uint8_t mode;
        uint32_t speed;
        uint8_t bits; 

    public:

        /**
         * @brief Construct a new Serial object
         * 
         */
        Serial();

        /**
         * @brief 
         * 
         * @param buff_size 
         */
        void Set_BufferSize(int buff_size);


        /**
         * @brief Initialize the given pin and set its direction.
         * 
         * @param gpioNum 
         * @param pinDirection Direction of Pin (IN or OUT)
         * @return ** int8_t - Returns 1 is successful, -1 if error occured.
         */
        int8_t PinInit(uint8_t gpioNum, int pinDirection);


        /**
         * @brief Initialize serial
         * 
         * @param uartNum
         * @return int8_t 
         */
        int8_t UARTInit(uint8_t uartNum);

        /**
         * @brief 
         * 
         * @param uartNum 
         */
        void UARTDeInit(uint8_t uartNum);


        /**
         * @brief Initialize the I2C Bus
         * 
         * @param i2cNum 
         * @param slaveAddress 
         * @return int8_t 
         */
        int8_t I2CInit(uint8_t i2cNum, uint8_t slaveAddress);


        /**
         * @brief 
         * 
         * @param i2cNum 
         */
        void I2CDeInit(uint8_t i2cNum);

        /**
         * @brief Initialize the SPI Bus
         * 
         * @param spiNum 0: /dev/spidev0.0, 1: /dev/spidev0.1
         * @param spi_mode 
         * @param speed_hz Clock Speed
         * @param spi_bits Num of Bits to read/write on every SPI access
         * @return int8_t 
         */
        int8_t SPIInit(uint8_t spiNum, uint8_t spi_mode, uint32_t speed_hz, uint8_t spi_bits);


        /**
         * @brief 
         * 
         * @param spiNum 
         */
        void SPIDeInit(uint8_t spiNum);


        /**
         * @brief Set the state of a given pin.
         * 
         * @param gpioNum 
         * @param pinState State of Pin (HIGH or LOW)
         * @return ** int8_t - Returns 1 is successful, -1 if error occured.
         */
        int8_t PinWrite(uint8_t gpioNum, uint8_t pinState);

        /**
         * @brief Get the state of a given pin.
         * 
         * @param gpioNum 
         * @return ** int8_t - Returns State of Pin, or -1 if error occured.
         */
        int8_t PinRead(uint8_t gpioNum);


        /**
         * @brief 
         * 
         * @param uart 
         * @param data 
         * @param datalen 
         * @return int8_t 
         */
        int8_t UARTWrite(int uart, char* data, int datalen);


        /**
         * @brief 
         * 
         * @param uart 
         * @param data 
         * @param datalen 
         * @return int8_t 
         */
        int8_t UARTRead(int uart, char* data, int datalen);


        /**
         * @brief Write data to I2C bus
         * 
         * @param i2c_bus I2C Bus device file
         * @param dataBytes Data to write
         * @param byteNum Number of bytes to write
         * @return int 
         */
        int8_t I2CWrite(int8_t &i2c_bus, int8_t *dataBytes, int byteNum);


        /**
         * @brief Read data from I2C bus
         * 
         * @param i2c_bus I2C Bus device file
         * @param dataBytes Buffer to write data to
         * @param byteNum Number of bytes to read
         * @return int 
         */
        int8_t I2CRead(int8_t &i2c_bus, int8_t *dataBytes, int byteNum);


        /**
         * @brief Makes SPI transfer
         * 
         * @param spi_bus SPI Bus device file descriptor
         * @param dataBytes Buffer to write data to. When data is Received, this buffer will be overwritten.
         * @param len Length of data buffer
         * 
         * @return int
         */
        int SPITransfer(int8_t spi_bus, int8_t *dataBytes, int len);

};