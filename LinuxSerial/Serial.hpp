#pragma once
#include <iostream>
#include <unordered_map>
#include <string>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

// Linux handles all of the single bit operations for you, Start/Stop, R/W, Ack/Nack

#define PIN_IN 0
#define PIN_OUT 1
#define PIN_LOW 0
#define PIN_HIGH 1

class Serial {

    private:
        int buffer_size;

        // SPI Settings
        uint8_t mode;
        uint32_t speed;
        uint8_t bits; 

        static std::unordered_map<int, struct termios> default_terminal_settings;

        // File Descriptors
        static std::unordered_map<int, int> uart_buses;
        std::unordered_map<int, int> i2c_buses;
        std::unordered_map<int, int> spi_buses;

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
         * @param pinDirection Direction of Pin (PIN_IN or PIN_OUT)
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
         * @brief Restore terminal settings when the program ends
         * 
         */
        static void RestoreTerminal();

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
         * 
         */
        void SPIDeInit(uint8_t spiNum);


        /**
         * @brief Set the state of a given pin.
         * 
         * @param gpioNum 
         * @param pinState State of Pin (PIN_HIGH or PIN_LOW)
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
         * @param uartNum
         * @param data 
         * @param datalen 
         * @return int8_t 
         */
        int8_t UARTWrite(uint8_t uartNum, char* data, int datalen);


        /**
         * @brief 
         * 
         * @param uartNum
         * @param data 
         * @param datalen 
         * @return int8_t 
         */
        int8_t UARTRead(uint8_t uartNum, char* data, int datalen);


        /**
         * @brief Write data to I2C bus
         * 
         * @param i2cNum 
         * @param dataBytes Data to write
         * @param byteNum Number of bytes to write
         * @return int 
         */
        int8_t I2CWrite(uint8_t i2cNum, uint8_t *dataBytes, int byteNum);


        /**
         * @brief Read data from I2C bus
         * 
         * @param i2cNum 
         * @param dataBytes Buffer to write data to
         * @param byteNum Number of bytes to read
         * @return int 
         */
        int8_t I2CRead(uint8_t i2cNum, uint8_t *dataBytes, int byteNum);


        /**
         * @brief Makes SPI transfer
         * 
         * @param spiNum 
         * @param dataBytes Buffer to write data to. When data is Received, this buffer will be overwritten.
         * @param len Length of data buffer
         * 
         * @return int
         */
        int SPITransfer(uint8_t spiNum, int8_t *dataBytes, int len);

        /**
         * @brief 
         * 
         * @param chipNum 
         * @param pwmNum 
         * @param period 
         * @param duty_cycle 
         * @return int 
         */
        int PWMInit(uint8_t chipNum, uint8_t pwmNum, uint64_t period, uint64_t duty_cycle);

        /**
         * @brief 
         * 
         * @param chipNum 
         * @param pwmNum 
         * @return int 
         */
        int PWMDeInit(uint8_t chipNum, uint8_t pwmNum);

        /**
         * @brief 
         * 
         * @param chipNum 
         * @param pwmNum 
         * @param duty_cycle 
         * @return int 
         */
        int PWMUpdate(uint8_t chipNum, uint8_t pwmNum, uint64_t duty_cycle);

};