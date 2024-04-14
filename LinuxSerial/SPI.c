/**
 * Most of the logic here is from the 'Johannes 4GNU_Linux' Tutorial
 */
#include "SPI.h"

uint8_t mode = 0;
uint32_t speed = 500000; // 500 KHz
uint8_t bits = 8; // Num of Bits to read/write on every SPI access

/**
 * @brief Initialize the SPI Bus
 * 
 * @param spiNum 0: /dev/spidev0.0, 1: /dev/spidev0.1
 * @return int8_t 
 */
int8_t SPIInit(uint8_t spiNum) {

    char path[BUFFER_SIZE];
    int spi_bus;

    snprintf(path, sizeof(path), "/dev/spidev0.%d", spiNum);
    
    if(spi_bus = open(path, O_RDWR) < 0) {
        perror("Unable to open file. Cannot initialize spi %d", spiNum);
        close(spi_bus);
        return -1;
    }
	
    // Setup the SPI Bus----------------------------------------------
	if (ioctl(spi_bus, SPI_IOC_WR_MODE, &mode) < 0) {
		perror("Failed to set SPI Mode\n");
        close(spi_bus);
        return -1;
	}

    if (ioctl(spi_bus, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		perror("Failed to set SPI Mode\n");
        close(spi_bus);
        return -1;
	}

    if (ioctl(spi_bus, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		perror("Failed to set SPI Mode\n");
        close(spi_bus);
        return -1;
	}

    return spi_bus;
}

/**
 * @brief Makes SPI transfer
 * 
 * @param spi_bus SPI Bus device file descriptor
 * @param dataBytes Buffer to write data to. When data is Received, this buffer will be overwritten.
 * @param len Length of data buffer
 */
void spiTransfer(int8_t spi_bus, int8_t *dataBytes, int len) {

    struct spi_ioc_transfer spi_info[len];

    // Set up the transfer struct
    for (int i = 0; i < len; i++) {
        
        // Initialize w/ all 0 values
        memset(&spi_info[i], 0, sizeof(struct spi_ioc_transfer));

        spi_info[i].tx_buf = (unsigned long) (dataBytes + i);
        spi_info[i].rx_buf = (unsigned long) (dataBytes + i);
        spi_info[i].len = 1;
        spi_info[i].speed_hz = speed;
        spi_info[i].bits_per_word = bits;
    }

    // Transfer Data
    if (ioctl(spi_bus, SPI_IOC_MESSAGE(len), spi_info) < 0) {
        perror("Failed transfer data over SPI\n");
        close(spi_bus);
        return -1;
    }
}


// Example Main -------------------------------------------------------------
#define DATA_BUFF_SIZE 5
int main(int argc, char* argv[]) {

    uint8_t data[DATA_BUFF_SIZE];
    int spi_bus = SPIInit(0);
    if (spi_bus < 0) { return 1; }


    // Send (Random Dummy) Data-------------------
    for (int i = 0; i < DATA_BUFF_SIZE; i++) {
        data[i] = 0x49 + i;
    }
    spiTransfer(spi_bus, data, DATA_BUFF_SIZE);


    // Read Data----------------------------------
    spiTransfer(spi_bus, data, DATA_BUFF_SIZE);
    for (int i = 0; i < DATA_BUFF_SIZE; i++) {
        printf("%d \n", data[i]);
    }
    
    close(spi_bus);
}