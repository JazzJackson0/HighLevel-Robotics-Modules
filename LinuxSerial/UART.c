#include "UART.h"



/**
 * @brief Initialize serial
 * 
 * @param uartNum
 * @return int8_t 
 */
int8_t UARTInit(uint8_t uartNum) {

    char path[BUFFER_SIZE];
    struct termios settings; // Serial settings
    int uart;

    snprintf(path, sizeof(path), "/dev/serial.%d", uartNum);
    
    if(uart = open(path, O_RDWR | O_NDELAY, O_NOCTTY) < 0) {
        perror("Unable to open file. Cannot initialize serial/uart %d", uartNum);
        close(uart);
        return -1;
    }
	
    // Get Serial Port Settings
	// tcgetattr(uart, &settings);

    // Set Serial Port Settings
    //cfsetspeed(&settings, B9600);
    settings.c_cflag = B9600 | CS8 | CLOCAL | CREAD; // Baudrate | Size(Bits) | Ignore Modem Status | Set Receiver
    settings.c_iflag = IGNPAR; // Input Flags -> Ignore Parity Errors
    settings.c_oflag = 0; // Ouptut flags
    settings.c_lflag = 0; // Local flags

    // Apply Serial Port Settings
    tcflush(uart, TCIFLUSH); // TCIFLUSH: Flush the INPUT buffer
    tcsetattr(uart, TCSANOW, &settings); // TCSANOW: Apply settings immediately

    return uart;
}

/**
 * @brief 
 * 
 * @param uart 
 * @param data 
 * @param datalen 
 * @return int8_t 
 */
int8_t UARTWrite(int uart, char* data, int datalen) {

    if(write(uart, data, datalen) != datalen){
        perror("Failed to write\n");
        return 0;
    }
    return 1;
}

/**
 * @brief 
 * 
 * @param uart 
 * @param data 
 * @param datalen 
 * @return int8_t 
 */
int8_t UARTRead(int uart, char* data, int datalen) {

    if(write(uart, data, datalen) != datalen){
        perror("Failed to write\n");
        return 0;
    }
    return 1;
}



// Example Main -------------------------------------------------------------
#define MAX_DATA 255

int main(int argc, char* argv[]) {

    char data_write[MAX_DATA];
    char data_read[MAX_DATA];

    int uart = UARTInit(0);

    // Write
    strcpy(data_write, "This is some data to send\n");
    int len = strlen(data_write);
    UARTWrite(uart, data_write, len);

    sleep(3);

    // Read
    memset(data_read, 0, MAX_DATA);
    UARTRead(uart, data_read, MAX_DATA);
    printf("Received Data %s \n", data_read);

    close(uart);
    return 0;
}




















