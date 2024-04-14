#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <libusb-1.0/libusb.h>


int USBInit(libusb_device_handle *dev) {

	if(libusb_init(NULL) != 0) {
		perror("Error initializing libusb");
		return -1;
	}

    // Args: Context, Vendor ID, Product ID
    dev = libusb_open_device_with_vid_pid(NULL, 0x03eb, 0x0001); 
	if(dev == NULL) {
		printf("Hey! No USB device found!\n");
		libusb_exit(NULL);
		return -1;
	}

    return 1;
}

void USBDeInit(libusb_device_handle *dev) {

	libusb_close(dev);
	libusb_exit(NULL);
}



int USB_BulkIN(libusb_device_handle *dev, char* buffer, int datalen) {
	int bytes_transferred;
    memset(buffer, 0, 8); // Zero out the buffer

    // Receive the data | Args: Device, Endpoint, Data, Data length, bytes transferred, timeout value (ms)
	if (libusb_bulk_transfer(dev, 0x80 | 2, buffer, datalen, &bytes_transferred, 100) != 0) {
        perror("Error receiving data");
        return -1;
    }

    return bytes_transferred;
}



void USB_BulkOUT(libusb_device_handle *dev, char* buffer, int datalen) {
    int bytes_transferred;
    
    // Send the data | Args: Device, Endpoint, Data, Data length, bytes transferred, timeout value (ms)
    if (libusb_bulk_transfer(dev, 1, buffer, 8, &bytes_transferred, 100) != 0) {
        perror("Error sending data");
        return -1;
    }
}


void USB_ControlIN(libusb_device_handle *dev, int* buffer, int datalen) {

    int value = 0;

    // Args: Device, Request Type, Request Number, value, index, datafield, length of data, timeout value (ms)
    if (libusb_control_transfer(dev, 0xC0, 0x2, 0, 0, (unsigned char *) &buffer, datalen, 100) < 0) {
        perror("Error receiving data");
        libusb_close(dev);
		libusb_exit(NULL);
        return -1;
    }

    return 1;
}

void USB_ControlOUT(libusb_device_handle *dev, uint16_t value, int* data, int datalen) {

    // Args: Device, Request Type, Request Number, value, index, datafield, length of data, timeout value (ms)
    if (libusb_control_transfer(dev, 0x40, 0x1, value, 0, data, datalen, 100) < 0) {
        perror("Error sending data");
        libusb_close(dev);
		libusb_exit(NULL);
        return -1;
    }

    return 1;
}


// Example Main -------------------------------------------------------------
#define DATA_LEN 8
#define RANDOM_BYTE 5
libusb_device_handle *dev = NULL;
char buffer[DATA_LEN] = {0xd, 0xd, 0xd, 0xd, 0xd, 0xd, 0xd, 0xd};
char control_buffer = RANDOM_BYTE;
int value = 0;
int main(int argc, char **argv) {

    USBInit(dev);

    // Control Transfer--------------------------------------------
    // Read Control
    USB_ControlIN(dev, control_buffer, DATA_LEN);

    // Write Control
    USB_ControlOUT(dev, value, 0, 0);

    // Bulk Transfer-----------------------------------------------
    // Read Bulk
    USB_BulkIN(dev, buffer, DATA_LEN);

    // Write Bulk
    memset(buffer, 0, 8);
    USB_BulkOUT(dev, buffer, DATA_LEN);
 
	USBDeInit(dev);
	return 0;
}







