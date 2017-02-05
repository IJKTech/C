#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "smbus.h"
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

int test_sequence(int loops)
{

	int file;	  
	file = open("/dev/i2c-0", O_RDWR);
	if (file < 0)
	{
		exit(1);
	}

 	int addr = 0x60;

 	if(ioctl(file, I2C_SLAVE, addr) < 0)
 	{
    	return 1;
	}

	__u8 reg = 0x00; /* Device register to access */
	__s32 res;
	int count=0;

	while (count < loops)
	{ 
		res = i2c_smbus_write_byte_data(file, 0x05, 0x00);
		usleep(500000);
		res = i2c_smbus_write_byte_data(file, 0x05, 0x55);
		usleep(500000);
		res = i2c_smbus_write_byte_data(file, 0x05, 0x54);
		usleep(500000);
		res = i2c_smbus_write_byte_data(file, 0x05, 0x50);
		usleep(500000);
		res = i2c_smbus_write_byte_data(file, 0x05, 0x40);
		usleep(500000);
		res = i2c_smbus_write_byte_data(file, 0x05, 0x00);
		usleep(500000);
		res = i2c_smbus_write_byte_data(file, 0x01, 0x20);
		res = i2c_smbus_write_byte_data(file, 0x05, 0xaa);
		usleep(1500000);
		res = i2c_smbus_write_byte_data(file, 0x01, 0x01);
		usleep(1500000);
		res = i2c_smbus_write_byte_data(file, 0x05, 0x55);
		count++;
	}

	close(file);
	return 0;

}

int knightrider(int loops)
{

	int file;	  
	file = open("/dev/i2c-0", O_RDWR);
	if (file < 0)
	{
		exit(1);
	}

	int addr = 0x60;

	if(ioctl(file, I2C_SLAVE, addr) < 0)
	{
		return 1;
	}

	__u8 reg = 0x00; /* Device register to access */
	__s32 res;
	int count=0;
	int sleeptime;
	int i;
	while (count < loops)
	{ 
		sleeptime = 50000;

		for (i=0; i < 20; i++){
		
			res = i2c_smbus_write_byte_data(file, 0x05, 0x54);
			usleep(sleeptime);
			res = i2c_smbus_write_byte_data(file, 0x05, 0x51);
			usleep(sleeptime);
			res = i2c_smbus_write_byte_data(file, 0x05, 0x45);
			usleep(sleeptime);
			res = i2c_smbus_write_byte_data(file, 0x05, 0x15);
			usleep(sleeptime);

			sleeptime -= 2500;

			res = i2c_smbus_write_byte_data(file, 0x05, 0x15);
			usleep(sleeptime);
			res = i2c_smbus_write_byte_data(file, 0x05, 0x45);
			usleep(sleeptime);
			res = i2c_smbus_write_byte_data(file, 0x05, 0x51);
			usleep(sleeptime);
			res = i2c_smbus_write_byte_data(file, 0x05, 0x54);
			usleep(sleeptime);
		}

		count++;
	}

	close(file);
	return 0;
}


int main()
{

	knightrider(10);
	test_sequence(10);
	return 0;
}
