#include <asf.h>
#include "Drivers/uart.h"
#include "Drivers/adc.h"
#include "Drivers/imu.h"
#include "Drivers/mpu9250.h"
int main (void)
{
	// ************** INITS ************** //
	board_init();
	adc_init();
	delay_ms(10);
	sysclk_init();
	uart_sd_init();
	delay_ms(20);
	
	PORTC.DIR = 0b11111111; // Define direction of LED pins
	PORTC.OUT = 0b11111111; // Turn LED Off
	delay_ms(200);
	PORTC.OUT = 0b00000000; // Turn LED On
	delay_ms(200);
	
	
	
	//IMU initialization
	twi_options_t m_options = {
		.speed = 400000,
		.speed_reg = TWI_BAUD(32000000, 400000),
	};
	sysclk_enable_peripheral_clock(&TWIE);
	sysclk_enable_peripheral_clock(&ADCA);
	
	twi_master_options_t opt = {
		.speed = 50000,
		.chip  = 0x50
	};

	twi_master_setup(&TWIE, &opt);
	twi_master_init(&TWIE, &m_options);
	twi_master_enable(&TWIE);
	
	mpu9250_t imu_e =
	{
		.twi = &TWIE,
	};
	
	init_imu(imu_e);
	
	
	// ************** MAIN LOOP ************** //
	imu_data_t imu_data;
	int16_t voltage;
	uint32_t packetNumber = 0;
	
	PORTC.DIR = 0b11111111;

	printf("Packet #, ADC,  Pitch,   Roll,    Yaw, AccX, AccY, AccZ,GyroX,GyroY,GyroZ, MagX, MagY, MagZ, Temp\n");

	while(1){
		
		voltage = adc_read(); // Reading ADC
		imu_data = imu_update(imu_e); // Reading IMU
		PORTC.OUT = 0b00000000; // Turn LED On
		
		printf("%8lu,%4d,%3d.%03d,%3d.%03d,%3d.%03d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\n", 
						packetNumber, voltage, 
						(int16_t)imu_data.pitch, (int16_t)abs(((imu_data.pitch-(int16_t)imu_data.pitch)*1000)), 
						(int16_t)imu_data.roll, (int16_t)abs(((imu_data.roll-(int16_t)imu_data.roll)*1000)), 
						(int16_t)imu_data.yaw, (int16_t)abs(((imu_data.yaw-(int16_t)imu_data.yaw)*1000)),
						imu_data.data.acc_x, imu_data.data.acc_y, imu_data.data.acc_z,
						imu_data.data.gyro_x, imu_data.data.gyro_y, imu_data.data.gyro_z,
						imu_data.data.mag_x, imu_data.data.mag_y, imu_data.data.mag_z,
						imu_data.data.imu_temperature);
						
		PORTC.OUT = 0b11111111; // Turn LED Off
		
		packetNumber = packetNumber + 1;
		delay_ms(10);
	}
}
