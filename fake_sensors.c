#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define SENSOR_VALUES 100	
int a = 0;
int b = 0;
int c = 0;
uint16_t fake_accel_x[SENSOR_VALUES];
uint16_t fake_accel_y[SENSOR_VALUES];
uint16_t fake_accel_z[SENSOR_VALUES];
uint16_t fake_joystick_x[SENSOR_VALUES];
uint16_t fake_joystick_y[SENSOR_VALUES];
float fake_microphone[SENSOR_VALUES];




void initialize_fake_sensors()
{
	int i;
	uint16_t fake_rand = 0;
	uint16_t fake_rand_1 = 1;
	uint16_t fake_rand_2 = 1;
	for (i = 0; i < SENSOR_VALUES; i++)
	{
		fake_rand = fake_rand_1 + fake_rand_2;
		fake_accel_x[i] = fake_rand%50;
		fake_rand_2 = fake_rand_1;		
		fake_rand_1 = fake_rand;
		fake_rand = fake_rand_1 + fake_rand_2;
		fake_accel_y[i] = fake_rand%120;
		fake_rand_2 = fake_rand_1;		
		fake_rand_1 = fake_rand;
		fake_rand = fake_rand_1 + fake_rand_2;
		fake_accel_z[i] = fake_rand%80;
		fake_rand_2 = fake_rand_1;		
		fake_rand_1 = fake_rand;
		fake_rand = fake_rand_1 + fake_rand_2;
		fake_joystick_x[i] = (fake_rand+5)%50;
		fake_rand_2 = fake_rand_1;		
		fake_rand_1 = fake_rand;
		fake_rand = fake_rand_1 + fake_rand_2;
		fake_joystick_y[i] = (fake_rand+7)%50;
		fake_rand_2 = fake_rand_1;		
		fake_rand_1 = fake_rand;
		fake_rand = fake_rand_1 + fake_rand_2;
		fake_microphone[i] = fake_rand%1000/5.5;
		fake_rand_2 = fake_rand_1;		
		fake_rand_1 = fake_rand;
		fake_rand = fake_rand_1 + fake_rand_2;
	}
} 

void read_accel(uint16_t * sensor)
{
	sensor[0] = fake_accel_x[a];
	sensor[1] = fake_accel_y[a];
	sensor[2] = fake_accel_z[a];
	a = (a +1) % SENSOR_VALUES;
}

void read_joystick(uint16_t * sensor)
{
	sensor[0] = fake_joystick_x[b];
	sensor[1] = fake_joystick_y[b];
	b = (b+1)%SENSOR_VALUES;
}

void read_microphone(float * sensor)
{
	sensor[0] = fake_microphone[c];
	c=(c+1)%SENSOR_VALUES;
}

// Test main function - commented out since main() is in main.c
/*
void main()
{
	uint16_t acc[3];
	uint16_t joy [2];
	float mic [1];
	int i;
	initialize_fake_sensors();
	
	read_accel(acc);
	read_joystick(joy);
	read_microphone(mic);
	
}
*/
