/*
 * Copyright (c) 2023 ��Ϊ���ܿƼ�(����)
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#include "sw_i2c.h"

#define ACK 0x0 // acknowledge (SDA LOW)
#define NACK 0x1 // not acknowledge (SDA HIGH)

#define LOW 0x0
#define HIGH 0x1

#define I2C_READ 0x1
#define I2C_WRITE 0x0

static void sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface);
static void sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface);

static void sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit);
static uint8_t sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface);

static uint8_t sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte);
static uint8_t sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack);


int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* ��ʼλ */
	sw_i2c_hal_start(i2c_interface);

	/* ��ַ+��дλ */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_READ);
	if (ack_bit) {
		/* ���豸û�лظ�ACK,ֱ���˳� */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* ������ȡN-1������ ��ACK */
	for (i = 0; i < data_length - 1; ++i) {
		data[i] = sw_i2c_hal_read_byte(i2c_interface, ACK);
	}

	/* ���һ�����ݸ� NACK */
	data[i] = sw_i2c_hal_read_byte(i2c_interface, NACK);

	/* ֹͣλ */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}


int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* ��ʼλ */
	sw_i2c_hal_start(i2c_interface);

	/* ��ַ+��дλ */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		/* ���豸û�лظ�ACK,ֱ���˳� */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* ����д��N������, ÿ�ζ�ȡ1 bit�� ACK */
	for (i = 0; i < data_length; ++i) {
		 ack_bit = sw_i2c_hal_write_byte(i2c_interface, data[i]);
	}

	/* ֹͣλ */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}


int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data)
{
	return sw_i2c_read(i2c_interface, dev_addr, data, 1);
}


int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t data)
{
	return sw_i2c_write(i2c_interface, dev_addr, &data, 1);
}


int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface,
					   uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length)
{
	uint8_t ack_bit;
	sw_i2c_hal_start(i2c_interface);
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, mem_addr);

	return sw_i2c_read(i2c_interface, dev_addr, data, data_length);
}


int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data,
                        uint8_t data_length)
{
	uint8_t ack_bit;
	sw_i2c_hal_start(i2c_interface);
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, mem_addr);

//	return sw_i2c_write(i2c_interface, dev_addr, data, data_length);

	/**/
	for(int i = 0; i < data_length; i++){
		ack_bit = sw_i2c_hal_write_byte(i2c_interface, *(data + i));
	}
	return ack_bit;    
	/**/
}


/***************************
 * ���A���������
 **************************/

/**
 * @brief send start bit by driving sda and scl LOW
 * @param i2c_interface
 */
static void sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface)
{
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->sda_out(LOW, i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief send stop bit
 * @param i2c_interface
 */
static void sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface)
{
	i2c_interface->sda_out(LOW, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
}

/**
 * @brief ��� sda ��ƽ,Ȼ�� scl ���һ��ʱ��
 * @param i2c_interface
 * @param bit bit level to send, 0:LOW, 1:HIGH
 */
static void sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit)
{
	i2c_interface->sda_out(bit, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief �� sda ��ƽֵ,Ȼ�� scl ���һ��ʱ��
 * @param i2c_interface
 * @return ���� SDA ��ƽֵ, 0:LOW, 1:HIGH
 */
static uint8_t sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface)
{
	uint8_t bit;
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	bit = i2c_interface->sda_in(i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
	return bit;
}

/**
 * @brief ��IIC���һ���ֽ�
 * @param i2c_interface
 * @param byte
 * @return ���豸������ ACK ��ƽֵ
 */
static uint8_t sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte)
{
	uint8_t i;
	uint8_t ack;

	for (i = 0; i < 8; ++i) {
		sw_i2c_hal_write_bit(i2c_interface, byte & (0x80 >> i));
	}

	ack = sw_i2c_hal_read_bit(i2c_interface);
	return ack;
}

/**
 * @brief ��IIC�����϶�ȡһ���ֽ�
 * @param i2c_interface
 * @param ack ����豸���� ACK ���� NACK
 * @return ��ȡ�����ֽ�
 */
static uint8_t sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack)
{
	uint8_t byte = 0;
	uint8_t i;

	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	for (i = 0; i < 8; ++i) {
		i2c_interface->scl_out(HIGH, i2c_interface->user_data);
		byte <<= 1;
		byte |= i2c_interface->sda_in(i2c_interface->user_data);
		i2c_interface->scl_out(LOW, i2c_interface->user_data);
	}

	sw_i2c_hal_write_bit(i2c_interface, ack);
	return byte;
}
