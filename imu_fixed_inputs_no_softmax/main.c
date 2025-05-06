/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/**
 * @file        main.c
 * @brief       I2C Loopback Example
 * @details     This example uses the I2C Master to read/write from/to the I2C Slave. For
 *              this example you must connect P0.10 to P0.16 (SCL) and P0.11 to P0.17 (SCL). The Master
 *              will use P0.10 and P0.11. The Slave will use P0.16 and P0.17. You must also
 *              connect the pull-up jumpers (JP21 and JP22) to the proper I/O voltage.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2c.h"
#include "gpio.h"
#include "uart.h"
#include "mxc.h"
#include "cnn.h"
#include "sampledata.h"
#include "sampleoutput.h"

/***** Definitions *****/
#define PMIC_I2C MXC_I2C1

#define I2C_FREQ 115200
#define PMIC_SLAVE_ADDR (0x69)
#define INIT_READ_LEN 1
#define INIT_WRITE_LEN 2
#define READ_LEN 14

#define CLKSEL 1
#define GYRO_RANGE 0
#define ACC_RANGE 0
#define SLEEP 1

#define RL_PORT MXC_GPIO0
#define LL_PORT MXC_GPIO0
#define W_PORT MXC_GPIO0
#define RA_PORT MXC_GPIO0
#define LA_PORT MXC_GPIO0
#define H_PORT MXC_GPIO0
#define RL_PIN MXC_GPIO_PIN_5
#define LL_PIN MXC_GPIO_PIN_6
#define W_PIN MXC_GPIO_PIN_7
#define RA_PIN MXC_GPIO_PIN_8
#define LA_PIN MXC_GPIO_PIN_9
#define H_PIN MXC_GPIO_PIN_11

#define HM20_UART MXC_UART2
#define HM20_BAUDRATE 57600
#define BUFF_SIZE 64

/***** Globals *****/
static uint8_t tx_buf[INIT_WRITE_LEN];
static uint8_t rx_buf[INIT_READ_LEN];
static uint8_t tx_data[BUFF_SIZE];

static uint8_t ack_RL[BUFF_SIZE];
static uint8_t ack_LL[BUFF_SIZE];
static uint8_t ack_W[BUFF_SIZE];
static uint8_t ack_RA[BUFF_SIZE];
static uint8_t ack_LA[BUFF_SIZE];
static uint8_t ack_H[BUFF_SIZE];

static uint32_t cnn_input0[36];
static uint32_t cnn_input4[36];
static uint32_t cnn_input8[36];
static uint32_t cnn_input12[36];
static uint32_t cnn_input16[36];
static uint32_t cnn_input20[36];
static uint32_t cnn_input24[36];
static uint32_t cnn_input28[36];

static uint8_t result0[BUFF_SIZE];
static uint8_t result1[BUFF_SIZE];
static uint8_t result2[BUFF_SIZE];
static uint8_t result3[BUFF_SIZE];
static uint8_t result4[BUFF_SIZE];

static int32_t ml_data[CNN_NUM_OUTPUTS];
static char temp_display[BUFF_SIZE];

volatile uint32_t cnn_time; // Stopwatch

/***** Functions *****/

void fail(void) {
	printf("\n*** FAIL ***\n\n");
	while (1)
		;
}

void GPIO_init(void) {
	mxc_gpio_cfg_t gpio_cfg1;
	gpio_cfg1.port = RL_PORT;
	gpio_cfg1.mask = RL_PIN;
	gpio_cfg1.pad = MXC_GPIO_PAD_NONE;
	gpio_cfg1.func = MXC_GPIO_FUNC_OUT;
	gpio_cfg1.vssel = MXC_GPIO_VSSEL_VDDIOH;
	gpio_cfg1.drvstr = MXC_GPIO_DRVSTR_3;

	MXC_GPIO_Config(&gpio_cfg1);

	mxc_gpio_cfg_t gpio_cfg2;
	gpio_cfg2.port = LL_PORT;
	gpio_cfg2.mask = LL_PIN;
	gpio_cfg2.pad = MXC_GPIO_PAD_NONE;
	gpio_cfg2.func = MXC_GPIO_FUNC_OUT;
	gpio_cfg2.vssel = MXC_GPIO_VSSEL_VDDIOH;
	gpio_cfg2.drvstr = MXC_GPIO_DRVSTR_3;

	MXC_GPIO_Config(&gpio_cfg2);

	mxc_gpio_cfg_t gpio_cfg3;
	gpio_cfg3.port = W_PORT;
	gpio_cfg3.mask = W_PIN;
	gpio_cfg3.pad = MXC_GPIO_PAD_NONE;
	gpio_cfg3.func = MXC_GPIO_FUNC_OUT;
	gpio_cfg3.vssel = MXC_GPIO_VSSEL_VDDIOH;
	gpio_cfg3.drvstr = MXC_GPIO_DRVSTR_3;

	MXC_GPIO_Config(&gpio_cfg3);

	mxc_gpio_cfg_t gpio_cfg4;
	gpio_cfg4.port = RA_PORT;
	gpio_cfg4.mask = RA_PIN;
	gpio_cfg4.pad = MXC_GPIO_PAD_NONE;
	gpio_cfg4.func = MXC_GPIO_FUNC_OUT;
	gpio_cfg4.vssel = MXC_GPIO_VSSEL_VDDIOH;
	gpio_cfg4.drvstr = MXC_GPIO_DRVSTR_3;

	MXC_GPIO_Config(&gpio_cfg4);

	mxc_gpio_cfg_t gpio_cfg5;
	gpio_cfg5.port = LA_PORT;
	gpio_cfg5.mask = LA_PIN;
	gpio_cfg5.pad = MXC_GPIO_PAD_NONE;
	gpio_cfg5.func = MXC_GPIO_FUNC_OUT;
	gpio_cfg5.vssel = MXC_GPIO_VSSEL_VDDIOH;
	gpio_cfg5.drvstr = MXC_GPIO_DRVSTR_3;

	MXC_GPIO_Config(&gpio_cfg5);

	mxc_gpio_cfg_t gpio_cfg6;
	gpio_cfg6.port = H_PORT;
	gpio_cfg6.mask = H_PIN;
	gpio_cfg6.pad = MXC_GPIO_PAD_NONE;
	gpio_cfg6.func = MXC_GPIO_FUNC_OUT;
	gpio_cfg6.vssel = MXC_GPIO_VSSEL_VDDIOH;
	gpio_cfg6.drvstr = MXC_GPIO_DRVSTR_3;

	MXC_GPIO_Config(&gpio_cfg6);
}

void I2C_init(void) {
	int error = 0;

	//Setup the I2CM
	error = MXC_I2C_Init(PMIC_I2C, 1, 0x69);

	if (error != E_NO_ERROR) {
		printf("-->Failed master\n");
		while (1) {
		}
	} else {
		printf("\n-->I2C Initialization Complete");
	}

	MXC_I2C_SetFrequency(PMIC_I2C, I2C_FREQ);

	return;
}

void clearCNNInput(void) {
	memset(cnn_input0, 0, sizeof(cnn_input0));
	memset(cnn_input4, 0, sizeof(cnn_input4));
	memset(cnn_input8, 0, sizeof(cnn_input8));
	memset(cnn_input12, 0, sizeof(cnn_input12));
	memset(cnn_input16, 0, sizeof(cnn_input16));
	memset(cnn_input20, 0, sizeof(cnn_input20));
	memset(cnn_input24, 0, sizeof(cnn_input24));
	memset(cnn_input28, 0, sizeof(cnn_input28));
}

void load_input(void) {
	// This function loads the sample data input -- replace with actual data

	memcpy32((uint32_t*) 0x50400000, cnn_input0, 36);
	memcpy32((uint32_t*) 0x50408000, cnn_input4, 36);
	memcpy32((uint32_t*) 0x50410000, cnn_input8, 36);
	memcpy32((uint32_t*) 0x50418000, cnn_input12, 36);
	memcpy32((uint32_t*) 0x50800000, cnn_input16, 36);
	memcpy32((uint32_t*) 0x50808000, cnn_input20, 36);
	memcpy32((uint32_t*) 0x50810000, cnn_input24, 36);
	memcpy32((uint32_t*) 0x50818000, cnn_input28, 36);
}

bool MPU_init(mxc_i2c_req_t reqMaster) {
	int error;

	MXC_Delay(MXC_DELAY_MSEC(1));

	reqMaster.i2c = PMIC_I2C;
	reqMaster.addr = PMIC_SLAVE_ADDR;
	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_READ_LEN;
	reqMaster.rx_buf = rx_buf;
	reqMaster.rx_len = INIT_READ_LEN;
	reqMaster.restart = 0;

	tx_buf[0] = 0x6B;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		printf("error: %d", error);
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[1] = (rx_buf[0] & 0xF8) | CLKSEL;

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_WRITE_LEN;
	reqMaster.rx_buf = NULL;
	reqMaster.rx_len = 0;

	tx_buf[1] = (rx_buf[0] & 0xF8) | CLKSEL;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[0] = 0x1B;
	rx_buf[0] = 0;

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_READ_LEN;
	reqMaster.rx_buf = rx_buf;
	reqMaster.rx_len = INIT_READ_LEN;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[1] = (rx_buf[0] & 0xE7);

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_WRITE_LEN;
	reqMaster.rx_buf = NULL;
	reqMaster.rx_len = 0;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[0] = 0x1C;
	rx_buf[0] = 0;

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_READ_LEN;
	reqMaster.rx_buf = rx_buf;
	reqMaster.rx_len = INIT_READ_LEN;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[1] = (rx_buf[0] & 0xE7);

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_WRITE_LEN;
	reqMaster.rx_buf = NULL;
	reqMaster.rx_len = 0;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[0] = 0x6B;
	rx_buf[0] = 0;

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_READ_LEN;
	reqMaster.rx_buf = rx_buf;
	reqMaster.rx_len = INIT_READ_LEN;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	tx_buf[1] = (rx_buf[0] & 0xBF);

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_WRITE_LEN;
	reqMaster.rx_buf = NULL;
	reqMaster.rx_len = 0;

	MXC_Delay(MXC_DELAY_MSEC(1));

	if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
		return true;
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	return false;
}

int main(void) {

	clearCNNInput();

	MXC_ICC_Enable(MXC_ICC0); // Enable cache

	// Switch to 100 MHz clock
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
	SystemCoreClockUpdate();

	printf("Waiting...\n");

	// DO NOT DELETE THIS LINE:
	MXC_Delay(SEC(2)); // Let debugger interrupt if needed

	// Enable peripheral, enable CNN interrupt, turn on CNN clock
	// CNN clock: APB (50 MHz) div 1
	cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK,
	MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

	printf("\n*** CNN Inference Test imu_fixed_inputs_no_softmax ***\n");

	cnn_init(); // Bring state machine into consistent state
	cnn_load_weights(); // Load kernels
	cnn_load_bias();
	cnn_configure(); // Configure state machine

	const char *msg = "Hello from MAX78000\r\n";
	const char *done1 = "Completed initializing RL\r\n";
	const char *done2 = "Completed initializing LL\r\n";
	const char *done3 = "Completed initializing W\r\n";
	const char *done4 = "Completed initializing RA\r\n";
	const char *done5 = "Completed initializing LA\r\n";
	const char *done6 = "Completed initializing H\r\n";

	const char *downstairs = "You are probably going downstairs\r\n";
	const char *sitting = "You are probably sitting\r\n";
	const char *standing = "You are probably standing\r\n";
	const char *upstairs = "You are probably going upstairs\r\n";
	const char *walking = "You are probably walking\r\n";

	// Initialize UART1
	int err = MXC_UART_Init(HM20_UART, HM20_BAUDRATE, MXC_UART_APB_CLK);

	if (err != E_NO_ERROR) {
		printf("UART init failed: %d\n", err);
		while (1)
			;
	}

	mxc_uart_req_t write_req;
	write_req.uart = HM20_UART;
	write_req.txData = tx_data;
	write_req.txLen = BUFF_SIZE;
	write_req.rxLen = 0;
	write_req.callback = NULL;

	for (int i = 0; i < strlen(msg); i++) {
		tx_data[i] = msg[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		tx_data[j] = '\0';
	}

	for (int i = 0; i < strlen(downstairs); i++) {
		result0[i] = downstairs[i];
	}
	for (int j = strlen(downstairs); j < BUFF_SIZE; j++) {
		result0[j] = '\0';
	}

	for (int i = 0; i < strlen(sitting); i++) {
		result1[i] = sitting[i];
	}
	for (int j = strlen(sitting); j < BUFF_SIZE; j++) {
		result1[j] = '\0';
	}

	for (int i = 0; i < strlen(standing); i++) {
		result2[i] = standing[i];
	}
	for (int j = strlen(standing); j < BUFF_SIZE; j++) {
		result2[j] = '\0';
	}

	for (int i = 0; i < strlen(upstairs); i++) {
		result3[i] = upstairs[i];
	}
	for (int j = strlen(upstairs); j < BUFF_SIZE; j++) {
		result3[j] = '\0';
	}

	for (int i = 0; i < strlen(walking); i++) {
		result4[i] = walking[i];
	}
	for (int j = strlen(walking); j < BUFF_SIZE; j++) {
		result4[j] = '\0';
	}

	for (int i = 0; i < strlen(done1); i++) {
		ack_RL[i] = done1[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		ack_RL[j] = '\0';
	}

	for (int i = 0; i < strlen(done2); i++) {
		ack_LL[i] = done2[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		ack_LL[j] = '\0';
	}

	for (int i = 0; i < strlen(done3); i++) {
		ack_W[i] = done3[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		ack_W[j] = '\0';
	}

	for (int i = 0; i < strlen(done4); i++) {
		ack_RA[i] = done4[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		ack_RA[j] = '\0';
	}

	for (int i = 0; i < strlen(done5); i++) {
		ack_LA[i] = done5[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		ack_LA[j] = '\0';
	}

	for (int i = 0; i < strlen(done6); i++) {
		ack_H[i] = done6[i];
	}
	for (int j = strlen(msg); j < BUFF_SIZE; j++) {
		ack_H[j] = '\0';
	}

	// Optional: Wait for HM-10 to power up
	MXC_Delay(MXC_DELAY_MSEC(1000));

	int error;

	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}

	MXC_Delay(MXC_DELAY_MSEC(500)); //Wait for PMIC to power-up

	GPIO_init();

	MXC_GPIO_OutClr(RL_PORT, RL_PIN);
	MXC_GPIO_OutClr(LL_PORT, LL_PIN);
	MXC_GPIO_OutClr(W_PORT, W_PIN);
	MXC_GPIO_OutClr(RA_PORT, RA_PIN);
	MXC_GPIO_OutClr(LA_PORT, LA_PIN);
	MXC_GPIO_OutClr(H_PORT, H_PIN);

	MXC_GPIO_OutSet(W_PORT, W_PIN);

	I2C_init();

	MXC_I2C_SetTimeout(PMIC_I2C, 100000);

	mxc_i2c_req_t reqMaster;
	reqMaster.i2c = PMIC_I2C;
	reqMaster.addr = PMIC_SLAVE_ADDR;
	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_READ_LEN;
	reqMaster.rx_buf = rx_buf;
	reqMaster.rx_len = INIT_READ_LEN;
	reqMaster.restart = 1;

	//tx_buf[0] = 0x75;

	//if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
	//    printf("Error writing: %d\n", error);
	//    while (1) {}
	//}

	MXC_Delay(MXC_DELAY_MSEC(1));
	MXC_GPIO_OutSet(RL_PORT, RL_PIN);
	MXC_Delay(MXC_DELAY_MSEC(100));

	bool skip_RL = MPU_init(reqMaster);
	//while (skip_RL) {
	//	skip_RL = MPU_init(reqMaster);
	//}

	MXC_Delay(MXC_DELAY_MSEC(100));
	MXC_GPIO_OutClr(RL_PORT, RL_PIN);

	write_req.txData = ack_RL;
	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}
	MXC_Delay(MXC_DELAY_MSEC(1));

	MXC_GPIO_OutSet(LL_PORT, LL_PIN);
	MXC_Delay(MXC_DELAY_MSEC(100));

	bool skip_LL = MPU_init(reqMaster);
	//while (skip_LL) {
	//	skip_LL = MPU_init(reqMaster);
	//}

	MXC_Delay(MXC_DELAY_MSEC(100));
	MXC_GPIO_OutClr(LL_PORT, LL_PIN);

	write_req.txData = ack_LL;
	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}
	MXC_Delay(MXC_DELAY_MSEC(1));

	MXC_GPIO_OutSet(W_PORT, W_PIN);
	MXC_Delay(MXC_DELAY_MSEC(100));

	bool skip_W = MPU_init(reqMaster);
	//while (skip_W) {
	//	skip_W = MPU_init(reqMaster);
	//}

	MXC_Delay(MXC_DELAY_MSEC(100));
	MXC_GPIO_OutClr(W_PORT, W_PIN);

	write_req.txData = ack_W;
	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}
	MXC_Delay(MXC_DELAY_MSEC(1));

	MXC_GPIO_OutSet(RA_PORT, RA_PIN);
	MXC_Delay(MXC_DELAY_MSEC(100));

	bool skip_RA = MPU_init(reqMaster);
	//while (skip_RA) {
	//	skip_RA = MPU_init(reqMaster);
	//}

	MXC_Delay(MXC_DELAY_MSEC(100));
	MXC_GPIO_OutClr(RA_PORT, RA_PIN);

	write_req.txData = ack_RA;
	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}
	MXC_Delay(MXC_DELAY_MSEC(1));

	MXC_GPIO_OutSet(LA_PORT, LA_PIN);
	MXC_Delay(MXC_DELAY_MSEC(100));

	bool skip_LA = MPU_init(reqMaster);
	//while (skip_LA) {
	//	skip_LA = MPU_init(reqMaster);
	//}

	MXC_Delay(MXC_DELAY_MSEC(100));
	MXC_GPIO_OutClr(LA_PORT, LA_PIN);

	write_req.txData = ack_LA;
	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}
	MXC_Delay(MXC_DELAY_MSEC(1));

	MXC_GPIO_OutSet(H_PORT, H_PIN);
	MXC_Delay(MXC_DELAY_MSEC(100));

	bool skip_H = MPU_init(reqMaster);
	//while (skip_H) {
	//	skip_H = MPU_init(reqMaster);
	//}

	MXC_Delay(MXC_DELAY_MSEC(100));
	MXC_GPIO_OutClr(H_PORT, H_PIN);
	MXC_Delay(MXC_DELAY_MSEC(1));

	write_req.txData = ack_H;
	error = MXC_UART_Transaction(&write_req);

	if (error != E_NO_ERROR) {
		printf("-->Error starting sync write: %d\n", error);
	}

	MXC_Delay(MXC_DELAY_MSEC(1));

	write_req.txData = tx_data;

	reqMaster.tx_buf = tx_buf;
	reqMaster.tx_len = INIT_READ_LEN;
	reqMaster.rx_buf = rx_buf;
	reqMaster.rx_len = INIT_READ_LEN;

	tx_buf[0] = 0x3B;

	int prev[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int frame[6][6] = { { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0,
			0, 0, 0 }, { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0,
			0, 0 } };

	int frame_count = 0;

	while (1) {
		int ax = 0;
		int ay = 0;
		int az = 0;
		int gx = 0;
		int gy = 0;
		int gz = 0;

		rx_buf[0] = 0;
		//if (!skip_RL) {
		MXC_GPIO_OutSet(RL_PORT, RL_PIN);
		tx_buf[0] = 0x3B;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = rx_buf[0];
		tx_buf[0] = 0x3C;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = (ax << 8) | rx_buf[0];
		tx_buf[0] = 0x3D;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = rx_buf[0];
		tx_buf[0] = 0x3E;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = (ay << 8) | rx_buf[0];
		tx_buf[0] = 0x3F;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = rx_buf[0];
		tx_buf[0] = 0x40;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = (az << 8) | rx_buf[0];

		tx_buf[0] = 0x43;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = rx_buf[0];
		tx_buf[0] = 0x44;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = (gx << 8) | rx_buf[0];
		tx_buf[0] = 0x45;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = rx_buf[0];
		tx_buf[0] = 0x46;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = (gy << 8) | rx_buf[0];
		tx_buf[0] = 0x47;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = rx_buf[0];
		tx_buf[0] = 0x48;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = (gz << 8) | rx_buf[0];

		MXC_Delay(MXC_DELAY_MSEC(1));
		MXC_GPIO_OutClr(RL_PORT, RL_PIN);
		MXC_Delay(MXC_DELAY_MSEC(1));
		//} else {
		//	tx_data[36] = 52;
		//}
		//Do the rx_buf parsing here
		frame[0][0] = (abs(prev[0] - ax) / 128) - 128;
		prev[0] = ax;
		frame[0][1] = (abs(prev[1] - ay) / 128) - 128;
		prev[1] = ay;
		frame[0][2] = (abs(prev[2] - az) / 128) - 128;
		prev[2] = az;
		frame[0][3] = (abs(prev[3] - gx) / 128) - 128;
		prev[3] = gx;
		frame[0][4] = (abs(prev[4] - gy) / 128) - 128;
		prev[4] = gy;
		frame[0][5] = (abs(prev[5] - gz) / 128) - 128;
		prev[5] = gz;

		rx_buf[0] = 0;
		//if (!skip_LL) {
		MXC_GPIO_OutSet(LL_PORT, LL_PIN);
		tx_buf[0] = 0x3B;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = rx_buf[0];
		tx_buf[0] = 0x3C;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = (ax << 8) | rx_buf[0];
		tx_buf[0] = 0x3D;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = rx_buf[0];
		tx_buf[0] = 0x3E;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = (ay << 8) | rx_buf[0];
		tx_buf[0] = 0x3F;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = rx_buf[0];
		tx_buf[0] = 0x40;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = (az << 8) | rx_buf[0];

		tx_buf[0] = 0x43;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = rx_buf[0];
		tx_buf[0] = 0x44;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = (gx << 8) | rx_buf[0];
		tx_buf[0] = 0x45;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = rx_buf[0];
		tx_buf[0] = 0x46;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = (gy << 8) | rx_buf[0];
		tx_buf[0] = 0x47;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = rx_buf[0];
		tx_buf[0] = 0x48;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = (gz << 8) | rx_buf[0];

		MXC_Delay(MXC_DELAY_MSEC(1));
		MXC_GPIO_OutClr(LL_PORT, LL_PIN);
		MXC_Delay(MXC_DELAY_MSEC(1));
		//} else {
		//	tx_data[37] = 52;
		//}
		//Do the rx_buf parsing here
		frame[1][0] = (abs(prev[6] - ax) / 128) - 128;
		prev[6] = ax;
		frame[1][1] = (abs(prev[7] - ay) / 128) - 128;
		prev[7] = ay;
		frame[1][2] = (abs(prev[8] - az) / 128) - 128;
		prev[8] = az;
		frame[1][3] = (abs(prev[9] - gx) / 128) - 128;
		prev[9] = gx;
		frame[1][4] = (abs(prev[10] - gy) / 128) - 128;
		prev[10] = gy;
		frame[1][5] = (abs(prev[11] - gz) / 128) - 128;
		prev[11] = gz;

		rx_buf[0] = 0;
		//if (!skip_W) {
		MXC_GPIO_OutSet(W_PORT, W_PIN);
		tx_buf[0] = 0x3B;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = rx_buf[0];
		tx_buf[0] = 0x3C;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = (ax << 8) | rx_buf[0];
		tx_buf[0] = 0x3D;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = rx_buf[0];
		tx_buf[0] = 0x3E;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = (ay << 8) | rx_buf[0];
		tx_buf[0] = 0x3F;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = rx_buf[0];
		tx_buf[0] = 0x40;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = (az << 8) | rx_buf[0];

		tx_buf[0] = 0x43;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = rx_buf[0];
		tx_buf[0] = 0x44;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = (gx << 8) | rx_buf[0];
		tx_buf[0] = 0x45;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = rx_buf[0];
		tx_buf[0] = 0x46;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = (gy << 8) | rx_buf[0];
		tx_buf[0] = 0x47;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = rx_buf[0];
		tx_buf[0] = 0x48;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = (gz << 8) | rx_buf[0];

		MXC_Delay(MXC_DELAY_MSEC(1));
		MXC_GPIO_OutClr(W_PORT, W_PIN);
		MXC_Delay(MXC_DELAY_MSEC(1));
		//} else {
		//	tx_data[38] = 52;
		//}
		//Do the rx_buf parsing here
		frame[2][0] = (abs(prev[12] - ax) / 128) - 128;
		prev[12] = ax;
		frame[2][1] = (abs(prev[13] - ay) / 128) - 128;
		prev[13] = ay;
		frame[2][2] = (abs(prev[14] - az) / 128) - 128;
		prev[14] = az;
		frame[2][3] = (abs(prev[15] - gx) / 128) - 128;
		prev[15] = gx;
		frame[2][4] = (abs(prev[16] - gy) / 128) - 128;
		prev[16] = gy;
		frame[2][5] = (abs(prev[17] - gz) / 128) - 128;
		prev[17] = gz;

		rx_buf[0] = 0;
		//if (!skip_RA) {
		MXC_GPIO_OutSet(RA_PORT, RA_PIN);
		tx_buf[0] = 0x3B;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = rx_buf[0];
		tx_buf[0] = 0x3C;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = (ax << 8) | rx_buf[0];
		tx_buf[0] = 0x3D;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = rx_buf[0];
		tx_buf[0] = 0x3E;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = (ay << 8) | rx_buf[0];
		tx_buf[0] = 0x3F;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = rx_buf[0];
		tx_buf[0] = 0x40;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = (az << 8) | rx_buf[0];

		tx_buf[0] = 0x43;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = rx_buf[0];
		tx_buf[0] = 0x44;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = (gx << 8) | rx_buf[0];
		tx_buf[0] = 0x45;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = rx_buf[0];
		tx_buf[0] = 0x46;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = (gy << 8) | rx_buf[0];
		tx_buf[0] = 0x47;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = rx_buf[0];
		tx_buf[0] = 0x48;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = (gz << 8) | rx_buf[0];

		MXC_Delay(MXC_DELAY_MSEC(1));
		MXC_GPIO_OutClr(RA_PORT, RA_PIN);
		MXC_Delay(MXC_DELAY_MSEC(1));
		//} else {
		//	tx_data[39] = 52;
		//}
		//Do the rx_buf parsing here
		frame[3][0] = (abs(prev[18] - ax) / 128) - 128;
		prev[18] = ax;
		frame[3][1] = (abs(prev[19] - ay) / 128) - 128;
		prev[19] = ay;
		frame[3][2] = (abs(prev[20] - az) / 128) - 128;
		prev[20] = az;
		frame[3][3] = (abs(prev[21] - gx) / 128) - 128;
		prev[21] = gx;
		frame[3][4] = (abs(prev[22] - gy) / 128) - 128;
		prev[22] = gy;
		frame[3][5] = (abs(prev[23] - gz) / 128) - 128;
		prev[23] = gz;

		rx_buf[0] = 0;
		//if (!skip_LA) {
		MXC_GPIO_OutSet(LA_PORT, LA_PIN);
		tx_buf[0] = 0x3B;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = rx_buf[0];
		tx_buf[0] = 0x3C;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = (ax << 8) | rx_buf[0];
		tx_buf[0] = 0x3D;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = rx_buf[0];
		tx_buf[0] = 0x3E;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = (ay << 8) | rx_buf[0];
		tx_buf[0] = 0x3F;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = rx_buf[0];
		tx_buf[0] = 0x40;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = (az << 8) | rx_buf[0];

		tx_buf[0] = 0x43;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = rx_buf[0];
		tx_buf[0] = 0x44;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = (gx << 8) | rx_buf[0];
		tx_buf[0] = 0x45;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = rx_buf[0];
		tx_buf[0] = 0x46;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = (gy << 8) | rx_buf[0];
		tx_buf[0] = 0x47;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = rx_buf[0];
		tx_buf[0] = 0x48;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = (gz << 8) | rx_buf[0];

		MXC_Delay(MXC_DELAY_MSEC(1));
		MXC_GPIO_OutClr(LA_PORT, LA_PIN);
		MXC_Delay(MXC_DELAY_MSEC(1));
		//} else {
		//	tx_data[40] = 52;
		//}
		//Do the rx_buf parsing here
		frame[4][0] = (abs(prev[24] - ax) / 128) - 128;
		prev[24] = ax;
		frame[4][1] = (abs(prev[25] - ay) / 128) - 128;
		prev[25] = ay;
		frame[4][2] = (abs(prev[26] - az) / 128) - 128;
		prev[26] = az;
		frame[4][3] = (abs(prev[27] - gx) / 128) - 128;
		prev[27] = gx;
		frame[4][4] = (abs(prev[28] - gy) / 128) - 128;
		prev[28] = gy;
		frame[4][5] = (abs(prev[29] - gz) / 128) - 128;
		prev[29] = gz;

		rx_buf[0] = 0;
		//if (!skip_H) {
		MXC_GPIO_OutSet(H_PORT, H_PIN);
		tx_buf[0] = 0x3B;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = rx_buf[0];
		tx_buf[0] = 0x3C;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ax = (ax << 8) | rx_buf[0];
		tx_buf[0] = 0x3D;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = rx_buf[0];
		tx_buf[0] = 0x3E;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		ay = (ay << 8) | rx_buf[0];
		tx_buf[0] = 0x3F;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = rx_buf[0];
		tx_buf[0] = 0x40;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		az = (az << 8) | rx_buf[0];

		tx_buf[0] = 0x43;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = rx_buf[0];
		tx_buf[0] = 0x44;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gx = (gx << 8) | rx_buf[0];
		tx_buf[0] = 0x45;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = rx_buf[0];
		tx_buf[0] = 0x46;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gy = (gy << 8) | rx_buf[0];
		tx_buf[0] = 0x47;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = rx_buf[0];
		tx_buf[0] = 0x48;
		MXC_Delay(MXC_DELAY_MSEC(1));
		if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
			error = (error * -1) + 48;
			tx_data[36] = (char) (error);
			rx_buf[0] = 0;
		}
		gz = (gz << 8) | rx_buf[0];

		MXC_Delay(MXC_DELAY_MSEC(1));
		MXC_GPIO_OutClr(H_PORT, H_PIN);
		MXC_Delay(MXC_DELAY_MSEC(1));
		//} else {
		//	tx_data[41] = 52;
		//}
		//Do the rx_buf parsing here
		frame[5][0] = (abs(prev[30] - ax) / 128) - 128;
		prev[30] = ax;
		frame[5][1] = (abs(prev[31] - ay) / 128) - 128;
		prev[31] = ay;
		frame[5][2] = (abs(prev[32] - az) / 128) - 128;
		prev[32] = az;
		frame[5][3] = (abs(prev[33] - gx) / 128) - 128;
		prev[33] = gx;
		frame[5][4] = (abs(prev[34] - gy) / 128) - 128;
		prev[34] = gy;
		frame[5][5] = (abs(prev[35] - gz) / 128) - 128;
		prev[35] = gz;

		for (int i = 0; i < BUFF_SIZE - 3; i++) {
			if (i < 36) {
				if (prev[i] != 0) {
					tx_data[i] = '1';
				} else {
					tx_data[i] = '0';
				}
			} else if (i > 41) {
				tx_data[i] = '2';
			}
		}
		tx_data[BUFF_SIZE - 3] = '\r';
		tx_data[BUFF_SIZE - 2] = '\n';
		tx_data[BUFF_SIZE - 1] = '\0';

		error = MXC_UART_Transaction(&write_req);

		if (error != E_NO_ERROR) {
			printf("-->Error starting sync write: %d\n", error);
		}
		printf("%d", frame[0][0]);

		if (frame_count < 2) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input28[i * 6 + j] = (cnn_input28[i * 6 + j] << 8)
							| value;
				}
			}

		} else if (frame_count < 6) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input24[i * 6 + j] = (cnn_input24[i * 6 + j] << 8)
							| value;
				}
			}

		} else if (frame_count < 10) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input20[i * 6 + j] = (cnn_input20[i * 6 + j] << 8)
							| value;
				}
			}

		} else if (frame_count < 14) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input16[i * 6 + j] = (cnn_input16[i * 6 + j] << 8)
							| value;
				}
			}

		} else if (frame_count < 18) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input12[i * 6 + j] = (cnn_input12[i * 6 + j] << 8)
							| value;
				}
			}

		} else if (frame_count < 22) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input8[i * 6 + j] = (cnn_input8[i * 6 + j] << 8)
							| value;
				}
			}

		} else if (frame_count < 26) {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input4[i * 6 + j] = (cnn_input4[i * 6 + j] << 8)
							| value;
				}
			}

		} else {
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					uint8_t value = (uint8_t) (
							(frame[i][j] < 127) ? frame[i][j] : 127);
					cnn_input0[i * 6 + j] = (cnn_input0[i * 6 + j] << 8)
							| value;
				}
			}
		}

		frame_count = frame_count + 1;
		if (frame_count == 30) {
			//run cnn
			load_input(); // Load data input
			cnn_start(); // Start CNN processing

			while (cnn_time == 0)
				MXC_LP_EnterSleepMode(); // Wait for CNN

			cnn_unload((uint32_t*) ml_data);

			uint32_t value1 = ml_data[0];
			uint32_t value2 = ml_data[1];
			uint32_t value3 = ml_data[2];

			int prob1 = (value1 >> 22) & 0xF;
			uint32_t sign = prob1 & 0x80;
			if (sign != 0) {
				prob1 = prob1 | 0xFFF0;
			}

			int prob2 = (value1 >> 6) & 0xF;
			sign = prob2 & 0x80;
			if (sign != 0) {
				prob2 = prob2 | 0xFFF0;
			}

			int prob3 = (value2 >> 22) & 0xF;
			sign = prob3 & 0x80;
			if (sign != 0) {
				prob3 = prob3 | 0xFFF0;
			}

			int prob4 = (value2 >> 6) & 0xF;
			sign = prob4 & 0x80;
			if (sign != 0) {
				prob4 = prob4 | 0xFFF0;
			}

			int prob5 = (value3 >> 6) & 0xF;
			sign = prob5 & 0x80;
			if (sign != 0) {
				prob5 = prob5 | 0xFFF0;
			}

			sprintf(temp_display, "%d, %d, %d, %d, %d", prob1, prob2, prob3,
					prob4, prob5);

			for (int i = 0; i < BUFF_SIZE; i++) {
				tx_data[i] = temp_display[i];
			}

			error = MXC_UART_Transaction(&write_req);

			if (error != E_NO_ERROR) {
				printf("-->Error starting sync write: %d\n", error);
			}

			if (prob1 > prob2 && prob1 > prob3 && prob1 > prob4
					&& prob1 > prob5) {
				for (int i = 0; i < BUFF_SIZE; i++) {
					tx_data[i] = result0[i];
				}
			}
			if (prob2 > prob1 && prob2 > prob3 && prob2 > prob4
					&& prob2 > prob5) {
				for (int i = 0; i < BUFF_SIZE; i++) {
					tx_data[i] = result1[i];
				}
			}
			if (prob3 > prob2 && prob3 > prob1 && prob3 > prob4
					&& prob3 > prob5) {
				for (int i = 0; i < BUFF_SIZE; i++) {
					tx_data[i] = result2[i];
				}
			}
			if (prob4 > prob2 && prob4 > prob3 && prob4 > prob1
					&& prob4 > prob5) {
				for (int i = 0; i < BUFF_SIZE; i++) {
					tx_data[i] = result3[i];
				}
			}
			if (prob5 > prob2 && prob5 > prob3 && prob5 > prob4
					&& prob5 > prob1) {
				for (int i = 0; i < BUFF_SIZE; i++) {
					tx_data[i] = result4[i];
				}
			}

			error = MXC_UART_Transaction(&write_req);

			if (error != E_NO_ERROR) {
				printf("-->Error starting sync write: %d\n", error);
			}

			frame_count = frame_count - 8;
			memcpy(cnn_input28, cnn_input20, sizeof(cnn_input28));
			memcpy(cnn_input24, cnn_input16, sizeof(cnn_input24));
			memcpy(cnn_input20, cnn_input12, sizeof(cnn_input20));
			memcpy(cnn_input16, cnn_input8, sizeof(cnn_input16));
			memcpy(cnn_input12, cnn_input4, sizeof(cnn_input12));
			memcpy(cnn_input8, cnn_input0, sizeof(cnn_input8));

			for (int i = 0; i < 36; i++) {
				cnn_input28[i] = cnn_input28[i] & 0xFFFF;
			}
		}
	}
}
