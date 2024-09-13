/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <assert.h>
// #include <sys_clock.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif


// Calculate the real values on the Rpi more headroom there.
// #define PEAK_CURRENT_FACTOR 1
// #define VOLTAGE_FACTOR 1
// #define CURRENT_FACTOR 1
// #define POWER_FACTOR 1

#define IRQ 24

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* size of stack area used by each thread */
#define STACKSIZE 4096

/* scheduling priority used by each thread */
#define PRIORITY 7
#define PRIORITY_IRQ 2

/* The devicetree node identifier for the GPIOS. */
#define LED0_NODE DT_ALIAS(led0)

#define SSR1_NODE DT_ALIAS(ssr1)
#define SSR2_NODE DT_ALIAS(ssr2)
#define SSR3_NODE DT_ALIAS(ssr3)
#define SSR4_NODE DT_ALIAS(ssr4)
#define SSR5_NODE DT_ALIAS(ssr5)
#define SSR6_NODE DT_ALIAS(ssr6)
#define SSR7_NODE DT_ALIAS(ssr7)
#define SSR8_NODE DT_ALIAS(ssr8)

#define RPD1_NODE DT_ALIAS(rpd1)
#define RPD2_NODE DT_ALIAS(rpd2)
#define RPD3_NODE DT_ALIAS(rpd3)
#define RPD4_NODE DT_ALIAS(rpd4)
#define RPD5_NODE DT_ALIAS(rpd5)
// #define RPD6_NODE DT_ALIAS(rpd6)
// #define RPD7_NODE DT_ALIAS(rpd7)
#define RPD8_NODE DT_ALIAS(rpd8)

#define DEVON_NODE DT_ALIAS(devon)
#define DEVOFF_NODE DT_ALIAS(devoff)
#define DEVN0_NODE DT_ALIAS(devn0)
#define DEVN1_NODE DT_ALIAS(devn1)
#define DEVN2_NODE DT_ALIAS(devn2)

#define DATA0_NODE DT_ALIAS(data0)
#define DATA1_NODE DT_ALIAS(data1)
#define DATA2_NODE DT_ALIAS(data2)
#define DATA3_NODE DT_ALIAS(data3)
#define DATA4_NODE DT_ALIAS(data4)
#define DATA5_NODE DT_ALIAS(data5)
#define DATA6_NODE DT_ALIAS(data6)
#define DATA7_NODE DT_ALIAS(data7)

#define ACK_MCU_NODE DT_ALIAS(ackmcu)
#define ACK_MPU_NODE DT_ALIAS(ackmpu)


// SPI operation configuration
#define SPI_OP  		SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_TRANSFER_LSB | SPI_FULL_DUPLEX
#define BUFFER_SIZE 	5U


// ADC
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),


// gpios
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct gpio_dt_spec ssr1 = GPIO_DT_SPEC_GET(SSR1_NODE, gpios);
static const struct gpio_dt_spec ssr2 = GPIO_DT_SPEC_GET(SSR2_NODE, gpios);
static const struct gpio_dt_spec ssr3 = GPIO_DT_SPEC_GET(SSR3_NODE, gpios);
static const struct gpio_dt_spec ssr4 = GPIO_DT_SPEC_GET(SSR4_NODE, gpios);
static const struct gpio_dt_spec ssr5 = GPIO_DT_SPEC_GET(SSR5_NODE, gpios);
static const struct gpio_dt_spec ssr6 = GPIO_DT_SPEC_GET(SSR6_NODE, gpios);
static const struct gpio_dt_spec ssr7 = GPIO_DT_SPEC_GET(SSR7_NODE, gpios);
static const struct gpio_dt_spec ssr8 = GPIO_DT_SPEC_GET(SSR8_NODE, gpios);

static const struct gpio_dt_spec rpd1 = GPIO_DT_SPEC_GET(RPD1_NODE, gpios);
static const struct gpio_dt_spec rpd2 = GPIO_DT_SPEC_GET(RPD2_NODE, gpios);
static const struct gpio_dt_spec rpd3 = GPIO_DT_SPEC_GET(RPD3_NODE, gpios);
static const struct gpio_dt_spec rpd4 = GPIO_DT_SPEC_GET(RPD4_NODE, gpios);
static const struct gpio_dt_spec rpd5 = GPIO_DT_SPEC_GET(RPD5_NODE, gpios);
// static const struct gpio_dt_spec rpd6 = GPIO_DT_SPEC_GET(RPD6_NODE, gpios);
// static const struct gpio_dt_spec rpd7 = GPIO_DT_SPEC_GET(RPD7_NODE, gpios);
static const struct gpio_dt_spec rpd8 = GPIO_DT_SPEC_GET(RPD8_NODE, gpios);

static const struct gpio_dt_spec devon = GPIO_DT_SPEC_GET(DEVON_NODE, gpios);
static const struct gpio_dt_spec devoff = GPIO_DT_SPEC_GET(DEVOFF_NODE, gpios);
static const struct gpio_dt_spec devn0 = GPIO_DT_SPEC_GET(DEVN0_NODE, gpios);
static const struct gpio_dt_spec devn1 = GPIO_DT_SPEC_GET(DEVN1_NODE, gpios);
static const struct gpio_dt_spec devn2 = GPIO_DT_SPEC_GET(DEVN2_NODE, gpios);

static const struct gpio_dt_spec data0 = GPIO_DT_SPEC_GET(DATA0_NODE, gpios);
static const struct gpio_dt_spec data1 = GPIO_DT_SPEC_GET(DATA1_NODE, gpios);
static const struct gpio_dt_spec data2 = GPIO_DT_SPEC_GET(DATA2_NODE, gpios);
static const struct gpio_dt_spec data3 = GPIO_DT_SPEC_GET(DATA3_NODE, gpios);
static const struct gpio_dt_spec data4 = GPIO_DT_SPEC_GET(DATA4_NODE, gpios);
static const struct gpio_dt_spec data5 = GPIO_DT_SPEC_GET(DATA5_NODE, gpios);
static const struct gpio_dt_spec data6 = GPIO_DT_SPEC_GET(DATA6_NODE, gpios);
static const struct gpio_dt_spec data7 = GPIO_DT_SPEC_GET(DATA7_NODE, gpios);

static const struct gpio_dt_spec ack_mcu = GPIO_DT_SPEC_GET(ACK_MCU_NODE, gpios);
static const struct gpio_dt_spec ack_mpu = GPIO_DT_SPEC_GET(ACK_MPU_NODE, gpios);

const struct gpio_dt_spec datas[] = {
	data0,
	data1,
	data2,
	data3,
	data4,
	data5,
	data6,
	data7,
};

// SPI
const struct spi_dt_spec acs37800_devices[] = {
	SPI_DT_SPEC_GET(DT_NODELABEL(acs378000), SPI_OP, 0),
	SPI_DT_SPEC_GET(DT_NODELABEL(acs378001), SPI_OP, 0),
	SPI_DT_SPEC_GET(DT_NODELABEL(acs378002), SPI_OP, 0),
	SPI_DT_SPEC_GET(DT_NODELABEL(acs378003), SPI_OP, 0),
	// SPI_DT_SPEC_GET(DT_NODELABEL(acs378004), SPI_OP, 0),
};
// ADC
/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};


// SPI buffers
uint8_t tx_buffer[BUFFER_SIZE] = {0};
struct spi_buf my_spi_tx_buffer[] = {{
	.buf = tx_buffer,
	.len = BUFFER_SIZE,
}};
const struct spi_buf_set tx_buff = { my_spi_tx_buffer, 1 };

uint8_t rx_buffer[BUFFER_SIZE] = {0};
struct spi_buf my_spi_rx_buffer[] = {{
	.buf = rx_buffer,
	.len = BUFFER_SIZE,
}};
const struct spi_buf_set rx_buff = { my_spi_rx_buffer, 1 };


// ADC Buffer
uint16_t buf;
struct adc_sequence sequence = {
	.buffer = &buf,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf),
};

struct spi_data_t{
	void *fifo_reserved;
	uint8_t device;
	uint8_t address;
	uint32_t value;
};

//SPI thread data
struct processed_data_t {
	void *fifo_reserved;
	uint8_t command;
	uint8_t value;
};

K_FIFO_DEFINE(spi_fifo);
K_FIFO_DEFINE(process_data_fifo);

bool data_transfer_ready = true;
bool data_to_process = true;
bool state_data_ready = false;

enum power_monitor_addresses {
	POWER_MONITOR_VOLTAGE_CURRENT_RMS = 0x20,
	POWER_MONITOR_POWER_ACTIVE_IMAGINARY = 0x21,
	POWER_MONITOR_POWER_APPARENT_FACTOR = 0x22,
	POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1SEC = 0x26,
	POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1MIN = 0x27,
	POWER_MONITOR_VOLTAGE_CURRENT = 0x2A,
};

enum data_commands{
	VOLTAGE,
	CURRENT,
	POWER_ACTIVE,
	POWER_IMAGINARY,
	POWER_APPARENT,
	POWER_FACTOR,
	PEAK_CURRENT,
};

// Full enum list
// enum power_monitor_addresses power_monitor_spi_addresses[] = {
// 	POWER_MONITOR_VOLTAGE_CURRENT_RMS,
// 	POWER_MONITOR_POWER_ACTIVE_IMAGINARY,
// 	POWER_MONITOR_POWER_APPARENT_FACTOR,
// 	POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1SEC,
// 	POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1MIN,
// 	POWER_MONITOR_VOLTAGE_CURRENT,
// };

//test enum list rasp can only process 2 values: voltage and current
enum power_monitor_addresses power_monitor_spi_addresses[] = {
	POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1MIN,
	POWER_MONITOR_POWER_ACTIVE_IMAGINARY,
	POWER_MONITOR_POWER_APPARENT_FACTOR,
};

// Callback for the device toggle
static struct gpio_callback devon_cb_data;
static struct gpio_callback devoff_cb_data;

static struct gpio_callback ack_mpu_cb_data;
// Callback for the ack device toggle
void ack_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printf("ack interrupt\n");
	data_transfer_ready = true;
}

// ADC read function returns error code of the adc_read_dt function and the value of the ADC in the value pointer
int8_t read_adc_value(const struct adc_dt_spec *adc_dev, uint16_t *value){
	int8_t ret;

	(void)adc_sequence_init_dt(adc_dev, &sequence);
	ret = adc_read_dt(adc_dev, &sequence);
	if (ret < 0) {
		printf("Could not read (%d)\n", ret);
		return ret;
	}

	*value = buf;

	printf("%"PRId32, *value);
	uint32_t value_mv = (uint32_t)*value;
	ret = adc_raw_to_millivolts_dt(adc_dev,
						&value_mv);
	/* conversion to mV may not be supported, skip if not */
	if (ret < 0) {
		printf(" (value in mV not available)\n");
	} else {
		printf(" = %"PRId32" mV\n", value_mv);
	}

	return 0;
}

// Callback for the device toggle
void device_on(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	uint8_t val = 0;
	val += gpio_pin_get_dt(&devn0);
	val += gpio_pin_get_dt(&devn1) << 1;
	val += gpio_pin_get_dt(&devn2) << 2;

	// printf("devn0: %d, devn1: %d, devn2: %d\n", gpio_pin_get_dt(&devn0), gpio_pin_get_dt(&devn1), gpio_pin_get_dt(&devn2));

	switch (val) {
		case 0:
			gpio_pin_set_dt(&ssr1, 1);
			printf("SSR1 toggled\n");
			break;
		case 1:
			gpio_pin_set_dt(&ssr2, 1);
			printf("SSR2 toggled\n");
			break;
		case 2:
			gpio_pin_set_dt(&ssr3, 1);
			printf("SSR3 toggled\n");
			break;
		case 3:
			gpio_pin_set_dt(&ssr4, 1);
			printf("SSR4 toggled\n");
			break;
		case 4:
			gpio_pin_set_dt(&ssr5, 1);
			printf("SSR5 toggled\n");
			break;
		case 5:
			gpio_pin_set_dt(&ssr6, 1);
			printf("SSR6 toggled\n");
			break;
		case 6:
			gpio_pin_set_dt(&ssr7, 1);
			printf("SSR7 toggled\n");
			break;
		case 7:
			gpio_pin_set_dt(&ssr8, 1);
			printf("SSR8 toggled\n");
			break;
		default:
			printf("Invalid value\n");
			break;
	}

	// k_msleep(500);
	// uint16_t solution = 0;
	// uint8_t ret = 0;
	// ret = read_adc_value(&adc_channels[val], &solution);
	// if (ret) { printf("adc read status: %d", ret); }

	// struct spi_data_t data = {
	// 	.device = val,
	// 	.address = 0xFF,
	// 	.value = (uint32_t) solution
	// };

	// size_t size = sizeof(struct spi_data_t);
	// char *mem_ptr = k_malloc(size);
	// if(mem_ptr){
	// 	memcpy(mem_ptr, &data, size);
	// 	k_fifo_put(&spi_fifo, mem_ptr);
	// }
}

void device_off(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	uint8_t val = 0;
	val += gpio_pin_get_dt(&devn0);
	val += gpio_pin_get_dt(&devn1) << 1;
	val += gpio_pin_get_dt(&devn2) << 2;

	// printf("devn0: %d, devn1: %d, devn2: %d\n", gpio_pin_get_dt(&devn0), gpio_pin_get_dt(&devn1), gpio_pin_get_dt(&devn2));

	switch (val) {
		case 0:
			gpio_pin_set_dt(&ssr1, 0);
			printf("SSR1 toggled\n");
			break;
		case 1:
			gpio_pin_set_dt(&ssr2, 0);
			printf("SSR2 toggled\n");
			break;
		case 2:
			gpio_pin_set_dt(&ssr3, 0);
			printf("SSR3 toggled\n");
			break;
		case 3:
			gpio_pin_set_dt(&ssr4, 0);
			printf("SSR4 toggled\n");
			break;
		case 4:
			gpio_pin_set_dt(&ssr5, 0);
			printf("SSR5 toggled\n");
			break;
		case 5:
			gpio_pin_set_dt(&ssr6, 0);
			printf("SSR6 toggled\n");
			break;
		case 6:
			gpio_pin_set_dt(&ssr7, 0);
			printf("SSR7 toggled\n");
			break;
		case 7:
			gpio_pin_set_dt(&ssr8, 0);
			printf("SSR8 toggled\n");
			break;
		default:
			printf("Invalid value\n");
			break;
	}
}
// GPIO setup check if device tree is ready and configure the pin as output for ...
int8_t setup_gpios()
{
	//led
	if (!gpio_is_ready_dt(&led)) { return -1; }
	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) { return -1; }

	//ssr1
	if (!gpio_is_ready_dt(&ssr1)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr1, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr2
	if (!gpio_is_ready_dt(&ssr2)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr2, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr3
	if (!gpio_is_ready_dt(&ssr3)) { return -1; }
	if (gpio_pin_configure_dt(&ssr3, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr4
	if (!gpio_is_ready_dt(&ssr4)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr4, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr5
	if (!gpio_is_ready_dt(&ssr5)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr5, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr6
	if (!gpio_is_ready_dt(&ssr6)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr6, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr7
	if (!gpio_is_ready_dt(&ssr7)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr7, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//ssr8
	if (!gpio_is_ready_dt(&ssr8)) {	return -1; }
	if (gpio_pin_configure_dt(&ssr8, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//rpd1
	if (!gpio_is_ready_dt(&rpd1)) {	return -1; }
	if (gpio_pin_configure_dt(&rpd1, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//rpd2
	if (!gpio_is_ready_dt(&rpd2)) {	return -1; }
	if (gpio_pin_configure_dt(&rpd2, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//rpd3
	if (!gpio_is_ready_dt(&rpd3)) {	return -1; }
	if (gpio_pin_configure_dt(&rpd3, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//rpd4
	if (!gpio_is_ready_dt(&rpd4)) {	return -1; }
	if (gpio_pin_configure_dt(&rpd4, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//rpd5
	if (!gpio_is_ready_dt(&rpd5)) {	return -1; }
	if (gpio_pin_configure_dt(&rpd5, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	// //rpd6
	// if (!gpio_is_ready_dt(&rpd6)) {	return -1; }
	// if (gpio_pin_configure_dt(&rpd6, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	// //rpd7
	// if (!gpio_is_ready_dt(&rpd7)) {	return -1; }
	// if (gpio_pin_configure_dt(&rpd7, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//rpd8
	if (!gpio_is_ready_dt(&rpd8)) {	return -1; }
	if (gpio_pin_configure_dt(&rpd8, GPIO_OUTPUT_ACTIVE) < 0) {	return -1; }

	//devn0
	if (!gpio_is_ready_dt(&devn0)) { return -1; }
	if (gpio_pin_configure_dt(&devn0, GPIO_INPUT) < 0) { return -1; }

	//devn1
	if (!gpio_is_ready_dt(&devn1)) { return -1; }
	if (gpio_pin_configure_dt(&devn1, GPIO_INPUT) < 0) { return -1; }

	//devn2
	if (!gpio_is_ready_dt(&devn2)) { return -1; }
	if (gpio_pin_configure_dt(&devn2, GPIO_INPUT) < 0) { return -1; }

	//devon
	if (!gpio_is_ready_dt(&devon)) { return -1; }
	if (gpio_pin_configure_dt(&devon, GPIO_INPUT) < 0) { return -1; }

	// Set the interrupt for the device on
	gpio_pin_interrupt_configure_dt(&devon, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&devon_cb_data, device_on, BIT(devon.pin));
	gpio_add_callback(devon.port, &devon_cb_data);

	if (!gpio_is_ready_dt(&devoff)) { return -1; }
	if (gpio_pin_configure_dt(&devoff, GPIO_INPUT) < 0) { return -1; }

	// Set the interrupt for the device on
	gpio_pin_interrupt_configure_dt(&devoff, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&devoff_cb_data, device_off, BIT(devoff.pin));
	gpio_add_callback(devoff.port, &devoff_cb_data);

	//ack_mpu
	if (!gpio_is_ready_dt(&ack_mpu)) { return -1; }
	if (gpio_pin_configure_dt(&ack_mpu, GPIO_INPUT) < 0) { return -1; }

	// gpio_pin_interrupt_configure_dt(&ack, GPIO_INT_EDGE_TO_ACTIVE);
	// gpio_init_callback(&ack_cb_data, ack_interrupt, BIT(ack.pin));
	// gpio_add_callback(ack.port, &ack_cb_data);

	//data
	for (size_t i = 0; i < sizeof(datas) / sizeof(datas[0]); i++)
	{
		if (!gpio_is_ready_dt(&datas[i])) { return -1; }
		if (gpio_pin_configure_dt(&datas[i], GPIO_OUTPUT_ACTIVE) < 0) { return -1; }
	}

	//ack_mcu
	if (!gpio_is_ready_dt(&ack_mcu)) { return -1; }
	if (gpio_pin_configure_dt(&ack_mcu, GPIO_OUTPUT_ACTIVE) < 0) { return -1; }
	gpio_pin_set_dt(&ack_mcu, 0);
	state_data_ready = false;

	return 0;
}

// ADC setup check if device tree is ready and configure the ADC channels
int8_t setup_adc()
{
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printf("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return -1;
		}
		int err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printf("Could not setup channel #%d (%d)\n", i, err);
			return -1;
		}
	}
	return 0;
}

// SPI transfer function returns error code of the spi_transceive_dt function and the value of the register in the solution pointer
uint8_t read_value_power_monitor(const struct spi_dt_spec *spi_dev, uint8_t address, uint32_t *solution)
{
	uint8_t ret = 0;
	// Create the buffer to send the command
	tx_buffer[0] = 0x80 | address;

	// Send the 2 commands to the power monitor to send the desired register and receive the value
	for (size_t i = 0; i < 2; i++)
	{
		ret = spi_transceive_signal(spi_dev->bus,&spi_dev->config, &tx_buff, &rx_buff, NULL);
		if (ret) { printf("spi_transfer status: %d", ret); }
	}
	
	//check if it is a plausible value and try to read it again if it is not for 5 times max
	int i = 0;
	while (rx_buffer[0] != 0x00)
	{
		ret = spi_transceive_signal(spi_dev->bus,&spi_dev->config, &tx_buff, &rx_buff, NULL);
		if (ret) { printf("spi_transfer status: %d", ret); }
		if (i++ >= 1) { break; }
	}
	// return 0xFF if the value is not read
	if (i > 1)
	{
		return 0xFF;
	}
	// Convert the buffer to a uint32_t
	*solution = *((uint32_t *)(rx_buffer + 1));

	// return error code of the spi_transfer_dt function
	return ret;
}

// SPI thread that reads the values from the power monitor and puts them in the fifo
void spi_thread(void){
	uint32_t solution = 0;
	uint8_t ret;

	while (1)
	{
		uint32_t start_time = 0;
		uint8_t address = 0;

		for (size_t i = 0; i < sizeof(power_monitor_spi_addresses); i++)
		{
			address = power_monitor_spi_addresses[i];

			for (size_t j = 0; j < (sizeof(acs37800_devices)/sizeof(acs37800_devices[0])); j++)
			{
				start_time = sys_clock_cycle_get_32();
				ret = read_value_power_monitor(&acs37800_devices[j], address, &solution);
			
				if (ret) { solution = 0xDEADBEEF; }

				struct spi_data_t data = {
					.device = j,
					.address = address,
					.value = solution
				};

				size_t size = sizeof(struct spi_data_t);
				char *mem_ptr = k_malloc(size);
				if(mem_ptr){
					memcpy(mem_ptr, &data, size);
					k_fifo_put(&spi_fifo, mem_ptr);
				}					
				printf("delta time of device %d  : %d\n\r", j, (sys_clock_cycle_get_32() - start_time));
				k_msleep(1000);
			}
			k_msleep(5000);
		}
		k_msleep(10000);
	}
}

// Data processor thread that processes the data from the fifo and puts it in the process_data_fifo
void data_processor_thread(void){
	while (1){
		if(data_to_process){
			struct spi_data_t *data = k_fifo_get(&spi_fifo, K_FOREVER);
			uint8_t address = data->address;
			uint32_t solution = data->value;
			uint8_t device = data->device;
			for (size_t k = 0; k < 4; k++) {
				uint8_t command = 0x00;
				if(address == POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1MIN && k >= 2){
					command = VOLTAGE + (device * (63 / 5));
				}else if(address == POWER_MONITOR_VOLTAGE_CURRENT_RMS_AVG_1MIN && k < 2){
					command = CURRENT + (device * (63 / 5));
				}else if(address == POWER_MONITOR_POWER_ACTIVE_IMAGINARY && k >= 2){
					command = POWER_ACTIVE + (device * (63 / 5));
				}else if(address == POWER_MONITOR_POWER_ACTIVE_IMAGINARY && k < 2){
					command = POWER_IMAGINARY + (device * (63 / 5));
				}else if(address == POWER_MONITOR_POWER_APPARENT_FACTOR && k >= 2){
					command = POWER_APPARENT + (device * (63 / 5));
				}else if(address == POWER_MONITOR_POWER_APPARENT_FACTOR && k < 2){
					command = POWER_FACTOR + (device * (63 / 5));
				}else if(address == 0xFF && k < 2){
					command = PEAK_CURRENT + (device * (63 / 5));
				}else if(address == 0xFF && k >= 2){
					break;
				}

				struct processed_data_t data = {
					.command = command,
					.value = solution >> (k * 8) & 0xFF,
				};

				size_t size = sizeof(struct processed_data_t);
				char *mem_ptr = k_malloc(size);
				if(mem_ptr){
					memcpy(mem_ptr, &data, size);
					k_fifo_put(&process_data_fifo, mem_ptr);
				}

			}
			k_free(data);
			data_to_process = false;
		}
		k_msleep(10);
	}
}

// SPI consumer thread that reads the data from the process_data_fifo and sets the GPIO pins
void spi_consumer_thread(void){
	while (1)
	{
		size_t data_send = 0;
		while (data_send < 4){
			if(gpio_pin_get_dt(&ack_mpu) == state_data_ready){
				if (k_fifo_is_empty(&process_data_fifo)){
					// printf("fifo is empty\n");
					k_msleep(10);
					continue;
				}
				// data_transfer_ready = false;
				struct processed_data_t *data = k_fifo_get(&process_data_fifo, K_FOREVER);
				printf("command = %d: Value = %02X\n\r",data->command, data->value);
				// for (size_t i = 0; i < sizeof(dodn) / sizeof(dodn[0]); i++)
				// {
				// 	gpio_pin_set_dt(&dodn[i], (data->command & (0x01 << i)));
				// }

				for (size_t i = 0; i < sizeof(datas) / sizeof(datas[0]); i++)
				{
					gpio_pin_set_dt(&datas[i], (data->value & (0x01 << i)));
				}

				if(data->command == PEAK_CURRENT){
					data_send += 2;
				}else{
					data_send++;
				}

				k_free(data);

				// k_usleep(1);
				// gpio_pin_set_dt(&data_ready, 0);
				// k_usleep(100);
				// gpio_pin_set_dt(&data_ready, 1);
				if (state_data_ready){
					state_data_ready = false;
					gpio_pin_set_dt(&ack_mcu, 0);
				}else{
					state_data_ready = true;
					gpio_pin_set_dt(&ack_mcu, 1);
				}
			}
			k_msleep(10);
		}
		data_to_process = true;
		k_msleep(10);
	}
}

K_THREAD_DEFINE(spi_thread_id, STACKSIZE, spi_thread, NULL, NULL, NULL, 6, 0, 0);

K_THREAD_DEFINE(spi_consumer_thread_id, STACKSIZE, spi_consumer_thread, NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(data_processor_thread_id, STACKSIZE, data_processor_thread, NULL, NULL, NULL, PRIORITY, 0, 0);

int main(void)
{
	int ret;
	printf("Program is running!\n\r");	

	printf("1 ms to tick: %d\n\r", k_ms_to_ticks_ceil32(1));

	if (setup_gpios() > 0) {
		return 0;
	}

	if (setup_adc() > 0) {
		return 0;
	}

	// uint32_t solution = 0;
	// ret = read_value_power_monitor(&acs378002_dev, 0x1E, &solution);
	// if (ret) { printf("spi_transfer status: %d", ret); }

	// printf("Value = %08X on address %02X\n\r", solution, 0x1E);
	// printf("\r\n");

	// uint16_t value = 0;
	// ret = read_adc_value(&adc_channels[2], &value);
	// if (ret) { printf("adc read status: %d", ret); }
	// printf("the value is: %d\n\r", value);

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		// uint32_t start_time = sys_clock_cycle_get_32();
		k_msleep(SLEEP_TIME_MS);

		// printf("delta time: %d\n\r", (sys_clock_cycle_get_32() - start_time));
	}
	return 0;
}
