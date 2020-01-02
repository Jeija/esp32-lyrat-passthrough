#include <esp_log.h>
#include <driver/i2s.h>
#include <driver/i2c.h>

#include "es8388_registers.h"

/*
 * Basic I2S and I2C Configuration
 */
#define I2S_NUM I2S_NUM_0
#define I2S_READLEN 50 * 4

#define I2C_NUM I2C_NUM_0
#define ES8388_ADDR 0x20

/*
 * ES8388 Configuration Code
 * Configure ES8388 audio codec over I2C for AUX IN input and headphone jack output
 */
static esp_err_t es_write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
	esp_err_t res = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	res |= i2c_master_start(cmd);
	res |= i2c_master_write_byte(cmd, slave_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_stop(cmd);
	res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return res;
}

static esp_err_t es8388_init()
{
	esp_err_t res = ESP_OK;

	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_18,
		.sda_pullup_en = true,
		.scl_io_num = GPIO_NUM_23,
		.scl_pullup_en = true,
		.master.clk_speed = 100000
	};

	res |= i2c_param_config(I2C_NUM, &i2c_config);
	res |= i2c_driver_install(I2C_NUM, i2c_config.mode, 0, 0, 0);

	/* mute DAC during setup, power up all systems, slave mode */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

	/* power up DAC and enable only LOUT1 / ROUT1, ADC sample rate = DAC sample rate */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x30);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

	/* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

	/* DAC to output route mixer configuration */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

	/* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

	/* DAC volume control: 0dB (maximum, unattenuated)  */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

	/* power down ADC while configuring; volume: +9dB for both channels */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x33);

	/* select LINPUT2 / RINPUT2 as ADC input; stereo; 16 bit word length, format right-justified, MCLK / Fs = 256 */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0e);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);

	/* set ADC volume */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x20);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x20);

	/* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

	/* power up and enable DAC; power up ADC (no MIC bias) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);

	return res;
}

/*
 * Digital Filtering Code
 * You can implement your own digital filters (e.g. FIR / IIR / biquad / ...) here.
 * Please mind that you need to use seperate storage for left / right channels for stereo filtering.
 */
static inline int16_t dummyfilter(int16_t x)
{
	return x;
}

/*
 * Main
 */
void app_main(void)
{
	printf("[filter-dsp] Initializing audio codec via I2C...\r\n");

	if (es8388_init() != ESP_OK) {
		printf("[filter-dsp] Audio codec initialization failed!\r\n");
	} else {
		printf("[filter-dsp] Audio codec initialization OK\r\n");
	}

	/*******************/

	printf("[filter-dsp] Initializing input I2S...\r\n");

	i2s_config_t i2s_read_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
		.sample_rate = 44100,
		.bits_per_sample = 16,
		.communication_format = I2S_COMM_FORMAT_I2S,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
		.dma_buf_count = 3,
		.dma_buf_len = I2S_READLEN,
		.use_apll = 1,
		.tx_desc_auto_clear = 1,
		.fixed_mclk = 0
	};

	i2s_pin_config_t i2s_read_pin_config = {
		.bck_io_num = GPIO_NUM_5,
		.ws_io_num = GPIO_NUM_25,
		.data_out_num = GPIO_NUM_26,
		.data_in_num = GPIO_NUM_35
	};

	i2s_driver_install(I2S_NUM, &i2s_read_config, 0, NULL);
	i2s_set_pin(I2S_NUM, &i2s_read_pin_config);

	/*******************/

	printf("[filter-dsp] Initializing MCLK output...\r\n");

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	WRITE_PERI_REG(PIN_CTRL, 0xFFF0);

	/*******************/

	printf("[filter-dsp] Enabling Passthrough mode...\r\n");

	size_t i2s_bytes_read = 0;
	size_t i2s_bytes_written = 0;

	int16_t i2s_buffer_read[I2S_READLEN / sizeof(int16_t)];
	int16_t i2s_buffer_write[I2S_READLEN / sizeof(int16_t)];

	/* continuously read data over I2S, pass it through the filtering function and write it back */
	while (true) {
		i2s_bytes_read = I2S_READLEN;
		i2s_read(I2S_NUM, i2s_buffer_read, I2S_READLEN, &i2s_bytes_read, 100);

		/* left channel filter */
		for (uint32_t i = 0; i < i2s_bytes_read / 2; i += 2)
			i2s_buffer_write[i] = dummyfilter(i2s_buffer_read[i]);

		/* right channel filter */
		for (uint32_t i = 1; i < i2s_bytes_read / 2; i += 2)
			i2s_buffer_write[i] = dummyfilter(i2s_buffer_read[i]);

		i2s_write(I2S_NUM, i2s_buffer_write, i2s_bytes_read, &i2s_bytes_written, 100);
	}
}

