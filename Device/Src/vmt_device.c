#include "vmt_device.h"
#include "mti_can.h"
#include "mti_radar.h"

#include <math.h>
#include <stdlib.h>
#include "tim.h"
#include "spi.h"

#include "vmt_adc.h"
#include "vmt_uart.h"
#include "vmt_command.h"
#include "vmt_icm20948.h"
#include "mti_imu.h"
#include "mti_water.h"
#include "vmt_flash.h"
#include "vmt_flash_fifo.h"
#include "mti_system.h"
#include "mti_can.h"
#include "mti_radar.h"

flash_values_t flash_value;

typedef struct h_dev_spi_ch_
{
    SPI_HandleTypeDef *p_h_hal;
    GPIO_TypeDef      *p_cs_port;
    uint16_t           cs_pin;
    clk_source_get_t   clk_source_get;
} h_dev_spi_t;

// typedef struct h_imu_ { //dupe
//	uint8_t id;
//	h_icm20948_t *p_h_icm20948;
//	union {
//		int16_t data[ICM20948_DATA_NUM];
//		struct {
//			int16_t accel_x, accel_y, accel_z;
//			int16_t gyro_x, gyro_y, gyro_z;
//			int16_t mag_x, mag_y, mag_z;
//		};
//	};

//	uint8_t step;
//	bool b_g_log_en :1, b_g_log_busy :1, b_update :1, b_overflow :1;
//	h_moving_avg_i16_t *p_h_g_squ_avg[DEV_IMU_VECTOR_AXIS_N];
//	int16_t gravity[DEV_IMU_VECTOR_AXIS_N];
//	double gravity_len;
//	h_vector_t h_accel;
//	h_vector_t h_gyro;
//} h_imu_t;

/* function declaration */
extern void SystemClock_Config(void);

static void dev_icm_pin_set_cb(uint8_t id, icm20948_pin_t pin, bool b_status);
static bool dev_icm_spi_sent_cb(uint8_t id, h_icm20948_spi_t *p_h_spi);
static void dev_icm_spi_finish_cb(h_dev_spi_send_t *p_h_spi);
static void dev_icm_sample_finish_cb(uint8_t id);

static void dev_spi_init(uint8_t channel);

static bool dev_init_process(void);
static bool imu_process(h_imu_t *p_h_imu);
static bool log_save_process(void);
static bool log_load_process(void);
static void dev_debug_process(void);

void moving_avg_i16_update(h_moving_avg_i16_t *p_h_avg, int16_t new_data);
void moving_avg_i16_reset(h_moving_avg_i16_t *p_h_avg);

void moving_avg_f_update(h_moving_avg_f_t *p_h_avg, float new_data);
void moving_avg_f_reset(h_moving_avg_f_t *p_h_avg);

/* parameter */
static bool b_init_finish = false;

static h_sensor_t h_sensor = {
	.h_adc[ADC_SEQ_TEMP] = {
		.thre_h = 6.0,
		.thre_l = 5.8, },
	.h_adc[ADC_SEQ_WATER_BEGIN + 0] = {
		.thre_h = 1.0,
		.thre_l = 0.8,
	},
	.h_adc[ADC_SEQ_WATER_BEGIN + 1] = {
		.thre_h = 1.0,
		.thre_l = 0.8, }, };

static SPI_HandleTypeDef *p_h_spi_hal[DEV_SPI_CH_NUM] = {
    [0] = &hspi2,
    [1] = &hspi1,
};

static h_dev_spi_t h_dev_spi[DEV_SPI_CH_NUM] = {
	[0] = {
		.p_h_hal = &hspi2,
		.p_cs_port = IMU1_nCS_GPIO_Port,
		.cs_pin = IMU1_nCS_Pin,
		.clk_source_get = HAL_RCC_GetPCLK2Freq, },
	[1] = {
		.p_h_hal = &hspi1,
		.p_cs_port = IMU2_nCS_GPIO_Port,
		.cs_pin = IMU2_nCS_Pin,
		.clk_source_get = HAL_RCC_GetPCLK1Freq, }, };

static h_dev_spi_send_t h_spi = { 0x0 };

static h_icm20948_t h_icm20948[DEV_IMU_NUM] = {
	[0] = {
		.id = 0,
		.systick_get_cb = HAL_GetTick,
		.pin_set_cb = dev_icm_pin_set_cb,
		.spi_sent_cb = dev_icm_spi_sent_cb,
		.sample_finish_cb = dev_icm_sample_finish_cb,
		.sent_sel = ICM20948_SENT_SPI,
		.h_sample_p = {
			.timeout = 5, },
		.h_debug = {
			.b_init = true,
//			.b_setting = true,
//			.b_sample = true,
				}, },
	[1] = {
		.id = 1,
		.systick_get_cb = HAL_GetTick,
		.pin_set_cb = dev_icm_pin_set_cb,
		.spi_sent_cb = dev_icm_spi_sent_cb,
		.sample_finish_cb = dev_icm_sample_finish_cb,
		.sent_sel = ICM20948_SENT_SPI,
		.h_sample_p = {
			.timeout = 5, },
		.h_debug = {
			.b_init = true,
//			.b_setting = true,
//			.b_sample = true,
				}, }, };

static int16_t imu_gravity_squ_avg_buff[DEV_IMU_NUM][DEV_IMU_VECTOR_AXIS_N][DEV_IMU_AVG_BUFF_SIZE] = { 0x0 };
static h_moving_avg_i16_t h_imu_gravity_squ_avg[DEV_IMU_NUM][DEV_IMU_VECTOR_AXIS_N] =
		{
			[0][0] = {
				.p_buff = imu_gravity_squ_avg_buff[0][0],
				.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
			[0][1] = {
				.p_buff = imu_gravity_squ_avg_buff[0][1],
				.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
			[0][2] = {
				.p_buff = imu_gravity_squ_avg_buff[0][2],
				.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
			[1][0] = {
				.p_buff = imu_gravity_squ_avg_buff[1][0],
				.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
			[1][1] = {
				.p_buff = imu_gravity_squ_avg_buff[1][1],
				.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
			[1][2] = {
				.p_buff = imu_gravity_squ_avg_buff[1][2],
				.buff_len = DEV_IMU_AVG_BUFF_SIZE, }, };

// static float imu_acc_avg_buff[DEV_IMU_NUM][DEV_IMU_AVG_BUFF_SIZE] = {
//	0x0 };
// static h_moving_avg_f_t h_imu_acc_avg[DEV_IMU_NUM] = {
//	[0] = {
//		.p_buff = imu_acc_avg_buff[0],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
//	[1] = {
//		.p_buff = imu_acc_avg_buff[1],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, }, };
//
// static float imu_gyro_avg_buff[DEV_IMU_NUM][DEV_IMU_AVG_BUFF_SIZE] = {
//	0x0 };
// static h_moving_avg_f_t h_imu_gyro_avg[DEV_IMU_NUM] = {
//	[0] = {
//		.p_buff = imu_gyro_avg_buff[0],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
//	[1] = {
//		.p_buff = imu_gyro_avg_buff[1],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, }, };
//
// static float imu_acc_squ_avg_buff[DEV_IMU_NUM][DEV_IMU_AVG_BUFF_SIZE] = {
//	0x0 };
// static h_moving_avg_f_t h_imu_a_squ_avg[DEV_IMU_NUM] = {
//	[0] = {
//		.p_buff = imu_acc_squ_avg_buff[0],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
//	[1] = {
//		.p_buff = imu_acc_squ_avg_buff[1],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, }, };
//
// static float imu_gyro_squ_avg_buff[DEV_IMU_NUM][DEV_IMU_AVG_BUFF_SIZE] = {
//	0x0 };
// static h_moving_avg_f_t h_imu_gyro_squ_avg[DEV_IMU_NUM] = {
//	[0] = {
//		.p_buff = imu_gyro_squ_avg_buff[0],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, },
//	[1] = {
//		.p_buff = imu_gyro_squ_avg_buff[1],
//		.buff_len = DEV_IMU_AVG_BUFF_SIZE, }, };

static h_imu_t h_imu[DEV_IMU_NUM] = {
	[0] = {
		.id = 0,
		.p_h_icm20948 = &h_icm20948[0],
		.b_g_log_en = true,
		.p_h_g_squ_avg[0] = &h_imu_gravity_squ_avg[0][0],
		.p_h_g_squ_avg[1] = &h_imu_gravity_squ_avg[0][1],
		.p_h_g_squ_avg[2] = &h_imu_gravity_squ_avg[0][2],
//		.h_accel = {
//			.p_h_avg = &h_imu_acc_avg[0],
//			.p_h_squ_avg = &h_imu_a_squ_avg[0], },
//		.h_gyro = {
//			.p_h_avg = &h_imu_gyro_avg[0],
//			.p_h_squ_avg = &h_imu_gyro_squ_avg[0], },
			},
	[1] = {
		.id = 1,
		.p_h_icm20948 = &h_icm20948[1],
		.b_g_log_en = true,
		.p_h_g_squ_avg[0] = &h_imu_gravity_squ_avg[1][0],
		.p_h_g_squ_avg[1] = &h_imu_gravity_squ_avg[1][1],
		.p_h_g_squ_avg[2] = &h_imu_gravity_squ_avg[1][2],
//		.h_accel = {
//			.p_h_avg = &h_imu_acc_avg[0],
//			.p_h_squ_avg = &h_imu_a_squ_avg[0], },
//		.h_gyro = {
//			.p_h_avg = &h_imu_gyro_avg[0],
//			.p_h_squ_avg = &h_imu_gyro_squ_avg[0], },
			}, };

const double math_pi = 3.1415926535897932;

static double dump_angle       = 60.0;
static bool   b_dump           = false;
static bool   b_dump_log       = false;
static bool   b_dump_printf_en = false;

static bool b_water_det     = false;
static bool b_water_det_log = false;
static bool b_water_det_en  = false;

#define DEV_LOG_SAVE_Q_LEN (2)
static h_dev_log_t h_log_save_q[DEV_LOG_SAVE_Q_LEN];
static uint8_t     log_save_q_update = 0;
static uint8_t     log_save_q_new    = 0;

static uint32_t log_save_time_start = 0;

static bool b_log_save_act      = false;
static bool b_log_save_trigger  = false;
static bool b_log_save_overflow = false;
// static bool b_log_next_busy = false;

static bool b_log_erase_all_printf = false;

static h_dev_log_t h_log_load;

static bool          b_log_report_act   = false;
static uart_select_t log_report_uart_ch = UART_ALL;

static h_dev_debug_t h_dev_debug = {
    .b_init = true, .b_spi_init = true, .b_imu_sample_set = true,
    //	.b_adc_sample = true,
    //	.b_imu_sample = true,
};
static uint32_t imu_debug_sample_count = 0;

/* function */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &IMU_SAMPLE_HAL_TIMER)
    {
        h_icm20948[0].b_sample_start = true;
        h_icm20948[1].b_sample_start = true;
        imu_debug_sample_count++;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t channel = h_spi.channel;
    if (channel >= DEV_SPI_CH_NUM)
    {
        return;
    }

    h_dev_spi_t  *p_h_dev_spi_ch = &h_dev_spi[channel];
    GPIO_TypeDef *p_cs_port      = p_h_dev_spi_ch->p_cs_port;
    uint16_t      cs_pin         = p_h_dev_spi_ch->cs_pin;
    HAL_GPIO_WritePin(p_cs_port, cs_pin, GPIO_PIN_SET);

    if (h_spi.finish_cb != NULL)
    {
        h_spi.finish_cb(&h_spi);
    }
}

void adc_sample_finish_cb(adc_seq_t sequence)
{ // POC3 review for removal
    h_sensor.adc_value[sequence] = adc_value_get(sequence);

    h_sensor_adc_t *p_h_adc = &h_sensor.h_adc[sequence];
    double          voltage = adc_volt_get(sequence);
    p_h_adc->volt           = voltage;
    uart_tx_channel_set(UART_DEBUG);
    if (p_h_adc->b_level)
    {
        if (voltage <= p_h_adc->thre_l)
        {
            p_h_adc->b_level = false;
        }
        // p_h_adc->thre_h = voltage*water_threshold_ratio;
        // printf("thre_h = %lf\n",p_h_adc->thre_h);
    }
    else
    {
        if (voltage >= p_h_adc->thre_h)
        {
            p_h_adc->b_level = true;
        }
        // p_h_adc->thre_l = voltage/water_threshold_ratio;
        // printf("thre_l = %lf\n",p_h_adc->thre_l);
    }
    uart_tx_channel_undo();
    if (b_water_det_en)
    {
        if (sequence >= ADC_SEQ_WATER_BEGIN && sequence <= ADC_SEQ_WATER_END)
        {
            h_sensor_adc_t *p_h_adc = &h_sensor.h_adc[ADC_SEQ_WATER_BEGIN];
            b_water_det             = p_h_adc[0].b_level;
            b_water_det |= p_h_adc[1].b_level;

            if (b_water_det_log != b_water_det)
            { // change of state
                b_water_det_log = b_water_det;
            }
        }
    }
}

void flash_fifo_write_finish_cb(h_flash_fifo_t *p_h_flash, flash_fifo_res_t res)
{
    cmd_print_flash_fifo_write_finish(UART_DEBUG);
}

void flash_fifo_delete_finish_cb(h_flash_fifo_t *p_h_flash, size_t seq)
{
    cmd_print_flash_fifo_delete_finish(UART_DEBUG);
}

void flash_fifo_mark_set_finish_cb(h_flash_fifo_t *p_h_flash, h_flash_fifo_mark_t *p_h_mark)
{
    cmd_print_flash_fifo_delete_finish(UART_DEBUG);
}

void flash_fifo_erase_finish_cb(h_flash_fifo_t *p_h_flash)
{
    if (b_log_erase_all_printf)
    {
        b_log_erase_all_printf = false;
        cmd_print_log_erase(UART_ALL);
    }
}

static void dev_icm_pin_set_cb(uint8_t id, icm20948_pin_t pin, bool b_status)
{
    switch (pin)
    {
    case ICM20948_PIN_PWR:
        HAL_GPIO_WritePin(PWR_IMU_GPIO_Port, PWR_IMU_Pin, (GPIO_PinState)b_status);
        uart_tx_channel_set(UART_DEBUG);
        printf("IMU Power (%d): %d\n", id, b_status);
    }
}

static bool dev_icm_spi_sent_cb(uint8_t id, h_icm20948_spi_t *p_h_spi)
{
    if (id >= DEV_IMU_NUM)
    {
        return false;
    }
    h_dev_spi_send_t h_spi_sent = {
        .channel   = id,
        .p_tx_buff = p_h_spi->p_tx_buff,
        .p_rx_buff = p_h_spi->p_rx_buff,
        .data_size = p_h_spi->sent_size,
        .finish_cb = dev_icm_spi_finish_cb,
    };
    return dev_spi_send(&h_spi_sent) == DEV_RES_SUCCESS;
}

static void dev_icm_spi_finish_cb(h_dev_spi_send_t *p_h_spi)
{
    uint8_t id = p_h_spi->channel;
    if (id >= DEV_IMU_NUM)
    {
        return;
    }
    icm20948_sent_finish(&h_icm20948[id], ICM20948_RES_OK);
}

static void dev_icm_sample_finish_cb(uint8_t id)
{
    static bool    imu_status[2];
    static uint8_t retry;

    if (id >= DEV_IMU_NUM)
    {
        return;
    }
    h_imu_t *p_h_imu   = &h_imu[id];
    size_t   data_size = ICM20948_DATA_NUM * sizeof(int16_t);
    memcpy(p_h_imu->data, h_icm20948[id].raw_data, data_size);
    if (p_h_imu->b_update)
    {
        p_h_imu->b_overflow = true;
    }
    else
    {
        data_size      = DEV_IMU_VECTOR_AXIS_N * sizeof(int16_t);
        void *p_source = &p_h_imu->data[ICM20948_DATA_AX];
        memcpy(p_h_imu->h_accel.data_org, p_source, data_size);
        p_source = &p_h_imu->data[ICM20948_DATA_GX];
        memcpy(p_h_imu->h_gyro.data_org, p_source, data_size);
        // if(id == 2)
        if (id == 1)
        { // second IMU read complete
            imu_validate(h_imu);
            if (live_imu)
            {
                imu_compare(&h_imu[0], &h_imu[1]);
            }
            if (state_get() == measure_state || live_imu)
            {
                if (imu_active_get() == 0)
                {
                    bottom_detect(&h_imu[0]);
                }
                else if (imu_active_get() == 1)
                {
                    bottom_detect(&h_imu[1]);
                }
            }
        }
    }
    p_h_imu->b_update = true;
}

//__weak void imu_update_finish_cb(void) {
//}

void imu_update_finish_cb(void)
{
    b_dump = h_imu[0].h_accel.angle >= dump_angle;
    b_dump |= h_imu[1].h_accel.angle >= dump_angle;

    if (b_dump_printf_en)
    {
        if (b_dump_log != b_dump)
        {
            b_dump_log = b_dump;
            // cmd_print_bottom(UART_UPHOLE);
        }
    }

    if (b_log_save_act)
    {
        if (b_log_save_trigger)
        {
            b_log_save_overflow = true;
        }
        else
        {
            h_dev_log_t *p_h_log         = &h_log_save_q[log_save_q_update];
            p_h_log->time                = HAL_GetTick() - log_save_time_start;
            h_dev_log_imu_t *p_h_log_imu = p_h_log->h_imu;
            h_dev_log_imu_t *p_h_log_imu_ch;
            h_imu_t         *p_h_imu;
            for (uint8_t imu_n = 0; imu_n < DEV_IMU_NUM; imu_n++)
            {
                p_h_log_imu_ch    = &p_h_log_imu[imu_n];
                p_h_imu           = &h_imu[imu_n];
                flash_value.acc_x = p_h_imu->accel_x;
                flash_value.acc_y = p_h_imu->accel_y;
                flash_value.acc_z = p_h_imu->accel_z;
                if (imu_n == 0)
                {
                    flash_value.amplitude     = abs(flash_value.acc_x);
                    flash_value.amplitude_max = abs(flash_value.acc_x_max);
                    if (flash_value.amplitude > flash_value.amplitude_max)
                    {
                        flash_value.acc_x_max = flash_value.acc_x;
                    }
                    flash_value.amplitude     = abs(flash_value.acc_y);
                    flash_value.amplitude_max = abs(flash_value.acc_y_max);
                    if (flash_value.amplitude > flash_value.amplitude_max)
                    {
                        flash_value.acc_y_max = flash_value.acc_y;
                    }
                    flash_value.amplitude     = abs(flash_value.acc_z);
                    flash_value.amplitude_max = abs(flash_value.acc_z_max);
                    if (flash_value.amplitude > flash_value.amplitude_max)
                    {
                        flash_value.acc_z_max = flash_value.acc_z;
                    }
                    p_h_log_imu_ch->acc_x = flash_value.acc_x_max;
                    p_h_log_imu_ch->acc_y = flash_value.acc_y_max;
                    p_h_log_imu_ch->acc_z = flash_value.acc_z_max;
                }
                else
                {
                    p_h_log_imu_ch->acc_x = flash_value.acc_x;
                    p_h_log_imu_ch->acc_y = flash_value.acc_y;
                    p_h_log_imu_ch->acc_z = flash_value.acc_z;
                }
                p_h_log_imu_ch->angle_acc  = p_h_imu->h_accel.angle * 100;
                p_h_log_imu_ch->angle_gyro = gyro_angle_get();
                // p_h_imu->h_gyro.angle * 100;
            }
            log_save_q_new = log_save_q_update;
            if (++log_save_q_update >= DEV_LOG_SAVE_Q_LEN)
            {
                log_save_q_update = 0;
            }
            b_log_save_trigger = true;
        }
    }
    // printf("@val,%d:%d:%d||%d,%d,%d\r\n",h_imu[0].gyro_x,h_imu[0].gyro_y,h_imu[0].gyro_z,h_imu[1].gyro_x,h_imu[1].gyro_y,h_imu[1].gyro_z);
    // if(live_imu) imu_compare(&h_imu[0], &h_imu[1]);
}

void flash_data_reset(void)
{
    flash_value.acc_x_max = 0;
    flash_value.acc_y_max = 0;
    flash_value.acc_z_max = 0;
}

void dev_spi_init(uint8_t channel)
{
    if (channel >= DEV_SPI_CH_NUM)
    {
        return;
    }

    h_dev_spi_t *p_h_spi    = &h_dev_spi[channel];
    uint32_t     clk_source = p_h_spi->clk_source_get();
    uint32_t     spi_freq;
    uint8_t      prescaler;
    for (prescaler = 0; prescaler <= 7; prescaler++)
    {
        spi_freq = clk_source >> (prescaler + 1);
        if (spi_freq <= ICM20948_SPI_FREQ_MAX)
        {
            break;
        }
    }
    prescaler &= 0x7;

    SPI_HandleTypeDef *p_h_hal      = p_h_spi->p_h_hal;
    p_h_hal->Init.BaudRatePrescaler = prescaler;
    SPI_TypeDef *p_reg              = p_h_hal->Instance;
    uint32_t     reg                = p_reg->CR1 & ~(SPI_CR1_BR_Msk);
    p_reg->CR1                      = reg | (prescaler << SPI_CR1_BR_Pos);

    if (h_dev_debug.b_spi_init)
    {
        printf("> spi:init");
        printf(",%d", channel);
        printf(",%d", clk_source);
        printf(",%d", spi_freq);
        printf(",%d", prescaler);
        printf("\r\n");
    }
}

void device_init_start(void)
{
    b_init_finish = false;
}
bool device_init_finish_get(void)
{
    //	uart_tx_channel_set(UART_UPHOLE);
    //	if(reserve_get >0) {
    //		printf("@init,down,status,%d\n",module_status);
    //	}
    //	else printf("@init,down,water\n");
    // printf("@init,down,ok\n");
    return b_init_finish;
}

void device_process(void)
{
    if (b_init_finish == false)
    {
        if (dev_init_process())
        {
            // printf("@init,down,water\n"); //request water sensitivity
            //			HAL_Delay(250);
            //			printf("@init,down,ok\n");
        }
        return;
    }
    // printf("thre_h = %lf\n",h_sensor.h_adc->thre_h);
    // printf("water = %d\n",h_sensor.water[0]);
    // printf("acc_l: %lf\n",h_imu->accel_len);
    // printf("acc_x = %d\n",h_imu->accel_x);
    module_init();
    cmd_process();

    radar_system_process(); // Add this line only

    // water_detect();
    icm20948_process(&h_icm20948[0]);
    icm20948_process(&h_icm20948[1]);
    // if (b_water_det_printf) {
    //	b_water_det_printf = false;
    //	cmd_print_water(UART_UPHOLE); //event water
    // }

    imu_process(&h_imu[0]);
    imu_process(&h_imu[1]);
    // printf("@val,%d:%d:%d||%d,%d,%d\r\n",h_imu[0].accel_x,h_imu[0].accel_y,h_imu[0].accel_z,h_imu[1].accel_x,h_imu[1].accel_y,h_imu[1].accel_z);


    //	uart_tx_channel_set(UART_UPHOLE);
    //	if(state<= stopped_state) return;
    water_detect();
    keepalive_check();
    //	if(live_imu) imu_compare(&h_imu[0], &h_imu[1]);
    //	bottom_detect(&h_imu[1]);
    uart_tx_channel_set(UART_DEBUG);
    // printf("@val,water = %.3lf,%.3lf\n",adc_value_get(ADC_SEQ_WATER_BEGIN),adc_value_get(ADC_SEQ_WATER_BEGIN+1));
    // printf("@val,void = %.3lf,%.3lf,%.3lf\n",adc_value_get(ADC_SEQ_DIST_BEGIN),adc_value_get(ADC_SEQ_DIST_BEGIN+1),adc_value_get(ADC_SEQ_DIST_BEGIN+2));
    // printf("@val,temp = %.2f\n",adc_value_get(ADC_SEQ_TEMP));
    // printf("thre_h = %lf\n",h_sensor.h_adc->thre_h);
    // printf("acc_x = %d\n",h_imu->accel_x);
    // printf("@val,water = %d\n",adc_value[4]);
    // adc_sample_finish_cb(ADC_SEQ_WATER_BEGIN);

    log_save_process();
    log_load_process();
    flash_fifo_process(&h_flash_fifo_1);
    flash_process();

    dev_debug_process();
}

void device_restart(void)
{
    imu_reset(&h_imu[0], &h_imu[1]); // NC
    water_reset();
}

bool dev_init_process(void)
{
    typedef enum step_
    {
        STEP_IDLE,
        STEP_SPI,
        STEP_ADC,
        STEP_UART,
        STEP_COMMAND,
        STEP_FLASH,
        STEP_ICM20948,
        STEP_FINISH,
    } step_t;
    static step_t step = STEP_IDLE;

    static bool b_imu_init = false;

    uart_tx_channel_set(UART_DEBUG);
    switch (step)
    {
    case STEP_IDLE:
        if (b_init_finish)
        {
            break;
        }

        b_imu_init = false;

        if (h_dev_debug.b_init)
        {
            printf("\r\n");
            printf("/** STM project **/\r\n");
            printf("FW ver: %d,%d,%d\n", FW_VER_MAJOR, FW_VER_MINOR, FW_VER_SUB);
            // printf("%X", HW_VER);
            //			printf("%X", FW_VER_MAIN);
            //			printf("_%02X", FW_VER_SUB);
            //			printf("%s", FW_VER_NOTE);
            //			printf("\r\n");
            printf("Build Time: %s %s\r\n", __TIME__, __DATE__);
            printf("\r\n");
            printf("device init begin\r\n");

            printf("> adc_init\r\n");
        }
        step = STEP_SPI;
        break;
    case STEP_SPI:

        dev_spi_init(0);
        dev_spi_init(1);

        h_icm20948[0].b_init_finish = false;
        h_icm20948[1].b_init_finish = false;

        b_imu_init = true;
        step       = STEP_ADC;
        break;
    case STEP_ADC:
        adc_init(ADC_HAL_SEQ_TEMP);
        adc_init(ADC_HAL_SEQ_WATER);
        adc_sample_rate_set(ADC_HAL_SEQ_TEMP, DEV_ADC_SAMPLE_RATE_DEFAULT);
        adc_sample_rate_set(ADC_HAL_SEQ_WATER, DEV_ADC_SAMPLE_RATE_DEFAULT);
        adc_en_set(ADC_HAL_SEQ_TEMP, true);
        adc_en_set(ADC_HAL_SEQ_WATER, true);
        if (h_dev_debug.b_init)
        {
            printf("> uart_init\r\n");
        }
        step = STEP_UART;
        break;
    case STEP_UART:
        uart_init();
        if (h_dev_debug.b_init)
        {
            printf("> command_init\r\n");
        }
        step = STEP_COMMAND;
        break;
    case STEP_COMMAND:
        cmd_init();
        step = STEP_FLASH;
    case STEP_FLASH:
        if (h_flash_fifo_1.b_init_finish == false)
        {
            break;
        }
        step = STEP_ICM20948;
    case STEP_ICM20948:
        if (h_icm20948[0].b_init_finish == false)
        {
            break;
        }
        if (h_icm20948[1].b_init_finish == false)
        {
            break;
        }

        b_imu_init = false;
        // math_pi = acos(-1.0);
        //		printf("> PI:%.17f\r\n", math_pi);
        h_icm20948[0].b_sample_en = true;
        h_icm20948[1].b_sample_en = true;

        imu_g_log_en_set(0, true);
        imu_g_log_en_set(1, true);

        imu_sample_rate_set(DEV_IMU_SAMPLE_RATE);
        HAL_TIM_Base_Start_IT(&IMU_SAMPLE_HAL_TIMER);
        h_icm20948[0].b_sample_start = true;
        h_icm20948[1].b_sample_start = true;


        if (h_dev_debug.b_init)
        {
            printf("> ICM20948_init_finish\r\n");
        }
        step = STEP_FINISH;
        break;
    case STEP_FINISH:
        if (h_dev_debug.b_init)
        {
            printf("device init finish\r\n");
            printf("\r\n");
            // HAL_Delay(500);
        }
        b_init_finish = true;
        step          = STEP_IDLE;
    }

    if (h_flash_fifo_1.b_init_finish == false)
    {
        flash_fifo_process(&h_flash_fifo_1);
    }
    if (b_imu_init)
    {
        HAL_GPIO_WritePin(PWR_IMU_GPIO_Port, PWR_IMU_Pin, GPIO_PIN_SET);
        icm20948_process(&h_icm20948[0]);
        icm20948_process(&h_icm20948[1]);
        HAL_GPIO_WritePin(PWR_IMU_GPIO_Port, PWR_IMU_Pin, GPIO_PIN_SET);
    }
    uart_tx_channel_set(UART_UPHOLE);
    return b_init_finish;
}

static bool imu_process(h_imu_t *p_h_imu)
{
    typedef enum step_
    {
        STEP_IDLE = 0,
        STEP_VECTOR_LEN,
        STEP_VECTOR_ANGLE,
        STEP_VECTOR_VARIANCE,
        STEP_GRAVITY_UPDATE,
        STEP_GRAVITY_FINISH,
        STEP_FINISH,
    } step_t;
    // if(p_h_imu->id == 0) printf("IMU 0 STEP: %d\n",p_h_imu->step);
    switch (p_h_imu->step)
    {
    case STEP_FINISH:
        // printf("\n");
        p_h_imu->step = STEP_IDLE;
    case STEP_IDLE:
        if (p_h_imu->b_g_log_en)
        {
            p_h_imu->b_g_log_busy = true;
            moving_avg_i16_reset(p_h_imu->p_h_g_squ_avg[0]);
            moving_avg_i16_reset(p_h_imu->p_h_g_squ_avg[1]);
            moving_avg_i16_reset(p_h_imu->p_h_g_squ_avg[2]);
            p_h_imu->step = STEP_GRAVITY_UPDATE;
            break;
        }
        if (p_h_imu->b_update == false)
        {
            break;
        }
        p_h_imu->step = STEP_VECTOR_LEN;
    case STEP_VECTOR_LEN:
    {
        uart_tx_channel_set(UART_DEBUG);
        // printf("IMU: %d", p_h_imu->id);
        h_vector_t *p_h_vector = &p_h_imu->h_accel;
        int16_t    *p_data;
        int16_t     data;
        uint64_t    data_squ = 0;
        p_data               = p_h_vector->data_org;
        for (uint8_t axis_n = 0; axis_n < DEV_IMU_VECTOR_AXIS_N; axis_n++)
        {
            data = p_data[axis_n];
            data_squ += data * data;
        }
        p_h_vector->squ = data_squ;
        p_h_vector->len = sqrt(data_squ);

        p_h_vector = &p_h_imu->h_gyro;
        data_squ   = 0;
        p_data     = p_h_vector->data_org;
        for (uint8_t axis_n = 0; axis_n < DEV_IMU_VECTOR_AXIS_N; axis_n++)
        {
            data = p_data[axis_n];
            data_squ += data * data;
        }
        p_h_vector->squ = data_squ;
        p_h_vector->len = sqrt(data_squ);
        // printf("|L: %lf",p_h_imu->h_accel.len);
        p_h_imu->step = STEP_VECTOR_ANGLE;
    }
    break;
    case STEP_VECTOR_ANGLE:
    {
        if (p_h_imu->h_accel.len - 2070 < 350 && p_h_imu->h_accel.len - 2070 > -350)
        { // not accelerating
            int16_t    *p_gravity  = p_h_imu->gravity;
            h_vector_t *p_h_vector = &p_h_imu->h_accel;
            int16_t    *p_data     = p_h_vector->data_org;
            int64_t     dot        = 0;
            for (uint8_t axis_n = 0; axis_n < DEV_IMU_VECTOR_AXIS_N; axis_n++)
            {
                dot += p_data[axis_n] * p_gravity[axis_n];
            }
            // double cos = (double) dot / (p_h_vector->len * p_h_imu->gravity_len);
            double cos        = (p_h_imu->accel_y / p_h_imu->h_accel.len);
            p_h_vector->angle = 180.0 * acos(cos) / math_pi;

            p_h_vector = &p_h_imu->h_gyro;
            p_data     = p_h_vector->data_org;
            dot        = 0;
            for (uint8_t axis_n = 0; axis_n < DEV_IMU_VECTOR_AXIS_N; axis_n++)
            {
                dot += p_data[axis_n] * p_gravity[axis_n];
            }
            cos = (double)dot / (p_h_vector->len * p_h_imu->gravity_len);

            p_h_vector->angle = 180.0 * acos(cos) / math_pi;

            //		p_h_imu->step = STEP_VECTOR_VARIANCE;
            //	}
            //		break;
            //	case STEP_VECTOR_VARIANCE: {
            //		h_vector_t *p_h_vector = &p_h_imu->h_accel;
            //		h_moving_avg_f_t *p_h_avg = p_h_vector->p_h_avg;
            //		h_moving_avg_f_t *p_h_squ_avg = p_h_vector->p_h_squ_avg;
            //		moving_avg_f_update(p_h_avg, p_h_vector->len);
            //		moving_avg_f_update(p_h_squ_avg, p_h_vector->squ);
            //		double avg = p_h_avg->avg;
            //		p_h_vector->variance = p_h_vector->squ - avg * avg;
            //
            //		p_h_vector = &p_h_imu->h_gyro;
            //		p_h_avg = p_h_vector->p_h_avg;
            //		p_h_squ_avg = p_h_vector->p_h_squ_avg;
            //		moving_avg_f_update(p_h_avg, p_h_vector->len);
            //		moving_avg_f_update(p_h_squ_avg, p_h_vector->squ);
            //		double avg = p_h_avg->avg;
            //		p_h_vector->variance = p_h_vector->squ - avg * avg;
        }
        // printf("|A: %lf",p_h_imu->h_accel.angle);
        p_h_imu->b_update = false;
        imu_update_finish_cb();
        p_h_imu->step = STEP_FINISH;
    }
    break;
    case STEP_GRAVITY_UPDATE:
    {
        if (p_h_imu->b_update == false)
        {
            // printf("IMU: %d | update %d\n",p_h_imu->id,p_h_imu->b_update);
            break;
        }
        h_moving_avg_i16_t **pp_h_g_squ_avg = p_h_imu->p_h_g_squ_avg;
        bool                 b_update       = p_h_imu->b_g_log_en;
        b_update &= pp_h_g_squ_avg[0]->data_len < DEV_IMU_AVG_SAMPLE_NUM;
        if (b_update)
        {
            int16_t *p_data = p_h_imu->data;
            moving_avg_i16_update(pp_h_g_squ_avg[0], p_data[ICM20948_DATA_AX]);
            moving_avg_i16_update(pp_h_g_squ_avg[1], p_data[ICM20948_DATA_AY]);
            moving_avg_i16_update(pp_h_g_squ_avg[2], p_data[ICM20948_DATA_AZ]);
            p_h_imu->b_update = false;
            break;
        }
        p_h_imu->step = STEP_GRAVITY_FINISH;
    }
    case STEP_GRAVITY_FINISH:
    {
        h_moving_avg_i16_t **pp_h_g_squ_avg = p_h_imu->p_h_g_squ_avg;
        p_h_imu->gravity[0]                 = pp_h_g_squ_avg[0]->avg;
        p_h_imu->gravity[1]                 = pp_h_g_squ_avg[1]->avg;
        p_h_imu->gravity[2]                 = pp_h_g_squ_avg[2]->avg;

        int16_t *p_data = p_h_imu->gravity;
        int16_t  data;
        uint64_t data_squ;
        data     = p_data[0];
        data_squ = data * data;
        data     = p_data[1];
        data_squ += data * data;
        data = p_data[2];
        data_squ += data * data;
        p_h_imu->gravity_len  = sqrt(data_squ);
        p_h_imu->b_g_log_en   = false;
        p_h_imu->b_g_log_busy = false;

        //		bool b_busy = imu_g_log_busy_get(0);
        //		b_busy |= imu_g_log_busy_get(1);
        //		if (b_busy == false)
        //			cmd_print_init_g_log(UART_UPHOLE);
        p_h_imu->step = STEP_FINISH;
    }
    }
    uart_tx_channel_set(UART_DEBUG);
    uart_tx_channel_undo();
    return p_h_imu->step != STEP_IDLE;
}

static bool log_save_process(void)
{
    typedef enum step_
    {
        STEP_START_CHECK,
        STEP_IDLE,
        STEP_WRITE_SET,
    } step_t;
    static step_t               step              = STEP_START_CHECK;
    static h_flash_fifo_order_t h_log_flash_order = {
        .data_size = sizeof(h_dev_log_t),
    };

    switch (step)
    {
    case STEP_START_CHECK:
        step = STEP_IDLE;
    case STEP_IDLE:
        if (b_log_save_trigger == false)
        {
            return false;
        }
        h_log_flash_order.p_data = &h_log_save_q[log_save_q_new];
        b_log_save_trigger       = false;

        step = STEP_WRITE_SET;
    case STEP_WRITE_SET:
    {
        flash_fifo_res_t res;
        res = flash_fifo_write(&h_flash_fifo_1, &h_log_flash_order);
        if (res == FLASH_FIFO_RES_BUSY)
        {
            break;
        }

        if (res != FLASH_FIFO_RES_OK)
        {
            dev_log_save_enable_set(false);
            cmd_print_log_save(UART_ALL);
        }

        //		{
        //			static uint16_t log_count = 0;
        //			log_count++;
        //			static uint32_t time_log = 0;
        //			if (HAL_GetTick() - time_log >= 1000) {
        //				time_log = HAL_GetTick();
        //				printf("S:%d\n", log_count);
        //				log_count = 0;
        //			}
        //		}
        step = STEP_START_CHECK;
    }
    break;
    default:
        step = STEP_START_CHECK;
    }
    return step != STEP_IDLE;
}

static bool log_load_process(void)
{
    typedef enum step_
    {
        STEP_START,
        STEP_READ,
    } step_t;
    static step_t               step          = STEP_START;
    static size_t               seq_load      = 0;
    static h_flash_fifo_order_t h_flash_order = {
        .p_data    = &h_log_load,
        .data_size = sizeof(h_dev_log_t),
    };

    if (b_log_report_act == false)
    {
        step = STEP_START;
        return false;
    }

    switch (step)
    {
    case STEP_START:
        seq_load = h_flash_fifo_1.fifo_front;

        step = STEP_READ;
    case STEP_READ:
    {
        if (seq_load == h_flash_fifo_1.fifo_back)
        {
            step             = STEP_START;
            b_log_report_act = false;
            cmd_print_log_report_busy(log_report_uart_ch);
            break;
        }

        flash_fifo_res_t res;
        res = flash_fifo_read_seq(&h_flash_fifo_1, seq_load, &h_flash_order);
        if (res == FLASH_FIFO_RES_OK)
        {
            cmd_print_log_report_data(log_report_uart_ch);
        }

        if (++seq_load >= h_flash_fifo_1.h_cfg.pack_in_memory)
        {
            seq_load = 0;
        }
    }
    break;
    default:
        step = STEP_START;
    }
    return true;
}

static void dev_debug_process(void)
{
    if (h_dev_debug.b_adc_sample)
    {
        static uint32_t time_log = 0;
        uint32_t        time_now = HAL_GetTick();
        if (time_now - time_log >= 1000)
        {
            time_log = time_now;
            printf("> dev_adc:sample\r\n");
            //			for (adc_seq_t seq = ADC_SEQ_BEGIN; seq < ADC_SEQ_NUM; seq++)
            //				printf(",%d", adc_value_org_get(seq));
            //			printf("\r\n");
            for (adc_seq_t seq = ADC_SEQ_BEGIN; seq < ADC_SEQ_NUM; seq++)
            {
                printf(",%6.3f", h_sensor.h_adc[seq].volt);
            }
            printf("\r\n");
            for (adc_seq_t seq = ADC_SEQ_BEGIN; seq < ADC_SEQ_NUM; seq++)
            {
                printf(",%d", h_sensor.h_adc[seq].b_level);
            }
            printf("\r\n");
        }
    }
    if (h_dev_debug.b_imu_sample)
    {
        static uint32_t time_log = 0;
        uint32_t        time_now = HAL_GetTick();
        if (time_now - time_log >= 1000)
        {
            time_log               = time_now;
            uint32_t count         = imu_debug_sample_count;
            imu_debug_sample_count = 0;
            //			printf("> dev_imu:sample\r\n");
            printf("> dev_imu:sample,%d\r\n", count);
            h_imu_t *p_h_imu = &h_imu[0];
            int16_t *p_buff  = p_h_imu->data;
            for (uint8_t data_n = 0; data_n < ICM20948_DATA_NUM; data_n++)
            {
                printf(",%6d", p_buff[data_n]);
            }
            printf(",%f", p_h_imu->h_accel.angle);
            printf(",%f", p_h_imu->h_gyro.angle);
            //			printf(",%f", p_h_imu->h_accel.variance);
            //			printf(",%f", p_h_imu->h_gyro.variance);
            printf("\r\n");
            p_h_imu = &h_imu[1];
            p_buff  = p_h_imu->data;
            for (uint8_t data_n = 0; data_n < ICM20948_DATA_NUM; data_n++)
            {
                printf(",%6d", p_buff[data_n]);
            }
            printf(",%f", p_h_imu->h_accel.angle);
            printf(",%f", p_h_imu->h_gyro.angle);
            //			printf(",%f", p_h_imu->h_accel.variance);
            //			printf(",%f", p_h_imu->h_gyro.variance);
            printf("\r\n");
        }
    }

    // Add radar debug - every 2 seconds
    if (h_dev_debug.b_radar_sample)
    {
        static uint32_t time_log = 0;
        uint32_t        time_now = HAL_GetTick();
        if (time_now - time_log >= 2000)
        { // Every 2 seconds
            time_log = time_now;

            printf("> radar:status"); // Remove sensor count

            for (uint8_t i = 0; i < MAX_RADAR_SENSORS; i++)
            {
                if (radar_has_valid_data(i))
                {
                    printf(",S%d:%umm", i, radar_get_distance_mm(i));
                }
                else
                {
                    printf(",S%d:no_data", i);
                }
            }
            printf("\r\n");
        }
    }
}

void dev_printf_debug_set(h_dev_debug_t *p_h_flag)
{
    memcpy(&h_dev_debug, p_h_flag, sizeof(h_dev_debug_t));
}
void dev_printf_debug_get(h_dev_debug_t *p_h_flag)
{
    memcpy(p_h_flag, &h_dev_debug, sizeof(h_dev_debug_t));
}

dev_res_t dev_spi_send(h_dev_spi_send_t *p_h_spi)
{
    uint8_t channel = p_h_spi->channel;
    if (channel >= DEV_SPI_CH_NUM)
    {
        return DEV_RES_ERROR_PARA;
    }
    switch (HAL_SPI_GetState(p_h_spi_hal[channel]))
    {
    case HAL_SPI_STATE_READY:
        break;
    case HAL_SPI_STATE_RESET:
    case HAL_SPI_STATE_ERROR:
        return DEV_RES_ERROR;
    default:
        return DEV_RES_BUSY;
    }

    memcpy(&h_spi, p_h_spi, sizeof(h_dev_spi_send_t));

    h_dev_spi_t  *p_h_dev_spi_ch = &h_dev_spi[channel];
    GPIO_TypeDef *p_cs_port      = p_h_dev_spi_ch->p_cs_port;
    uint16_t      cs_pin         = p_h_dev_spi_ch->cs_pin;
    HAL_GPIO_WritePin(p_cs_port, cs_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef res_hal;
    res_hal = HAL_SPI_TransmitReceive_DMA(p_h_spi_hal[p_h_spi->channel], p_h_spi->p_tx_buff, p_h_spi->p_rx_buff, p_h_spi->data_size);

    dev_res_t res;
    switch (res_hal)
    {
    case HAL_OK:
        res = DEV_RES_SUCCESS;
        break;
    case HAL_BUSY:
        res = DEV_RES_BUSY;
        break;
    default:
        res = DEV_RES_ERROR;
    }

    if (res != DEV_RES_SUCCESS)
    {
        HAL_GPIO_WritePin(p_cs_port, cs_pin, GPIO_PIN_SET);
    }
    return res;
}

void dev_dump_printf_set(bool b_enable)
{
    b_dump_printf_en = b_enable;
    if (b_dump_printf_en)
    {
        b_dump_log = false;
    }
}

bool dev_dump_get(void)
{
    return b_dump;
}

void dev_water_det_printf_set(bool b_enable)
{
    b_water_det_en = b_enable;
    if (b_water_det_en)
    {
        b_water_det_log = false;
    }
}

bool dev_water_det_get(void)
{
    return b_water_det;
}

void imu_sample_rate_set(double sample_rate)
{
    uint32_t period_tick = HAL_RCC_GetPCLK1Freq() / sample_rate;
    uint32_t prescaler   = (period_tick + UINT16_MAX) / (UINT16_MAX + 1);
    uint32_t reload      = period_tick / prescaler;
    __HAL_TIM_SET_PRESCALER(&IMU_SAMPLE_HAL_TIMER, prescaler - 1);
    __HAL_TIM_SET_AUTORELOAD(&IMU_SAMPLE_HAL_TIMER, reload - 1);

    if (h_dev_debug.b_imu_sample_set)
    {
        printf("> imu:sample_rate");
        printf(",%.3f", sample_rate);
        printf(",%d", period_tick);
        printf(",%d", prescaler);
        printf(",%d", reload);
        printf("\r\n");
    }
}

void imu_g_log_en_set(uint8_t imu_select, bool b_enable)
{
    if (imu_select >= DEV_IMU_NUM)
    {
        return;
    }
    h_imu[imu_select].b_g_log_en = b_enable;
}

bool imu_g_log_busy_get(uint8_t imu_select)
{
    h_imu_t *p_h_imu  = &h_imu[imu_select];
    bool     b_finish = p_h_imu->b_g_log_en;
    b_finish |= p_h_imu->b_g_log_busy;
    return b_finish;
}

bool imu_overflow_get(uint8_t imu_select)
{
    if (imu_select >= DEV_IMU_NUM)
    {
        return false;
    }
    return h_imu[imu_select].b_overflow == false;
}

void imu_overflow_clear(uint8_t imu_select)
{
    if (imu_select >= DEV_IMU_NUM)
    {
        return;
    }
    h_imu[imu_select].b_overflow = false;
}


bool dev_log_erase_all_set(void)
{
    b_log_erase_all_printf = true;
    flash_fifo_res_t res   = flash_fifo_erase_all(&h_flash_fifo_1);
    if (res != FLASH_FIFO_RES_OK)
    { /* flash fifo is busy */
        return false;
    }
    dev_log_save_enable_set(false);
    return true;
}

bool dev_log_erase_busy_get(void)
{
    return h_flash_fifo_1.b_erase_busy;
}

bool dev_log_save_enable_set(bool b_enable)
{
    if (flash_fifo_pack_n_get_free(&h_flash_fifo_1) <= 0)
    {
        b_enable = false;
    }
    if (b_enable)
    {
        b_log_save_overflow = false;
        log_save_time_start = HAL_GetTick();
        b_log_save_act      = true;
    }
    else
    {
        b_log_save_act     = false;
        b_log_save_trigger = false;
    }
    return b_enable;
}

bool dev_log_save_busy_get(void)
{
    return b_log_save_act || h_flash_fifo_1.h_write.b_busy;
}

bool dev_log_save_overflow_get(void)
{
    return b_log_save_overflow;
}

// size_t dev_log_save_free_space_get(void) {
//	return flash_fifo_pack_n_get_free(&h_flash_fifo_1);
// }

void dev_log_report_uart_ch_set(uart_select_t channel)
{
    log_report_uart_ch = channel;
}

bool dev_log_report_enable_set(bool b_enable)
{
    if (flash_fifo_pack_n_get_saved(&h_flash_fifo_1) <= 0)
    {
        b_enable = false;
    }
    b_log_report_act = b_enable;
    return b_enable;
}

bool dev_log_report_busy_get(void)
{
    return b_log_report_act;
}

void dev_log_data_get(h_dev_log_t *p_h_log)
{
    memcpy(p_h_log, &h_log_load, sizeof(h_dev_log_t));
}

// log_res_t dev_log_data_next(void) {
//	if (b_log_next_busy)
//		return LOG_RES_BUSY;
//	b_log_next_busy = true;
//	flash_fifo_res_t res = flash_fifo_delete_front(&h_flash_fifo_1);
//	switch (res) {
//	case FLASH_FIFO_RES_OK:
//		return LOG_RES_OK;
//	case FLASH_FIFO_RES_BUSY:
//		b_log_next_busy = false;
//		return LOG_RES_BUSY;
//	default:
//		b_log_next_busy = false;
//		return LOG_RES_ERROR;
//	}
// }

void dev_sleep(void)
{
    printf("@db,Downhole sleep\r\n");

    /* UART RX OFF */
    uart_rx_en_set(UART_DEBUG, false);
    uart_rx_en_set(UART_UPHOLE, false);

    /* IMU OFF */
    h_icm20948[0].b_sample_en = false;
    h_icm20948[1].b_sample_en = false;
    HAL_SPI_DMAStop(h_dev_spi[0].p_h_hal);
    HAL_SPI_DMAStop(h_dev_spi[1].p_h_hal);

    /* ADC OFF */
    adc_en_set(ADC_HAL_SEQ_TEMP, false);
    adc_en_set(ADC_HAL_SEQ_WATER, false);
    adc_en_set(ADC_HAL_SEQ_WATER, false);


    uart_tx_wait_sent(UART_DEBUG, 100);

    /* UART RX pin set to event mode */
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin   = DEBUG_RXD_Pin,
        .Mode  = GPIO_MODE_EVT_FALLING,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = UPHOLE_RXD_Pin;
    HAL_GPIO_Init(UPHOLE_GPIO_Port, &GPIO_InitStruct);

    /* sleep start */
    HAL_SuspendTick();

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);

    /* is wake up */
    //	HAL_ResumeTick();
    SystemClock_Config();

    /* UART pin set to UART */
    GPIO_InitStruct.Pin       = UPHOLE_RXD_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(UPHOLE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = DEBUG_RXD_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);

    /* ADC ON */
    adc_en_set(ADC_HAL_SEQ_TEMP, true);
    adc_en_set(ADC_HAL_SEQ_WATER, true);
    adc_en_set(ADC_HAL_SEQ_WATER, true);

    /* IMU ON */
    h_icm20948[0].b_sample_en = true;
    h_icm20948[1].b_sample_en = true;

    /* UART RX ON */
    uart_rx_en_set(UART_DEBUG, true);
    uart_rx_en_set(UART_UPHOLE, true);

    printf("@db,wake\r\n");
    cmd_print_wake(UART_ALL);
}

void moving_avg_f_update(h_moving_avg_f_t *p_h_avg, float new_data)
{
    if (p_h_avg->p_buff == NULL)
    {
        return;
    }
    p_h_avg->data_n %= p_h_avg->buff_len;
    p_h_avg->p_buff[p_h_avg->data_n] = new_data;
    p_h_avg->data_n                  = (p_h_avg->data_n + 1) % p_h_avg->buff_len;
    p_h_avg->sum += new_data;
    if (p_h_avg->data_len < p_h_avg->buff_len - 1)
    {
        p_h_avg->data_len++;
    }
    else
    {
        p_h_avg->sum -= p_h_avg->p_buff[p_h_avg->data_n];
        p_h_avg->data_len = p_h_avg->buff_len - 1;
    }
    p_h_avg->avg = p_h_avg->sum / p_h_avg->data_len;
}

void moving_avg_f_reset(h_moving_avg_f_t *p_h_avg)
{
    memset(p_h_avg->p_buff, 0x0, sizeof(*p_h_avg->p_buff) * p_h_avg->buff_len);
    p_h_avg->data_len = 0;
    p_h_avg->data_n   = 0;
    p_h_avg->sum      = 0;
    p_h_avg->avg      = 0;
}

void moving_avg_i16_update(h_moving_avg_i16_t *p_h_avg, int16_t new_data)
{
    if (p_h_avg->p_buff == NULL)
    {
        return;
    }
    p_h_avg->data_n %= p_h_avg->buff_len;
    p_h_avg->p_buff[p_h_avg->data_n] = new_data;
    p_h_avg->data_n                  = (p_h_avg->data_n + 1) % p_h_avg->buff_len;
    p_h_avg->sum += new_data;
    if (p_h_avg->data_len < p_h_avg->buff_len - 1)
    {
        p_h_avg->data_len++;
    }
    else
    {
        p_h_avg->sum -= p_h_avg->p_buff[p_h_avg->data_n];
        p_h_avg->data_len = p_h_avg->buff_len - 1;
    }
    p_h_avg->avg = p_h_avg->sum / p_h_avg->data_len;
}

void moving_avg_i16_reset(h_moving_avg_i16_t *p_h_avg)
{
    //	memset(p_h_avg->p_buff, 0x0, sizeof(*p_h_avg->p_buff) * p_h_avg->buff_len);
    p_h_avg->data_len = 0;
    p_h_avg->data_n   = 0;
    p_h_avg->sum      = 0;
    p_h_avg->avg      = 0;
}
