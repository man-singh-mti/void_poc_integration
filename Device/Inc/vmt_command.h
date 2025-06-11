#ifndef __VMT_COMMAND_H__
#define __VMT_COMMAND_H__

#pragma anon_unions
#include <stdint.h>

#include "vmt_uart.h"
#include "vmt_string.h"

/* parameter */
#define CMD_VER_MAIN (1)
#define CMD_VER_SUB  (7)

/* variable */

/* function */
void cmd_init(void);
void cmd_process(void);

void cmd_print_bottom(uart_select_t channel);
void cmd_print_water(uart_select_t channel);
void cmd_print_init_g_log(uart_select_t channel);

void cmd_print_log_save(uart_select_t channel);
void cmd_print_log_report_busy(uart_select_t channel);
void cmd_print_log_report_data(uart_select_t channel);
void cmd_print_log_erase(uart_select_t channel);
void cmd_print_wake(uart_select_t channel);

void cmd_print_flash_fifo_write_finish(uart_select_t channel);
void cmd_print_flash_fifo_delete_finish(uart_select_t channel);
void cmd_print_flash_fifo_mark_finish(uart_select_t channel);

// Add temperature command printing functions (aligned with water pattern)
void cmd_print_temp_status(uart_select_t channel);
void cmd_print_temp_alert(uart_select_t channel);

void void_send_automatic_stream(void); // Add void streaming function

#endif /*__VMT_COMMAND_H__ */
