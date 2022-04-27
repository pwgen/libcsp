/* usart driver for use with esp32-idf ew@eric-weiss.de */

#include <csp/drivers/usart.h>

#include <csp/csp_debug.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <malloc.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include <csp/csp.h>
#include <pthread.h>
#define ECHO_TEST_TXD  (GPIO_NUM_1)
#define ECHO_TEST_RXD  (GPIO_NUM_3)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

//static char tx_item[] = "";
static const char* TAG = "ESP32_CSP-UART";
char buffer[16];
char buf[32];
char* test_str = "ESP32USART_1 READY\n";
TaskHandle_t task_usart_t;

 uint8_t *data ;
typedef struct {
	csp_usart_callback_t rx_callback;
	void * user_data;
	csp_usart_fd_t fd;
	pthread_t rx_thread;
} usart_context_t;
usart_context_t * ctx ;

int q;
const uart_port_t uart_num = UART_NUM_1;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    };

/* Linux is fast, so we keep it simple by having a single lock */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

void csp_usart_lock(void * driver_data) {
	pthread_mutex_lock(&lock);
}

void csp_usart_unlock(void * driver_data) {
	pthread_mutex_unlock(&lock);
}

static void  usart_rx_thread(void * arg) {
	usart_context_t * ctx=arg;
    int length=0;
   while (1) {
		int length = uart_read_bytes(uart_num, data, 255,  20 / 1);
		if (length <= 0) {
			vTaskDelay(1);
		}
		else
		{
  		    ctx->rx_callback(ctx->user_data, data, length, NULL);
		}
	}
}

int csp_usart_write(csp_usart_fd_t fd, const void * data, size_t data_length) 
{
    return uart_write_bytes(uart_num, (const char*)data, data_length);
}

int csp_usart_open(const csp_usart_conf_t * conf, csp_usart_callback_t rx_callback, void * user_data, csp_usart_fd_t * return_fd) 
{

    BaseType_t stat;
//    ESP_LOGI(TAG, "usart open called %p ",rx_callback);
    usart_context_t * ctx = calloc(1, sizeof(*ctx));
	data = (uint8_t *) malloc(BUF_SIZE);
	if (ctx == NULL) {
		ESP_LOGI(TAG,"%s: Error allocating context, device: [%s], errno: %s\n", __FUNCTION__, conf->device, strerror(errno));
		
		return CSP_ERR_NOMEM;
	}
	ctx->rx_callback = rx_callback;
	ctx->user_data = user_data;

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
//    ESP_ERROR_CHECK(uart_set_pin(uart_num,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_pin(uart_num,17, 16,  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // esp32 specific
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
    
	stat=xTaskCreate((TaskFunction_t) &usart_rx_thread, "usart_rx_thread", 2048, ctx, 5, &task_usart_t);
	
	return CSP_ERR_NONE;
}
