#include <stm32f10x.h>
#include "GPIO_STM32F10x.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "serial.h"


#define RUN_LED_IONUM				7
#define RUN_LED_BIT					(1<<RUN_LED_IONUM)
#define GSM_PWRCTRL					0
#define GSM_PWRCTRL_BIT			(1<<GSM_PWRCTRL)
#define GSM_PWRKEY					1
#define GSM_PWRKEY_BIT			(1<<GSM_PWRKEY)
#define GSM_STATUS					4
#define GSM_STATUS_BIT			(1<<GSM_STATUS)

#define BUF_LEN			100
															

int cmd_port_h = -1, gsm_port_h = -1;
uint8_t gsm_status = 0;


void init_gpio()
{
	GPIO_PinConfigure(GPIOA, RUN_LED_IONUM, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT50MHZ);
	GPIOA->ODR |= RUN_LED_BIT;
	
	GPIO_PinConfigure(GPIOA, GSM_PWRCTRL, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT50MHZ);
	GPIOA->ODR &= ~GSM_PWRCTRL_BIT;
	
	GPIO_PinConfigure(GPIOA, GSM_PWRKEY, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT50MHZ);
	GPIOA->ODR &= ~GSM_PWRKEY_BIT;
	
	GPIO_PinConfigure(GPIOA, GSM_STATUS, GPIO_IN_FLOATING, GPIO_MODE_INPUT);
}

void init_periph_clk()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, 
													ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
													RCC_APB2Periph_GPIOA |
													RCC_APB2Periph_USART1,
													ENABLE);
}

void init_nvic()
{
	//Must be set to NVIC_PriorityGroup_4
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}	

void setup_hardware(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	
	init_periph_clk();
	init_gpio();
	init_nvic();
	
	cmd_port_h = init_serial_port(USART1, 115200, 100, 100, UART_MODE);
	configASSERT(cmd_port_h >= 0);
	gsm_port_h = init_serial_port(USART2, 115200, 100, 100, UART_MODE);
	configASSERT(gsm_port_h >= 0);
}

uint8_t check_gsm_status()
{
	return GPIOA->IDR&GSM_STATUS_BIT ? 1 : 0;
}

void gsm_power_on()
{
	static const char *resp_success = "\r\n<POWER ON>\r\n", 
										*resp_fail = "\r\n<POWER ON FAILED>\r\n"	;
	uint8_t retry = 0;
	
	if (check_gsm_status())
		return;
	
	GPIOA->ODR |= GSM_PWRCTRL_BIT;			
	vTaskDelay(1000);	
	
	GPIOA->ODR |= GSM_PWRKEY_BIT;		
	vTaskDelay(1500);								
	GPIOA->ODR &= ~GSM_PWRKEY_BIT;	
	
	while ((gsm_status = check_gsm_status()) == 0) {
		if (retry++ == 10) {
			write_serial(cmd_port_h, (uint8_t *)resp_fail, 
									strlen(resp_fail), portMAX_DELAY);
			return;
		}	
			
		vTaskDelay(500);
	}
	
	write_serial(cmd_port_h, (uint8_t *)resp_success, 
								strlen(resp_success), portMAX_DELAY);
}

void gsm_power_off()
{
	static const char *resp = "\r\n<POWER OFF>\r\n";
	
	GPIOA->ODR |= GSM_PWRKEY_BIT;		
	vTaskDelay(1500);								
	GPIOA->ODR &= ~GSM_PWRKEY_BIT;	
	vTaskDelay(1000);	
	
	GPIOA->ODR &= ~GSM_PWRCTRL_BIT;	
	
	gsm_status = 0;
	write_serial(cmd_port_h, (uint8_t *)resp, strlen(resp), portMAX_DELAY);
}

void downlink_task(void *p)
{
	uint8_t str_len = 0, c;
	static uint8_t string[BUF_LEN];
	static const char *pon_cmd = "#POWERON\r",
										*poff_cmd = "#POWEROFF\r";
	
	vTaskDelay(1000);
	gsm_status = check_gsm_status();
	
	while (1) {
		read_serial(cmd_port_h, &c, 1, portMAX_DELAY);
		if (c != '\n')
			string[str_len++] = c;
		
		if (c == '\r') {
			if (strncmp((const char *)string, pon_cmd, str_len) == 0) 
				gsm_power_on();
			else if (strncmp((const char *)string, poff_cmd, str_len) == 0) 
 				gsm_power_off();
			else if (gsm_status) 	
				write_serial(gsm_port_h, string, str_len, portMAX_DELAY);
				
			str_len = 0;
		} else if (str_len >= BUF_LEN)
			str_len = 0;
	}	
}	

void uplink_task(void *p)
{
	uint8_t c;

	while (1) {
		read_serial(gsm_port_h, &c, 1, portMAX_DELAY);
		write_serial(cmd_port_h, &c, 1, portMAX_DELAY);
	}	
}	

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                    signed char *pcTaskName) 
{
	for ( ; ; ) {
		__WFI();
	}	
}																			

int main(void)
{
	setup_hardware();
	
	xTaskCreate(uplink_task, "uplink", 64, (void *)NULL, 
							tskIDLE_PRIORITY+1, NULL);	
	
	xTaskCreate(downlink_task, "downlink", 64, (void *)NULL, 
							tskIDLE_PRIORITY+1, NULL);	
		
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* Will only get here if there was insufficient memory to create the idle
   task.  The idle task is created within vTaskStartScheduler(). */
	for ( ; ; ) {
		__WFI();
	}
	
	return 0;
}

