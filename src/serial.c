#include <stm32f10x.h>

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "stm32f10x_gpio.h"
#include "serial.h"


struct serial_port_hardware_desc {
	USART_TypeDef *port;
	GPIO_TypeDef *gpio;
	uint16_t tx_pin;
	uint16_t rx_pin;
	uint32_t remap_bit;
	uint8_t irq;
};

struct serial_port_resources {
	USART_TypeDef *port;
	uint32_t irq;
	QueueHandle_t rx_q;
	QueueHandle_t tx_q;
	uint32_t lin_mode;
};


static const struct serial_port_hardware_desc hardware_desc[] = {
	{ USART1, GPIOA, GPIO_Pin_9, GPIO_Pin_10, 0, USART1_IRQn },
	{	USART2, GPIOA, GPIO_Pin_2, GPIO_Pin_3, 0, USART2_IRQn }	
};	
					
#define PORT_NUM 				(sizeof(hardware_desc)/sizeof(struct serial_port_hardware_desc))

static struct serial_port_resources port_res[PORT_NUM] = {0};


int init_serial_port(USART_TypeDef *port, uint32_t baudrate, 
										uint32_t rx_queue_size, uint32_t tx_queue_size, uint32_t mode)
{
	int i, handle = -1;
	UBaseType_t s;
	USART_InitTypeDef usart_init; 
	NVIC_InitTypeDef nvic_init;
	GPIO_InitTypeDef gpio_init;
				
	for (i = 0; i < PORT_NUM; i++) {
		if (port == hardware_desc[i].port) {
			handle = i;
			break;
		}	
	}	
	
	if (handle == -1)
		return handle;
	
	port_res[handle].port = port;
	port_res[handle].irq = hardware_desc[handle].irq;
	
	port_res[handle].lin_mode = mode;
	s =  mode == LIN_MODE ? sizeof(uint16_t) : sizeof(uint8_t);
	
	port_res[handle].rx_q = xQueueCreate(rx_queue_size, s);
	port_res[handle].tx_q = xQueueCreate(tx_queue_size, sizeof(uint8_t));
	if (port_res[handle].rx_q == 0 || port_res[handle].tx_q == 0)
		return -1;
	
	//configure rx as input floating 
	gpio_init.GPIO_Pin = hardware_desc[handle].rx_pin;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(hardware_desc[handle].gpio, &gpio_init );
		
	//configure tx as alternate function push-pull */
	gpio_init.GPIO_Pin = hardware_desc[handle].tx_pin;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(hardware_desc[handle].gpio, &gpio_init);
	
	if (hardware_desc[handle].remap_bit != 0)
		GPIO_PinRemapConfig(hardware_desc[handle].remap_bit, ENABLE);
	
	usart_init.USART_BaudRate = baudrate;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(port, &usart_init);
	
	if (mode == LIN_MODE) {
		USART_LINCmd(port, ENABLE);
		USART_ITConfig(port, USART_IT_LBD, ENABLE);
	}	
	
	USART_ITConfig(port, USART_IT_RXNE, ENABLE);
	
	nvic_init.NVIC_IRQChannel = hardware_desc[handle].irq;
	nvic_init.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
	
	USART_Cmd(port, ENABLE);	

	return handle;
}

void clear_serial_buffer(int handle)
{
	if (handle < 0 || handle >= PORT_NUM)
		return;
	
	xQueueReset(port_res[handle].rx_q);
	xQueueReset(port_res[handle].tx_q);
}	

int write_serial(int handle, uint8_t *buf, int size,  TickType_t timeout)  
{     
	int i;
	BaseType_t ret;
	QueueHandle_t q = port_res[handle].tx_q;
	
	if (handle < 0 || handle >= PORT_NUM)
		return 0;
	
	for (i = 0; i < size; i++) {
		ret = xQueueSend(q, buf+i, timeout);
		if (ret == pdPASS) 
			USART_ITConfig(port_res[handle].port, USART_IT_TXE, ENABLE);
		else 
			break;
	}

	return i;
}

int read_serial(int handle, void *buf, int size,  TickType_t timeout)  
{                 
	int i;
	BaseType_t ret;
	QueueHandle_t q = port_res[handle].rx_q;
	uint32_t mode = port_res[handle].lin_mode;
	
	if (handle < 0 || handle >= PORT_NUM)
		return 0;
	
	for (i = 0; i < size; i++) {
		if (mode == LIN_MODE)
			ret = xQueueReceive(q, (uint8_t *)buf+i*2, timeout);
		else
			ret = xQueueReceive(q, (uint8_t *)buf+i, timeout);
		
		if (ret == pdFALSE)
			break;
	}

	return i;
} 

__irq void serial_interrupt_handler()
{
	portBASE_TYPE task_woken = pdFALSE;
	uint8_t data;
	uint16_t data_l;
	uint32_t ipsr, find;
	int i;
	struct serial_port_resources *res;
	
	ipsr = __get_IPSR();
	ipsr &= 0xff;
	ipsr -= 16;
	
	find = 0;
	for (i = 0; i < PORT_NUM; i++) {
		if (port_res[i].irq == ipsr) { 
			res = port_res+i;
			find = 1;
		}	
	}	
	
	if (!find)
		goto __exit;
	
	if (USART_GetITStatus(res->port, USART_IT_LBD) == SET) {
		USART_ClearITPendingBit(res->port, USART_IT_LBD);
		if (res->lin_mode == LIN_MODE) {
			data_l = LIN_BREAK_CHAR;
			xQueueSendFromISR(res->rx_q, &data_l, &task_woken);
		}	
	}	
	
	if (USART_GetITStatus(res->port, USART_IT_TXE) == SET) {
		if (xQueueReceiveFromISR(res->tx_q, &data, &task_woken) == pdTRUE) 		
			USART_SendData(res->port, data);
		else
			USART_ITConfig(res->port, USART_IT_TXE, DISABLE);		
	}
	
	if (USART_GetITStatus(res->port, USART_IT_RXNE) == SET) {
		if (USART_GetFlagStatus(res->port, USART_FLAG_FE) == SET)
			data = USART_ReceiveData(res->port);
		else {	
			data = USART_ReceiveData(res->port);
			
			if (res->lin_mode == LIN_MODE) {
				data_l = data;
				xQueueSendFromISR(res->rx_q, &data_l, &task_woken);
			}	else
				xQueueSendFromISR(res->rx_q, &data, &task_woken);
		}	
	}	

__exit:	
	portEND_SWITCHING_ISR(task_woken);
}	




