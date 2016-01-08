#ifndef __SERIAL
#define __SERIAL


#define UART_MODE				0
#define LIN_MODE				1
#define LIN_BREAK_CHAR	0x100


int init_serial_port(USART_TypeDef *port, uint32_t baudrate, 
										uint32_t rx_queue_size, uint32_t tx_queue_size, uint32_t mode);
int write_serial(int handle, uint8_t *buf, int size,  TickType_t timeout);
int read_serial(int handle, void *buf, int size,  TickType_t timeout);
void clear_serial_buffer(int handle);


#endif

