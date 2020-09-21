/**
* \file
*
* \brief Empty user application template
*
*/

/**
* \mainpage User Application template doxygen documentation
*
* \par Empty user application template
*
* Bare minimum empty user application template
*
* \par Content
*
* -# Include the ASF header files (through asf.h)
* -# "Insert system clock initialization code here" comment
* -# Minimal main function that starts with a call to board_init()
* -# "Insert application code here" comment
*
*/

/*
* Include header files for all drivers that have been imported from
* Atmel Software Framework (ASF).
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/
#include <asf.h>
#include <string.h>

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
#define DEBUG_SERIAL

#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

// pc30
// #define BUT1_PIO PIOC
// #define BUT1_PIO_id ID_PIOC
// #define play_mask 1 << 30

// #define next_pio PIOA
// #define next_pio_id ID_PIOA
// #define next_mask 1 << 19

#define c_EOF 'X'

#define BUT1_COMMAND_ID '1'
#define BUT1_PIO		   PIOD
#define BUT1_PIO_ID		   ID_PIOD
#define BUT1_PIO_IDX       28
#define BUT1_PIO_IDX_MASK  (1u << BUT1_PIO_IDX)

//BOTÃO PLAY/PAUSE
#define BUT2_COMMAND_ID '2'
#define BUT2_PIO		   PIOC
#define BUT2_PIO_ID		   ID_PIOC
#define BUT2_PIO_IDX       31
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)

#define BUT3_COMMAND_ID '3'
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

//Prioridade dos Botoes
#define BUT1_PRIOR 5 //Prioridade botao XOLED1
#define BUT2_PRIOR 5 //Prioridade botao XOLED2
#define BUT3_PRIOR 5 //Prioridade botao XOLED3

void BUT1_callback(void);
void BUT2_callback(void);
void BUT3_callback(void);
/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

volatile char BUT1_flag = 0;
volatile char BUT2_flag = 0;
volatile char BUT3_flag = 0;

void BUT1_callback(void){
	if(pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) == 0){
		BUT1_flag = '0';
		} else {
		BUT1_flag = '1';
	}
}

void BUT2_callback(void){
	if(pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK) == 0){
		BUT2_flag = '0';
		} else {
		BUT2_flag = '1';
	}
}

void BUT3_callback(void){
	if(pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK) == 0){
		BUT3_flag = '0';
		} else {
		BUT3_flag = '1';
	}
}


volatile long g_systimer = 0;

void SysTick_Handler() {
	g_systimer++;
}


void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEBlue", 1000);		//AT+NAMEnomedesejado
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN0000", 1000);			//AT+PINpindesejado
	usart_log("hc05_server_init", buffer_rx);
}

/***  Funcoes  ***/


void init(void){
	
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	pio_set_input(BUT1_PIO,BUT1_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT2_PIO,BUT2_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT3_PIO,BUT3_PIO_IDX_MASK,PIO_DEFAULT);

	pio_pull_up(BUT1_PIO,BUT1_PIO_IDX_MASK,1);
	pio_pull_up(BUT2_PIO,BUT2_PIO_IDX_MASK,1);
	pio_pull_up(BUT3_PIO,BUT3_PIO_IDX_MASK,1);
	
	//Set interruptores do projeto
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, BUT1_PRIOR); // Priority 2
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, BUT2_PRIOR); // Priority 2
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, BUT3_PRIOR); // Priority 2

	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);

	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, BUT1_callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, BUT2_callback);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, BUT3_callback);
	
}

void send_command(char id, char status){
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, id);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, status);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, c_EOF);
}


int main (void){
	
	init();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	char buffer[1024];
	BUT1_flag = 0;
	BUT2_flag = 0;
	BUT3_flag = 0;
	
	while(1) {
		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); // Espera ate que um interrupitor ligue
		
		if(BUT1_flag) {
			send_command(BUT1_COMMAND_ID, BUT1_flag);
			BUT1_flag = 0;
		}
		if(BUT2_flag) {
			send_command(BUT2_COMMAND_ID, BUT2_flag);
			BUT2_flag = 0;
		}
		if(BUT3_flag) {
			send_command(BUT3_COMMAND_ID, BUT3_flag);
			BUT3_flag = 0;
		}
	}
}