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
#include <stdlib.h>
#include <stdio.h>

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
#define DEBUG_SERIAL

#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

// pc30
// #define BUT1_PIO PIOC
// #define BUT1_PIO_id ID_PIOC
// #define play_mask 1 << 30

// #define next_pio PIOA
// #define next_pio_id ID_PIOA
// #define next_mask 1 << 19

#define c_EOF 'X'

#define VERIFICA_COMMAND_ID 'S'

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

// LED1
#define LED1_PIO      PIOA
#define LED1_PIO_ID   ID_PIOA
#define LED1_IDX      0
#define LED1_IDX_MASK (1 << LED1_IDX)

// LED2
#define LED2_PIO      PIOC
#define LED2_PIO_ID   ID_PIOC
#define LED2_IDX      30
#define LED2_IDX_MASK (1 << LED2_IDX)

// LED3
#define LED3_PIO      PIOB
#define LED3_PIO_ID   ID_PIOB
#define LED3_IDX      2
#define LED3_IDX_MASK (1 << LED3_IDX)

//Prioridade dos Botoes
#define BUT1_PRIOR 5 //Prioridade botao XOLED1
#define BUT2_PRIOR 5 //Prioridade botao XOLED2
#define BUT3_PRIOR 5 //Prioridade botao XOLED3

void BUT1_callback(void);
void BUT2_callback(void);
void BUT3_callback(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;

volatile char BUT1_flag = 0;
volatile char BUT2_flag = 0;
volatile char BUT3_flag = 0;
volatile char flag_tc = 0;

/**
* \brief AFEC interrupt callback function.
*/
static void AFEC_pot_Callback(void){
	g_ul_value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	g_is_conversion_done = true;
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback){
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);
	
	/* configura IRQ */
	afec_set_callback(afec, afec_channel,	callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}

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

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc = 1;
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

int response(){
	uint32_t resp = 0;
	if (!usart_read(UART_COMM, &resp));
	{
		if (resp=='X')
		{
			return 1;
		}
	}
	return 0;
}

void vol_func(uint32_t g_ul_value,char *vol_char){
	// 0 .. 9
	if (g_ul_value<=370){
		*vol_char = '0';
	}
	else if(g_ul_value<=740){
		*vol_char = '1';
	}
	else if(g_ul_value<=1110){
		*vol_char = '2';
	}
	else if(g_ul_value<=1480){
		*vol_char = '3';
	}
	else if(g_ul_value<=1850){
		*vol_char = '4';
	}
	else if(g_ul_value<=2220){
		*vol_char = '5';
	}
	else if(g_ul_value<=2590){
		*vol_char = '6';
	}
	else if(g_ul_value<=2960){
		*vol_char = '7';
	}
	else if(g_ul_value<=3330){
		*vol_char = '8';
	}
	else{
		*vol_char = '9';
	}
}

/**
* @Brief Inicializa o pino do LED
*/
void LED1_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, estado, 0, 0 );
};

void init(void){
	
	LED1_init(1);
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
	
	TC_init(TC0, ID_TC1, 1, 2);
	
	/* inicializa e configura adc */
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	
	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
	
}
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
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
	char vol_char = '0';
	char vol_char_old = '0';

	BUT1_flag = 0;
	BUT2_flag = 0;
	BUT3_flag = 0;
	
	uint32_t resposta = 0;
	int mandou = 0;
	
	
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); // Espera ate que um interrupitor ligue
		
		if (flag_tc)
		{
			send_command(VERIFICA_COMMAND_ID, '0');
			delay_ms(100);
			if (response())
			{
				pio_clear(LED1_PIO,LED1_IDX_MASK);
				}else{
				pio_set(LED1_PIO,LED1_IDX_MASK);
			}
		}
		
		if (BUT1_flag) {
			send_command(BUT1_COMMAND_ID, BUT1_flag);
			BUT1_flag = 0;
		}
		if (BUT2_flag) {
			send_command(BUT2_COMMAND_ID, BUT2_flag);
			BUT2_flag = 0;
		}
		if (BUT3_flag) {
			send_command(BUT3_COMMAND_ID, BUT3_flag);
			BUT3_flag = 0;
		}
		if (flag_tc) {
			if (g_is_conversion_done) {
				vol_func(g_ul_value, &vol_char);
				
				// garante que volume só e' enviado quando for um novo valor
				if (vol_char != vol_char_old) {
					send_command('v', vol_char);
				}
				vol_char_old = vol_char;
				
				/* Selecina canal e inicializa conversão */
				afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
				afec_start_software_conversion(AFEC_POT);
			}
			flag_tc = 0;
		}
	}
}