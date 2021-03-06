/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "spx.h"

#define RTC32_ToscBusy()        !( VBAT.STATUS & VBAT_XOSCRDY_bm )
void RTC32_ToscEnable( bool use1khz );


#define PRINTF_BUFFER_SIZE        128U

static uint8_t stdout_buff[PRINTF_BUFFER_SIZE];

SemaphoreHandle_t sem_stdout_buff;
StaticSemaphore_t STDOUT_xMutexBuffer;

//-----------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro

	// LEDS:
	IO_config_LED_KA();
	IO_config_LED_COMMS();


	// BLUETOOTH POWER CTL
	IO_config_BT_PWR_CTL();

	// TERMINAL CTL PIN
	IO_config_TERMCTL_PIN();

	// TICK:
	//IO_config_TICK();

	// OUTPUTS
	OUT_config();

	// GPRS
	pub_gprs_io_config();

	// IO_DIGITAL
	pub_digital_io_config();



}
//-----------------------------------------------------------
void u_configure_systemMainClock(void)
{
/*	Configura el clock principal del sistema
	Inicialmente se arranca en 2Mhz.
	La configuracion del reloj tiene 2 componentes: el clock y el oscilador.
	OSCILADOR:
	Primero debo elejir cual oscilador voy a usar para alimentar los prescalers que me den
	el clock del sistema y esperar a que este este estable.
	CLOCK:
	Elijo cual oscilador ( puedo tener mas de uno prendido ) va a ser la fuente principal
	del closck del sistema.
	Luego configuro el prescaler para derivar los clocks de los perifericos.
	Puedo por ultimo 'lockear' esta configuracion para que no se cambie por error.
	Los registros para configurar el clock son 'protegidos' por lo que los cambio
	utilizando la funcion CCPwrite.

	Para nuestra aplicacion vamos a usar un clock de 32Mhz.
	Como vamos a usar el ADC debemos prestar atencion al clock de perifericos clk_per ya que luego
	el ADC clock derivado del clk_per debe estar entre 100khz y 1.4Mhz ( AVR1300 ).

	Opcionalmente podriamos deshabilitar el oscilador de 2Mhz para ahorrar energia.
*/

#if SYSMAINCLK == 32
	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 32 Mhz ).
	//
#endif

#if SYSMAINCLK == 8
	// Habilito el oscilador de 32Mhz y lo divido por 4
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// Pongo el prescaler A por 4 y el B y C en 0.
	CLKSYS_Prescalers_Config( CLK_PSADIV_4_gc, CLK_PSBCDIV_1_1_gc );

	//
#endif

#if SYSMAINCLK == 2
	// Este es el oscilador por defecto por lo cual no tendria porque configurarlo.
	// Habilito el oscilador de 2Mhz
	OSC.CTRL |= OSC_RC2MEN_bm;
	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC2MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 2Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC2M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 2 Mhz ).
	//
#endif

//#ifdef configUSE_TICKLESS_IDLE
	// Para el modo TICKLESS
	// Configuro el RTC con el osc externo de 32Khz
	// Pongo como fuente el xtal externo de 32768 contando a 32Khz.
	//CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc | CLK_RTCEN_bm;
	//do {} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Disable RTC interrupt.
	// RTC.INTCTRL = 0x00;
	//
	// Si uso el RTC32, habilito el oscilador para 1ms.

	RTC32_ToscEnable(true);
//#endif

	// Lockeo la configuracion.
	CCPWrite( &CLK.LOCK, CLK_LOCK_bm );

}
//-----------------------------------------------------------
void RTC32_ToscEnable( bool use1khz )
{
	/* Enable 32 kHz XTAL oscillator, with 1 kHz or 1 Hz output. */
	if (use1khz)
		VBAT.CTRL |= ( VBAT_XOSCEN_bm | VBAT_XOSCSEL_bm );
	else
		VBAT.CTRL |= ( VBAT_XOSCEN_bm );

	RTC32.PER = 10;
	RTC32.CNT = 0;

	/* Wait for oscillator to stabilize before returning. */
//	do { } while ( RTC32_ToscBusy() );
}
//-----------------------------------------------------------
void u_configure_RTC32(void)
{
	// El RTC32 lo utilizo para desperarme en el modo tickless.
	// V-bat needs to be reset, and activated
	VBAT.CTRL |= VBAT_ACCEN_bm;
	// Este registro esta protegido de escritura con CCP.
	CCPWrite(&VBAT.CTRL, VBAT_RESET_bm);

	// Pongo el reloj en 1.024Khz.
	VBAT.CTRL |=  VBAT_XOSCSEL_bm | VBAT_XOSCFDEN_bm ;

	// wait for 200us see AVR1321 Application note page 8
	_delay_us(200);

	// Turn on 32.768kHz crystal oscillator
	VBAT.CTRL |= VBAT_XOSCEN_bm;

	// Wait for stable oscillator
	while(!(VBAT.STATUS & VBAT_XOSCRDY_bm));

	// Disable RTC32 module before setting counter values
	RTC32.CTRL = 0;

	// Wait for sync
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );

	// EL RTC corre a 1024 hz y quiero generar un tick de 10ms,
	RTC32.PER = 1024;
	RTC32.CNT = 0;

	// Interrupt: on Overflow
	RTC32.INTCTRL = RTC32_OVFINTLVL_LO_gc;

	// Enable RTC32 module
	RTC32.CTRL = RTC32_ENABLE_bm;

	/* Wait for sync */
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );
}
//-----------------------------------------------------------
int xprintf_P( int fd, PGM_P fmt, ...)
{
	// Imprime formateando en el uart fd.usando el buffer stdout_buff.
	// Como se controla con semaforo, nos permite ahorrar los buffers de c/tarea.
	// Si bien vsnprintf no es thread safe, al usarla aqui con el semaforo del buffer
	// queda thread safe !!!

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_stdout_buff, ( TickType_t ) 1 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	va_start(args, fmt);
	vsnprintf_P( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = sFRTOS_write(fd, (char *)stdout_buff,PRINTF_BUFFER_SIZE);

	xSemaphoreGive( sem_stdout_buff );
	return(i);

}
//------------------------------------------------------------------------------------
int cmd_xprintf_P( int fd, PGM_P fmt, ...)
{
	// Similar a xprintf_P pero imprime en los 2 descriptores pUSB y pBT

va_list args;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_stdout_buff, ( TickType_t ) 1 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	va_start(args, fmt);
	vsnprintf_P( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);

	sFRTOS_write(pUSB, (char *)stdout_buff,PRINTF_BUFFER_SIZE);
//	sFRTOS_write(pBT, (char *)stdout_buff,PRINTF_BUFFER_SIZE);

	xSemaphoreGive( sem_stdout_buff );
	return(1);

}
//------------------------------------------------------------------------------------
void xprintf_P_init(void)
{
	sem_stdout_buff = xSemaphoreCreateMutexStatic( &STDOUT_xMutexBuffer );
}
//------------------------------------------------------------------------------------
bool pub_save_params_in_NVMEE(void)
{
	// Calculo el checksum del systemVars.
	// Considero el systemVars como un array de chars.

char *p;
uint8_t checksum;
uint16_t data_length;
uint16_t i;

	// Calculo el checksum del systemVars.
	systemVars.checksum = 0;
	data_length = sizeof(systemVars);
//	p = (char*)&systemVars;
//	checksum = 0;
	// Recorro todo el systemVars considerando c/byte como un char, hasta
	// llegar al ultimo ( checksum ) que no lo incluyo !!!.
//	for ( i = 0; i < (data_length - 1) ; i++) {
//		checksum += p[i];
//	}
//	checksum = ~checksum;
//	systemVars.checksum = checksum;

	// Guardo systemVars en la EE
	NVMEE_write(0x00, (char *)&systemVars, sizeof(systemVars));

	//return(checksum);
	return(true);
}
//------------------------------------------------------------------------------------
void pub_configPwrSave(uint8_t modoPwrSave, char *s_startTime, char *s_endTime)
{
	// Recibe como parametros el modo ( 0,1) y punteros a string con las horas de inicio y fin del pwrsave
	// expresadas en minutos.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrSave.modo = modoPwrSave;

	if ( s_startTime != NULL ) { pub_convert_str_to_time_t( s_startTime, &systemVars.pwrSave.hora_start); }
	if ( s_endTime != NULL ) { pub_convert_str_to_time_t( s_endTime, &systemVars.pwrSave.hora_fin); }

	xSemaphoreGive( sem_SYSVars );

}
//------------------------------------------------------------------------------------
void pub_load_defaults(void)
{

	strncpy_P(systemVars.dlgId, PSTR("SPX000\0"),DLGID_LENGTH);
	systemVars.debug = DEBUG_NONE;

	// PwrSave
	systemVars.pwrSave.modo = modoPWRSAVE_ON;
	systemVars.pwrSave.hora_start.hour = 23;
	systemVars.pwrSave.hora_start.min = 30;
	systemVars.pwrSave.hora_fin.hour = 5;
	systemVars.pwrSave.hora_fin.min = 30;

	// Xbee
	systemVars.xbee = XBEE_OFF;

	pub_analog_load_defaults();
	pub_digital_load_defaults();
	pub_gprs_load_defaults();
//	pub_outputs_load_defaults();
//	pub_rangeMeter_load_defaults();

}
//------------------------------------------------------------------------------------
bool pub_load_params_from_NVMEE(void)
{
	// Leo el systemVars desde la EE.
	// Calculo el checksum. Si no coincide es que hubo algun
	// error por lo que cargo el default.

char *p;
uint8_t stored_checksum, checksum;
uint16_t data_length;
uint16_t i;

	// Leo de la EE es systemVars.
	NVMEE_read(0x00, &systemVars, sizeof(systemVars));
	return(true);

	// Guardo el checksum que lei.
	stored_checksum = systemVars.checksum;

	// Calculo el checksum del systemVars leido
	systemVars.checksum = 0;
	data_length = sizeof(systemVars);
	p = (char*)&systemVars;	// Recorro el systemVars como si fuese un array de int8.
	checksum = 0;
	for ( i = 0; i < ( data_length - 1 ); i++) {
		checksum += p[i];
	}
	checksum = ~checksum;

	if ( stored_checksum != checksum ) {
		return(false);
	}

	return(true);
}
//------------------------------------------------------------------------------------
void pub_convert_str_to_time_t ( char *time_str, time_t *time_struct )
{

	// Convierte un string hhmm en una estructura time_type que tiene
	// un campo hora y otro minuto

uint16_t time_num;

	time_num = atol(time_str);
	time_struct->hour = (uint8_t) (time_num / 100);
	time_struct->min = (uint8_t)(time_num % 100);

}
//----------------------------------------------------------------------------------------

