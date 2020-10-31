  /******************************************************************************
  * @file    Low_Power.h
  * @author  Sergio Vera Muñoz
  * @brief   Libreria de funciones de Bajo Consumo para proyecto de TFG sobre la
  * 		 placa de desarrollo de STM32 L4 -> B-L475E-IOT01A
  ******************************************************************************
  * @attention
  *
  *  Copyright (c) 2020 Sergio Vera - TFG: "Sensor IoT para integración de
  *  generacion fotovoltáica en vehículos eléltricos". ETSIDI - UPM
  * All rights reserved
  *
  * THIS SOFTWARE IS PROVIDED BY SERGIOVERAELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW.
  ******************************************************************************
  */

#ifndef APPLICATION_USER_LOW_POWER_H_
#define APPLICATION_USER_LOW_POWER_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines Privados ------------------------------------------------------------*/

#define PERIODO_MIN_LPTIM1 		 1		//Periodo minimo de disparo del LPTIM1 para tomar muestras. Debe ser divisor de PERIODO_LECTURA_DATOS
#define PERIODO_MIN_LPTIM2  	10		//Periodo minimo de disparo del LPTIM2 para publicar/recuperar conexion. Debe ser divisor de PERIODO_PUBLI y PERIODO_RECUPERA_DATOS

#define FREC_LOWPOW    RCC_MSIRANGE_2  //frecuencia de trabajo en el modo de bajo consumo -> 400 kHz

#define PERIODO_LPTIM 	((32768/128)-1)			//Periodo de cuenta del  LPTIM1, equivalente a 32.768kHz/128 = 256-1

#define TIMEOUT_LPTIM1 	(((PERIODO_LPTIM)*PERIODO_MIN_LPTIM1))	//Tiempo maximo de disparo del LPTIM1, exuivalente a 1 segundo
#define TIMEOUT_LPTIM2 	(((PERIODO_LPTIM)*PERIODO_MIN_LPTIM2))	//Tiempo maximo de disparo del LPTIM2, exuivalente a 10 segundos

#define TICKS_RTC_PER_SEC 	((32768/16)-1)	//numero de Ticks en un segundo segun la configuracion del RTC
								// el reloj que rige el RTC es de 32.768kHz/16 = 2048 ticks/seg

#define LONGITUD_STD  		-3.72694444f  //Longitud de referencia constante (formato decimal), tomada como la del IES (Madrid)
#define LATITUD_STD   		40.45333333f	//Idem, solo para la primera comprobación, estándar

/* Declaraicion de variables -----------------------------------------------*/

enum {SUENYO=0, VIGILIA};	// variable booleana para get_PeriodoDia return

extern char buffc_DMA_UART[TAM_BUFNMEA] ;	//el buffer del main para tomar datos
extern bool  modo_BajoConsumo, ocioso;

/* Prototipos privados de funciones -----------------------------------------------*/

float hour_2_decimal(uint8_t hora, uint8_t min, uint8_t seg) ;

bool get_PeriodoDia(float* Hora_Amanecer_Oficial, float* Hora_Atardecer_Oficial);
void get_AmanecerAtardecer(float* Hora_Amanecer_Oficial, float* Hora_Atardecer_Oficial, float latitud, float longitud);

void entraSleepMode(float Hora_Amanecer_Oficial);

void entrar_LowPowerMode(void);
void salir_LowPowerMode(void);

void desinicializarPerifericos(void);


/* Funciones externas de otros ficheros-----------------------------------------------------------*/

extern void SystemClock_Config(void);	//reinicializacion de los relojes del sistema.

extern bool decodificadorNMEA(char bufferNMEA[TAM_BUFNMEA], float* latit, float* longit, float* altit, float* speed);
extern bool desconectaConexionMQTT(void);

extern RTC_TimeTypeDef sTiempo_actual;
extern RTC_DateTypeDef sDia_actual;

/* Declaraciones de dichas funciones -----------------------------------------------*/

float hour_2_decimal(uint8_t hora, uint8_t min, uint8_t seg)  {

	return (float) hora + min/60.0f + seg/3600.0f ;

}

/*
  * @brief Función de actualización de las horas de amanecer y anochecer en la localización actual del sensor.
  *  Se realizan los cálculos de la hora solar a partir de la hora civil del RTC y a partir de la localización
  *  geografica actual. Esta informacion se emplea después por la función get_PeriodoDia()
  * @param Hora_Amanecer_Oficial: dirrección de memoria actualizar
  * @param Hora_Atardecer_Oficial: dirrección de memoria actualizar
  * @param latitud: componente de ubicación geografica actual, como parametro de entrada
  * @param longitud: componente de ubicación geografica actual, como parametro de entrada
  * @retval void
  */
void get_AmanecerAtardecer(float* Hora_Amanecer_Oficial, float* Hora_Atardecer_Oficial, float latitud, float longitud)  {

	uint8_t dia_agno = 0;
	float declinacion = 0.0f, EoT = 0.0f, Hora_Amanecer_Solar = 0.0f, dia_angular = 0.0f;
	float correccion_solar = 0.0f, mediodia_solar = 0.0f;

	 if (HAL_RTC_GetDate(&hrtc, &sDia_actual, RTC_FORMAT_BIN) != HAL_OK ) {
		    	printf("Error al dar las obterner fecha actual del RTC para calcular sleep mode.\n");
		 }
	 else {
		switch( sDia_actual.Month) {

		case 1: dia_agno=  0 + sDia_actual.Date;
		break;
		case 2: dia_agno= 31 + sDia_actual.Date;	//si es año bisiesto no pasa nada, se pierde 1 dia
		break;
		case 3: dia_agno=  59 + sDia_actual.Date;
		break;
		case 4: dia_agno=  90 + sDia_actual.Date;
		break;
		case 5: dia_agno= 120 + sDia_actual.Date;
		break;
		case 6: dia_agno= 151 + sDia_actual.Date;
		break;
		case 7: dia_agno=  181 + sDia_actual.Date;
		break;
		case 8: dia_agno= 212 + sDia_actual.Date;
		break;
		case 9: dia_agno=  243 + sDia_actual.Date;
		break;
		case 10: dia_agno=  273 + sDia_actual.Date;
		break;
		case 11: dia_agno=  304 + sDia_actual.Date;
		break;
		case 12: dia_agno=  334 + sDia_actual.Date;
		break;

		default: dia_agno=365/2; //por si acaso, en mitad del año
		}

	 }

	 declinacion = deg2rad(23.45f) * sinf( (2*M_PI*(dia_agno+284)) /365.0f );

	 Hora_Amanecer_Solar = -acosf(-tanf(declinacion) * tanf(deg2rad(latitud)) );	//hora solar en radianes
	 Hora_Amanecer_Solar = Hora_Amanecer_Solar / (M_PI/12.0f); // hora solar en horas, factor conversion [rd/h]

	dia_angular = (2*M_PI*dia_agno)/365.24f;	//en radianes
	EoT = 229.18 * (-0.0334 * sinf(dia_angular) + 0.04184 * sinf(2*dia_angular + 3.5884)); //en minutos
	EoT = EoT/60.0f; //a horas

	correccion_solar = -EoT - ( deg2rad( (longitud) ) / (M_PI/12.0f) );	//en horas, por la Equation of Time y por el desfae en longitud
	mediodia_solar = 12.0f + correccion_solar;
				/*Para la franja horaria que sea, referida a GMT, que es la del RTC*/

	*Hora_Amanecer_Oficial = mediodia_solar + Hora_Amanecer_Solar;
	*Hora_Atardecer_Oficial = mediodia_solar - Hora_Amanecer_Solar + 0.50;	//30 min más
								/*se añade arbitrariamente un poco más de tiempo (30min) para tomar medidas
									a última hora de la tarde, ya que puede resultar interesante*/
}

/*
  * @brief Función de comprobación del periodo del día en el que se encuentra el sensor. A partir de
  * la hora civil actual del RTC y las horas de amanecer y anochecer actuales, se divide el día entre
  *  la noche y el día, de cara a enviar al sensor al modo de shutdown durante la noche.
  * @param Hora_Amanecer_Oficial: parametro de entrada
  * @param Hora_Atardecer_Oficial: parametro de entrada
  * @retval void
  */
bool get_PeriodoDia(float* Hora_Amanecer_Oficial, float* Hora_Atardecer_Oficial) {

	static uint64_t ant_millis = 0;
	float hora_actual = 0.0f;

	if( HAL_GetTick() - ant_millis >= 1000*3600*1)  {	//cada hora actualiza el amanecer-atardecer donde se encuentre

		ant_millis = HAL_GetTick();

		float latit_act = 0.0f, longit_act = 0.0f;

		if ( decodificadorNMEA(buffc_DMA_UART, &latit_act, &longit_act, NULL, NULL) )  {
			get_AmanecerAtardecer(Hora_Amanecer_Oficial, Hora_Atardecer_Oficial, latit_act, longit_act);
		}
	}

	 if (HAL_RTC_GetTime(&hrtc, &sTiempo_actual, RTC_FORMAT_BIN) != HAL_OK  ) {
		    	printf("Error al dar las obterner hora actual del RTC para calcular sleep mode.\n");
		    	return VIGILIA;
		 }
	 else {
		 hora_actual = hour_2_decimal( sTiempo_actual.Hours , sTiempo_actual.Minutes, sTiempo_actual.Seconds );

		 if (hora_actual>=*Hora_Amanecer_Oficial && hora_actual<=*Hora_Atardecer_Oficial)  {
			 return VIGILIA;
		 }
		 else { return SUENYO;  }
	 }
}

/*
  * @brief Función de entrada al modo ShutDown. Se calculan las horas restantes de noche para programar
  * una interrupción del RTC que despierte al MCU. El resto de periféricos son desinicializados y apagados.
  * Tras el despertar, el programa se reincia.
  * @param Hora_Amanecer_Oficial: parametro de entrada
  * @retval void
  */
void entraSleepMode(float Hora_Amanecer_Oficial)  {

	float tiempo_sleep = 0.0f, hora_actual = 0.0f;

	 if (HAL_RTC_GetTime(&hrtc, &sTiempo_actual, RTC_FORMAT_BIN) != HAL_OK  ) {
		    	printf("Error al dar las obterner hora actual del RTC para calcular sleep mode.\n");
		    	return;
		 }

	 hora_actual = hour_2_decimal( sTiempo_actual.Hours , sTiempo_actual.Minutes, sTiempo_actual.Seconds );

	 if(hora_actual>12.0f)	{	//si es mas de medio dia
		 tiempo_sleep = ((24.0f-hora_actual) + Hora_Amanecer_Oficial ) * 3600.0f;	//numero de segundos que duerme
		 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	//hasta el amanecer
	 }
	 else {
		 tiempo_sleep = (Hora_Amanecer_Oficial - hora_actual)* 3600.0f  ;
	 }

	printf("El dispositivo saldra del Modo Sleep dentro de %02d horas y %02d minutos... \n",
			(int) truncf(tiempo_sleep/3600),(int) roundf(((tiempo_sleep/3600) - (truncf(tiempo_sleep/3600)) )*60));


	// Configurar el RTC para despertarse al cabo de tantos ticks de su reloj/16
	uint32_t contador_tiempo =  TICKS_RTC_PER_SEC * tiempo_sleep;	//pasar de segundos a ticks
	//activar la interrupcion para salir del modo de SHUTDOWN
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, contador_tiempo, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	 HAL_GPIO_WritePin(ARD_A0_3_3CTRL_GPIO_Port, ARD_A0_3_3CTRL_Pin, GPIO_PIN_RESET);
	  	  	  	  	  	  	  /*apagar alimentacion sensores a 3.3V del BMS */
	 HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin | ARD_A2_LEDON_Pin, GPIO_PIN_RESET);	//LED de conexión Wi-Fi & ON

	 desinicializarPerifericos();

	  HAL_PWREx_EnterSHUTDOWNMode();	//el modo mas potente de desactivacion, el sueño mas profundo
/*
 * ***********************************************************************************************************************
 * ********************************************* MODO DORMIDO SHUTDOWN ***********************************************************
 * ***********************************************************************************************************************
 */
	  HAL_NVIC_SystemReset();   /*Una vez el dispositivo despiesta de su letargo, se reinicia*/
}

/*
  * @brief Función de desinicialización y apagado de todos los periféricos salvo el RTC. Permite un mayor
  * ahorro de energía durante el modo shutdown.
  * @param void
  * @retval void
  */
void desinicializarPerifericos(void)  {

	extern  SPI_HandleTypeDef hspi;
	extern  ADC_HandleTypeDef hadc1;
	extern  DMA_HandleTypeDef hdma_adc1;
	extern  OPAMP_HandleTypeDef hopamp2;
	extern  RNG_HandleTypeDef hrng;
	extern  CRC_HandleTypeDef hcrc;
	extern  TIM_HandleTypeDef htim3;
	extern  TIM_HandleTypeDef htim6;
	extern  LPTIM_HandleTypeDef hlptim1;
	extern  LPTIM_HandleTypeDef hlptim2;
	extern  UART_HandleTypeDef huart4;
	extern  UART_HandleTypeDef huart1;
	extern  DMA_HandleTypeDef hdma_uart4_rx;

	HAL_FLASHEx_EnableRunPowerDown();
	desconectaConexionMQTT();
	WIFI_Disconnect();
	//WIFI_ResetModule();   //provoca errores

	  HAL_ADC_Stop_IT(&hadc1);
	  HAL_ADC_Stop_DMA(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	  HAL_ADC_DeInit(&hadc1);

	  HAL_UART_Abort(&huart1);
	  HAL_UART_DMAStop(&huart1);
	  HAL_UART_AbortReceive_IT(&huart1);
	  HAL_UART_DeInit(&huart1);
	  HAL_UART_Abort(&huart4);
	  HAL_UART_DMAStop(&huart4);
	  HAL_UART_AbortReceive_IT(&huart4);
	  HAL_UART_DeInit(&huart4);

	  HAL_DMA_Abort(&hdma_adc1);
	  HAL_DMA_Abort_IT(&hdma_adc1);
	  HAL_DMA_DeInit(&hdma_adc1);
	  HAL_DMA_Abort(&hdma_uart4_rx);
	  HAL_DMA_Abort_IT(&hdma_uart4_rx);
	  HAL_DMA_DeInit(&hdma_uart4_rx);

	  HAL_OPAMP_Stop(&hopamp2);
	  HAL_OPAMP_DeInit(&hopamp2);

	  HAL_SPI_AbortCpltCallback(&hspi);
	  HAL_SPI_Abort_IT(&hspi);
	  HAL_SPI_Abort(&hspi);
	  HAL_SPI_DMAStop(&hspi);
	  HAL_SPI_DeInit(&hspi);

	  HAL_TIM_Base_Stop(&htim3);
	  HAL_TIM_Base_Stop_IT(&htim3);
	  HAL_TIM_Base_DeInit(&htim3);
	  HAL_TIM_Base_Stop(&htim6);
	  HAL_TIM_Base_Stop_IT(&htim6);
	  HAL_TIM_Base_DeInit(&htim6);
	  HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
	  HAL_LPTIM_DeInit(&hlptim1);
	  HAL_LPTIM_TimeOut_Stop_IT(&hlptim2);
	  HAL_LPTIM_DeInit(&hlptim2);

	  HAL_RNG_DeInit(&hrng);
	  HAL_CRC_DeInit(&hcrc);

	  HAL_GPIO_WritePin(GPIOE, RED_LED_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, ARD_A2_LEDON_Pin|ARD_A1_LEDWIFI_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(ARD_D10_MFT_GPIO_Port, ARD_D10_MFT_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin|ARD_D7_Pin|ARD_D11_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(ARD_A0_3_3CTRL_GPIO_Port, ARD_A0_3_3CTRL_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin|ARD_D8_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_DeInit(GPIOE, RED_LED_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin);
	  HAL_GPIO_DeInit(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin);
	  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
	  HAL_GPIO_DeInit(GPIOC, ARD_A2_LEDON_Pin|ARD_A1_LEDWIFI_Pin|LED3_WIFI__LED4_BLE_Pin);
	  HAL_GPIO_DeInit(ARD_D10_MFT_GPIO_Port, ARD_D10_MFT_Pin);
	  HAL_GPIO_DeInit(GPIOA, ARD_D4_Pin|ARD_D7_Pin|ARD_D11_Pin);
	  HAL_GPIO_DeInit(ARD_A0_3_3CTRL_GPIO_Port, ARD_A0_3_3CTRL_Pin);
	  HAL_GPIO_DeInit(GPIOB, ARD_D6_Pin|ARD_D8_Pin|ARD_D5_Pin);
	  HAL_GPIO_DeInit(GPIOE, DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin);
	  HAL_GPIO_DeInit(GPIOE, QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin|QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin);
	  HAL_GPIO_DeInit(GPIOB, ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin);
	  HAL_GPIO_DeInit(GPIOD, INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin);
	  HAL_GPIO_DeInit(GPIOD, LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin|PMOD_IRQ_EXTI12_Pin);
	  HAL_GPIO_DeInit(LSM3MDL_DRDY_EXTI8_GPIO_Port, LSM3MDL_DRDY_EXTI8_Pin);
	  HAL_GPIO_DeInit(PMOD_SPI2_SCK_GPIO_Port, PMOD_SPI2_SCK_Pin);
	  HAL_GPIO_DeInit(GPIOD, PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin);
	  HAL_GPIO_DeInit(GPIOB, ARD_D15_Pin|ARD_D14_Pin);
	  HAL_GPIO_DeInit(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin);

	  HAL_SuspendTick();

	  HAL_RCCEx_DisableLSCO();
	  HAL_RCCEx_DisableMSIPLLMode();
	  HAL_RCCEx_DisablePLLSAI1();
	  HAL_RCCEx_DisablePLLSAI2();
	  HAL_RCC_DeInit();
	  __HAL_RCC_PWR_CLK_DISABLE();

}


/*
  * @brief Función de entrada al modo de operación de bajo consumo. Los relojes del sistema con reconfigurados para
  * ahorrar energía, bajando su frecuencia hasta 400KHz. Una vez realizado, se habilita dicho modo.
  * @param void
  * @retval void
  */
void entrar_LowPowerMode(void)  {

	printf("\tEntrando en el modo de Bajo Consumo...\n\n");
	/* Previo a la entrada del modo, hay que bajar la frecuencia de reloj a menos de 2 MHz  */

	/* Disminución de la velocidad del Reloj del Sistema. La fuente del reloj del sistema es elevada
	 * desde el HSI al MSI, mientras que al mismo tiempo, el rango del MSI se establece en una frecuencia
	 * menor, concretamente a 400 KHz */

	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};

	 /* MSI es habilitado*/
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	  RCC_OscInitStruct.MSIClockRange = FREC_LOWPOW;  //frecuencia del MSI bajada a 400kHz
	  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		  printf("Error al configurar el reloj a baja frecuencia.\n");
		  Error_Handler();
	  }

	  /* Se selecciona el MSI como la fuente del reloj del sistema y se configura los dividores de
	   * frecuencia HCLK, PCLK1 and PCLK2 */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
		  printf("Error al configurar el reloj a baja frecuencia.\n");
		  Error_Handler();
	  }

	HAL_GPIO_WritePin(GPIOC, ARD_A2_LEDON_Pin, GPIO_PIN_RESET); //indicador visual
	HAL_SuspendTick();
	/*Ahora si, se entra en el modo de bajo consumo */

	HAL_PWREx_EnableLowPowerRunMode();

}


/*
  * @brief Función de salida del modo de operación de bajo consumo. Los relojes del sistema con reconfigurados
  * mediante su configuración inicial, a una mayor frecuencia. Informa al usuario por pantalla.
  * @param void
  * @retval void
  */
void salir_LowPowerMode(void) {

	if( HAL_PWREx_DisableLowPowerRunMode() != HAL_OK )  {	//comprobación visual
		Error_Handler();
		}

	SystemClock_Config();	//reinicializar el reloj habitual
	HAL_ResumeTick();

	HAL_GPIO_WritePin(GPIOC, ARD_A2_LEDON_Pin, GPIO_PIN_SET); //indicador visual
#ifdef ENABLE_LOWPWR
	modo_BajoConsumo=false;
	ocioso = false;
#endif
	printf("\tSaliendo del modo de Bajo Consumo... \n");
}


#endif /* APPLICATION_USER_LOW_POWER_H_ */


/************************ (C) COPYRIGHT Sergio Vera Muñoz --- TFG 2020   --- *****END OF FILE****/
