/**
  ******************************************************************************
  * @file    AppIoT_TFG_VIPV.h
  * @author  Sergio Vera Muñoz
  * @brief   Fichero con la Aplicación principal del Sensor IoT para VIPV. Realiza
  * 		 la conexion a internet, recaba datos y los publica.
  ******************************************************************************
  * @attention
  *
  *  Copyright (c) 2020 Sergio Vera - TFG: "Sensor IoT para integración de
  *  generacion fotovoltáica en vehículos eléltricos". ETSIDI - UPM
  * All rights reserved
  *
  * THIS SOFTWARE IS PROVIDED BY SERGIOVERAELECTRONICS & STMICROELECTRONICS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING,
  *  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW.
  ******************************************************************************
  */
#ifndef __AppIOTGenericaMQTT_H
#define __AppIOTGenericaMQTT_H


/* Private macro -------------------------------------------------------------*/

#define ENABLE_SLEEP
				/*Compila el código habilitando la funcionalidad de entrar en modo Sleep. Comentar para deshabilitar */
#define ENABLE_IMPRIMIR_MUESTRAS
				/*Compila el código habilitando la funcionalidad de Imprimir los datos muestreados. Comentar para deshabilitar */
//#define ENABLE_LOWPWR
				/*Compila el código habilitando la funcionalidad de Bajo Consumo. Comentar para deshabilitar */
//#define ENABLE_PULSADOR
				/*Compila el código habilitando la funcionalidad de accionar el pulsador para detener publicacion. Comentar para deshabilitar */
#define PUBLI_DATOS_THINGSPEAK
				/*Compila el código habilitando la funcionalidad de publicar los datos recabados. Comentar para deshabilitar */


/*  Defines específicos del proyecto del TFG----------------------------------*/

#define PERIODO_PUBLI_DATOS      60 	//Periodo de publicacion de datos a la nube, en segundos, ha de ser entero
#define PERIODO_LECTURA_DATOS     1		//Periodo de lectura de los datos
#define PERIODO_RECUPERA_DATOS   20 	/*periodo minimo de ThingSpeak para recuperar los datos es de 15 seg
										 https://thingspeak.com/pages/license_faq   */


#define T_MEDICION		  3     //Tiempo en ms durante el cual permanece midiendo un módulo FV
#define T_ESPERA		  5	   //Tiempo que espera entre permutaciones de los BJT para tomar las medidas, por si acaso, grande, no hay prisa
#define NMAX_MODULOS 	  5	   // numero de modulos fotovoltaicos del sensor

#define FACTOR_OPAMP 	  4.0f			//Ganancia del amplificador opereacional para mejorar rango dinamico
#define SENS_HALL  		0.7984f		/*Sensibilidad corriente tensión del sensor Hall,
									a tener también en cuenta las caidas de tensión en los interruptores BJT, etc*/

#define VREF_ADC 	    3300.0f    //Tension de referencia del ADC en mV
#define NIVELES_ADC     4095.0f 	//Nº de niveles de los 12b del ADC

#define NMAX_VUELTAS_OCIO  	3		//Nº maximo de vueltas al bucle de publicacion en estado ocioso para meterse al modo de bajo consumo

#define NLECTURAS_MAGNETO  100 	//numero de muestras tomadas para leer valor medio del magnetometro
#define TOL_MAGNETO  		5	//tolerancia de desviación estándar absoluta de medida del magnetómetro

#define SUPER_O    			167	   //para imprimir el caracter 'º', en ASCII


/**********************************************************************************************************/
/**************   MACROS PARA ABREVIAR TAREAS     *********************************************************/
#define noesNAN(x)    ( !( (x)!=(x) ) )  //MACRO para idefntificar a los NaN
#define deg2rad(x)    ( (float)(x) * (M_PI/180.0f))
#define rad2deg(x)    ( (float)(x) * (180.0f/M_PI))
#define N_ELEMENTOS  (PERIODO_PUBLI_DATOS / PERIODO_LECTURA_DATOS)

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "FIFO.h"	//contiene las funciones y estructuras para crear una lista enlazada comportamiento fifo
#include "Low_Power.h"
#include "GenericMQTT.h"

#endif /* __AppIOTGenericaMQTT_H */


/* Private typedef -----------------------------------------------------------*/

//Valores de Corrientes de Calibración G/Isc de los diferentes módulos FV caracterizados, en unidades de [(W/m^2)/mA]

const float CTE_CALIBR_FV[NMAX_MODULOS] = { 3.814272392f, 3.804324917f, 3.702794995f, 3.797673078f, 3.666993446f };


enum {DESCONECTADO=0, CONECTADO};	//Enumeracion simple para ver estado conexión wifi

extern bool iniciado_Programa;		//Variable para comrpobar el punto del programa en el que el haya

extern uint32_t ADC1_buffer;		//En el main
extern LPTIM_HandleTypeDef hlptim1, hlptim2;

//Más adelante, servirá para establecer un nuevo timeout en la función reconecta_Wifi()

/* Prototipos de funciones propias -----------------------------------------------*/

void aplicacion_ClienteMQTT_XCLD_IoT(void);

void mideRadiacion(float vectIrradiancia[]);
float toma_datoADC(uint16_t t_espera);
float orienta_Magneto(void) ;
void recabar_Datos(megaDato* miLectura); //función de recogida de datos
bool publica_DatosThingSpeak(megaDato* miDato);
void calcula_mediaVector(megaDato* mediaDatos, megaDato* p_ectorLecturas, uint8_t n_elem );
void imprimir_Dato(megaDato Dato);

void bucle_Lectura_Publicacion(void);
bool reconecta_WiFi(void);

int check_protocoloConexion(void);
bool inicia_ClienteMQTT(int ret);
bool desconectaConexionMQTT(void);
bool inicializa_ConexionIoT(void);

/* Funcíón externa de la biblioteca NMEA --------------------------------------------------------*/

extern bool decodificadorNMEA(char bufferNMEA[TAM_BUFNMEA], float* latit, float* longit, float* altit, float* speed);



/************************ (C) COPYRIGHT Sergio Vera Muñoz *****END OF FILE****/
