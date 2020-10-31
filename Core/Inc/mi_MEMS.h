/******************************************************************************
* @file    mi_MEMS.h
* @author  Sergio Vera Muñoz
* @brief   Librería de funciones de obtención de datos del MEMS LSM6DSL y del
* Magnetómetro LIS3MDL del SoC B-L475E-IOT01A. Contiene funciones de toma de datos,
* organización y llamada a funciones del paquete de liberería de "ST X-CUBE-MEMS1" -
* Motion-FX. Llevan a cabo calibración de los sensores y filtro de Kalman de sus señales
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

#ifndef INC_MI_MEMS_H_
#define INC_MI_MEMS_H_

#include "stdbool.h"
#include "motion_fx.h"
#include "motion_fx_cm0p.h"

/* Private defines -----------------------------------------------------------*/
#define MAGNETOMETRO_CALIBRADO    1 		//Define si el magnetometro se encuentra calibrado en Hard Iron a priori (0 ó 1)
#define MAG_HIOFFSET_X 			-187 	//Ensayo de calibracion previo, depende mucho del ambiente a medir
#define MAG_HIOFFSET_Y 			 233	// Comprobar paper de STM: "Getting started with MotionFX sensor fusion library in X-CUBE-MEMS1expansion"
#define MAG_HIOFFSET_Z			 91		//https://www.st.com/resource/en/user_manual/dm00394369-getting-started-with-motionfx-sensor-fusion-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf

#define ALGORITHM_FREQ  50.0f /* Frecuencia de ejecucción del algoritomo, necesita ser modificada desde el MX TIM6*/

#define ALGO_PERIOD  ((int)(1000.0f / ALGORITHM_FREQ)) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  ((float)(1.0f / ALGORITHM_FREQ)) //periodo de computacion de f. Kalman en [s]

#define FREC_CALIB_MAGNETO  40  //Frecuencia de acualizacion del codigo de calibracion del magentometro

#define ATIME_REF   (0.9f)	//Entre 0 y 1 para ponderar magnetometro
#define MTIME_REF	(0.667f)
#define FRTIME_REF	(1.5f)
	//Descripcion dada por las datasheets de ST:
	/*ATime, MTime, FrTime represent the weighting stability of sensors for prediction (trust factor), from 0 to 1.
	Default values are recommended.
	– ATime: lowering the value will increase the accelerometer weight and sensitivity towards external
	acceleration. If the application experiences low acceleration (<1g) most of time, it is recommended to
	increase the value.
	– MTime, FrTime: for 9X solution, the lower value will increase the weight of magnetometer reading. If
	the application experiences low magnetic interference (<50 μT) most of time, it is recommended to
	increase the value.*/
	/*Float ATime: 0.5 to 10; if it is low, accelerometer is trusted more in the update phase
	(see below) in static condition (no motion); it is convenient to set it to a high value if
	accelerometer is considered unreliable because there are large offset and gain errors.
	Float MTime: 0.5 to 10; if it is low, magnetometer is trusted more in the update phase
	(see below) in dynamic condition; it is convenient to set it to a high value if magnetometer
	is unreliable because of magnetic anomalies or imperfect hard/soft iron compensation. In
	the static condition, the magnetometer is not trusted.
	Float FrTime: 0.5 to 10; if it is low, accelerometer is trusted more in the update phase
	(see below) in dynamic condition (high-g motion); it is convenient to set it to high value to
	reduce the effect of high-g motion, or if the accelerometer is considered unreliable*/

#define AUTOMATIC_BIAS_CALCULATION      0 //falso, de entrada no
#define GBIAS_ACC_TH_SC_6X              (2.0f*0.000765f) //por defecto
#define GBIAS_GYRO_TH_SC_6X             (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X              (2.0f*0.001500f)
#define GBIAS_ACC_TH_SC_9X              (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X             (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9X              (2.0f*0.001500f)

#define ACC_ODR  ((float)ALGORITHM_FREQ)
#define ACC_FS  4 /* FS = <-4g, 4g> */
#define DWT_LAR_KEY  0xC5ACCE55 /* DWT register unlock key */

#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f



typedef struct	//estructura para tipos de datos espaciales
{
  int32_t x;
  int32_t y;
  int32_t z;
} MOTION_SENSOR_Axes_t;

/* ------------------------------------------------- Variables ---------------------------------------------------------*/
static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t MagValue;
static volatile uint32_t TimeStamp = 0;


static MOTION_SENSOR_Axes_t MagOffset = {MAG_HIOFFSET_X, MAG_HIOFFSET_Y, MAG_HIOFFSET_Z}; //pueden estar inicializados
static uint8_t MagCalStatus = MAGNETOMETRO_CALIBRADO; //magenetometro (des)calibrado

static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;



/* ------------------------------------Prototipos de funciones ----------------------------------------------------------*/

void MotionFX_manager_init(void);
void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);

static void Accelero_Sensor_Handler(int16_t *pAcceleroXYZ);
static void Gyro_Sensor_Handler(float *pGyroXYZ);
static void Magneto_Sensor_Handler(int16_t *pMagnetoXYZ);

void BSP_SENSOR_ACC_GetOrientation(char *Orientation);
void BSP_SENSOR_GYR_GetOrientation(char *Orientation);
void BSP_SENSOR_MAG_GetOrientation(char *Orientation);


void get_DatosIMU(int16_t* pAcc, float* pGyr, int16_t* pMag);
void MX_MEMS_Init(void);
void MX_MEMS_Process(float* roll, float* pitch, float* yaw);
void FX_Data_Handler(float* roll, float* pitch, float* yaw);

void DWT_Init(void);
void DWT_Start(void);
uint32_t DWT_Stop(void);
char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data);
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data);


/* ------------------------------------Definicion de funciones ----------------------------------------------------------*/



/**
  * @brief  Funcion de inicializacion de megnetometro y acelerometro del paquete de bibliotecas de ST X-CUBE-MEMS1
  * @retval None
  * @retval None
  */
 void MX_MEMS_Init(void)
{
  float ans_float;

  /* Sensor Fusion API initialization function */
  MotionFX_manager_init();

  /* Enable magnetometer calibration */
  MotionFX_MagCal_init(ALGO_PERIOD, 1);

  /* Test if calibration data are available */
  MFX_MagCal_output_t mag_cal_test;
  MotionFX_MagCal_getParams(&mag_cal_test);

  /* If calibration data are available load HI coeficients : Hard Iron */
  if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
  {
    ans_float = (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
    MagOffset.x = (int32_t)ans_float;
    ans_float = (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
    MagOffset.y = (int32_t)ans_float;
    ans_float = (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
    MagOffset.z = (int32_t)ans_float;

    MagCalStatus = 1;
  }

  DWT_Init();


  MotionFX_enable_6X(MFX_ENGINE_DISABLE); 	//solo magneto y accelerometro
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);	//Habilitamos los 3 ejes de los 3 sensores

}


/**
  * @brief  Función principal de procesado del algoritmo. Cada llamada representa una iteración de cómputo del
  * algoritmo entero de las funciones de la libería. Recaba datos de sensores y provesa el algoritmo, realizando un
  * cambio de base espacial en función de los ejes definidos para la aplicación.
  * @param punteros a variables que albergan los datos a ser devueltos por el algoritmo
  * @retval None
  */
 void MX_MEMS_Process(float* roll, float* pitch, float* yaw)
{
  int16_t AccData[MFX_NUM_AXES] = {0};
  float GyroData[MFX_NUM_AXES] = {0.0f};
  int16_t MagnetoData[MFX_NUM_AXES] = {0};

	 /*SENSOR	gaterhing Data*/
	get_DatosIMU(&AccData[0], &GyroData[0], &MagnetoData[0]);

	AccValue.x = (int32_t) AccData[0];  AccValue.y = (int32_t) AccData[1];  AccValue.z = (int32_t) AccData[2];
	MagValue.x = (int32_t) MagnetoData[0]; MagValue.y = (int32_t) MagnetoData[1];  MagValue.z = (int32_t) MagnetoData[2];
	GyrValue.x = (int32_t) roundf(GyroData[0]);  GyrValue.y = (int32_t) roundf(GyroData[1]);    GyrValue.z = (int32_t) roundf(GyroData[2]);

	/* Sensor Fusion specific part */
	FX_Data_Handler(roll, pitch, yaw);

	//printf("\x1b[2J" "\x1b[f"); //limpiar buffer y ventana de TeraTerm

}


/**
 * @brief  Función de llamada a la funcion interna de procesado de datos. Convierte los datos de entrada a las unidades
 * correspondientes.
 * @param  punteros a vectores que albergan los datos devueltos por el algoritmo
 * @retval None
 */
 void FX_Data_Handler(float* roll, float* pitch, float* yaw)
{
 // uint32_t elapsed_time_us = 0U;
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;

	/* Convert angular velocity from [mdps] to [dps] */
	data_in.gyro[0] = (float)GyrValue.x * FROM_MDPS_TO_DPS;
	data_in.gyro[1] = (float)GyrValue.y * FROM_MDPS_TO_DPS;
	data_in.gyro[2] = (float)GyrValue.z * FROM_MDPS_TO_DPS;

	/* Convert acceleration from [mg] to [g] */
	data_in.acc[0] = (float)AccValue.x * FROM_MG_TO_G;
	data_in.acc[1] = (float)AccValue.y * FROM_MG_TO_G;
	data_in.acc[2] = (float)AccValue.z * FROM_MG_TO_G;

	/* Convert magnetic field intensity from [mGauss] to [uT / 50] */
	data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
	data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
	data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

	/* Run Sensor Fusion algorithm */
	DWT_Start();
	MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);
	DWT_Stop();

//	typedef struct		//Estructura de datos que maneja la liberia
//	{
//	  float rotation_9X[MFX_NUM_AXES];            /* 9 axes yaw, pitch and roll */
//	  float quaternion_9X[MFX_QNUM_AXES];         /* 9 axes quaternion */
//	  float gravity_9X[MFX_NUM_AXES];             /* 9 axes device frame gravity */
//	  float linear_acceleration_9X[MFX_NUM_AXES]; /* 9 axes device frame linear acceleration */
//	  float heading_9X;                           /* 9 axes heading */
//	  float headingErr_9X;                        /* 9 axes heading error in deg */
//	} MFX_output_t;


	*roll =  pdata_out->rotation_9X[2] ; 	//Cambio sistema referencia del vehiculo, a NED
	*pitch=  pdata_out->rotation_9X[1] ;
	*yaw =   (pdata_out->rotation_9X[0] < 0.0f ) ? (360.0f + pdata_out->rotation_9X[0]) : (pdata_out->rotation_9X[0]); //positivo

}


 /**
  * @brief   Funcion para muestrear todos los datos de la IMU desarrollada.
   * @param   punteros a variables que albergan los vectores de datos en componentes
  * @retval  void
  */
 void get_DatosIMU(int16_t* pAcc, float* pGyr, int16_t* pMag)
 {

	Accelero_Sensor_Handler(pAcc);
	Gyro_Sensor_Handler(pGyr);
	Magneto_Sensor_Handler(pMag);

 }



 /**
  * @brief  Initialize the MotionFX engine
  * @param  None
  * @retval None
  */
 void MotionFX_manager_init(void)
 {
   MotionFX_initialize();

   MotionFX_getKnobs(ipKnobs);

   BSP_SENSOR_ACC_GetOrientation(ipKnobs->acc_orientation);
   BSP_SENSOR_GYR_GetOrientation(ipKnobs->gyro_orientation);
   BSP_SENSOR_MAG_GetOrientation(ipKnobs->mag_orientation);

   ipKnobs->start_automatic_gbias_calculation = AUTOMATIC_BIAS_CALCULATION;

   if ( ipKnobs->start_automatic_gbias_calculation ){
	   printf("Calibrando sensores MEMS... espere 10 seg...\n");
	   HAL_Delay(10*1000);
	   MotionFX_getKnobs(ipKnobs);
   }
   else {
   ipKnobs->gbias_acc_th_sc_6X = GBIAS_ACC_TH_SC_6X;
   ipKnobs->gbias_gyro_th_sc_6X = GBIAS_GYRO_TH_SC_6X;
   ipKnobs->gbias_mag_th_sc_6X = GBIAS_MAG_TH_SC_6X;

   ipKnobs->gbias_acc_th_sc_9X = GBIAS_ACC_TH_SC_9X;
   ipKnobs->gbias_gyro_th_sc_9X = GBIAS_GYRO_TH_SC_9X;
   ipKnobs->gbias_mag_th_sc_9X = GBIAS_MAG_TH_SC_9X;
   }

   ipKnobs->ATime = ATIME_REF; //alto para bajas aceleraciones <g
   ipKnobs->FrTime = FRTIME_REF;
   ipKnobs->MTime = MTIME_REF;

   ipKnobs->output_type = MFX_ENGINE_OUTPUT_NED; //apunta Norte Este Down
   ipKnobs->LMode = 1;
   ipKnobs->modx = 1;

   MotionFX_setKnobs(ipKnobs);

   MotionFX_enable_6X(MFX_ENGINE_DISABLE); //solo magneto y accelerometro
   MotionFX_enable_9X(MFX_ENGINE_DISABLE);	//3 ejes de 3 sensores
 }



/**
 * @brief  Run Motion Sensor Data Fusion algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @param  delta_time Delta time
 * @retval None
 */
void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time)
{

    MotionFX_propagate(data_out, data_in, &delta_time); //FILTRO DE KALMAN

    MotionFX_update(data_out, data_in, &delta_time, NULL);

}




/**
 * @brief  Funcion driver del componente MEMS acelerómetro de la placa. Recaba datos de el.
 * @param  puntero a variable que recoge los datos devueltos, en componentes
 * @retval None
 */
static void Accelero_Sensor_Handler(int16_t *pAcceleroXYZ)
{
	 BSP_ACCELERO_AccGetXYZ(pAcceleroXYZ);
}

/**
 * @brief  Funcion driver del componente MEMS giroscopio de la placa. Recaba datos de el.
 * @param  puntero a variable que recoge los datos devueltos, en componentes
 * @retval None
 */
static void Gyro_Sensor_Handler(float* pGyroXYZ)
{
	 BSP_GYRO_GetXYZ(pGyroXYZ);
}

/**
 * @brief  Funcion driver del componente MEMS magnetómetro de la placa. Recaba datos de el, y en caso de que
 * los valores de calibración del magnetómetro no se encuentren calibrados, pide al usuario realizar una calibración
 * manual y devuelve por consola los valores de calibración.
 * @param  Msg the MAG part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(int16_t *pMagnetoXYZ)
{
	int16_t magnetoComponentes[MFX_NUM_AXES] = {0};

	  BSP_MAGNETO_GetXYZ(&magnetoComponentes[0]);

	  if (MagCalStatus == 0U) //si el magnetometro esta sin calibrar
	  {
		  float ans_float;
		  MFX_MagCal_input_t mag_data_in;
		  MFX_MagCal_output_t mag_data_out;
		  uint32_t mseg_calibmag = HAL_GetTick(), mseg_act;

		  printf("\nATENCION: El Magnetometro no se encuentra calibrado. Por favor, coja el dispositivo con una mano con cuidado \n"
				  "y realice movimientos con la mu%ceca en forma de ocho tumbado durante algunos segundos...\n ", 165); //%c para la 'ñ'

		  do{
			  mseg_act = HAL_GetTick();

			  if(mseg_act - mseg_calibmag >= (1000.0f/FREC_CALIB_MAGNETO) ) {

				BSP_MAGNETO_GetXYZ(&magnetoComponentes[0]);

				mag_data_in.mag[0] = (float) magnetoComponentes[0] * FROM_MGAUSS_TO_UT50;
				mag_data_in.mag[1] = (float) magnetoComponentes[1] * FROM_MGAUSS_TO_UT50;
				mag_data_in.mag[2] = (float) magnetoComponentes[2] * FROM_MGAUSS_TO_UT50;

				mag_data_in.time_stamp = (int)TimeStamp;
				TimeStamp += (mseg_act - mseg_calibmag);
				mseg_calibmag = mseg_act;

				MotionFX_MagCal_run(&mag_data_in);
				MotionFX_MagCal_getParams(&mag_data_out);
			  }
		  }while(mag_data_out.cal_quality != MFX_MAGCALGOOD);

		  MagCalStatus = 1;

		  ans_float = (float)(mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS );
		  MagOffset.x = (int32_t) roundf(ans_float);
		  ans_float = (float)(mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS );
		  MagOffset.y = (int32_t) roundf(ans_float);
		  ans_float = (float)(mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS );
		  MagOffset.z = (int32_t) roundf(ans_float);

		  printf("\n Los nuevos valores de calibracion del Magnetometro en Hard Iron son: \n"
				  "MagOffset.x = %d \nMagOffset.y = %d \nMagOffset.z = %d \n\n",
				  (int)MagOffset.x, (int)MagOffset.y ,(int)MagOffset.z );

		  /* Disable magnetometer calibration */
		  MotionFX_MagCal_init(ALGO_PERIOD, 0); //STOP magneto calibration
	  }


	  *(pMagnetoXYZ+0) = (int16_t)(magnetoComponentes[0] - MagOffset.x);
	  *(pMagnetoXYZ+1) = (int16_t)(magnetoComponentes[1] - MagOffset.y);
	  *(pMagnetoXYZ+2) = (int16_t)(magnetoComponentes[2] - MagOffset.z);
}

/**
  * @brief  Get accelerometer sensor orientation
  * @param  Orientation Pointer to sensor orientation
  * @retval None
  */
void BSP_SENSOR_ACC_GetOrientation(char *Orientation)
{
  Orientation[0] = 'e'; //x del accelerometro & giroscopio
  Orientation[1] = 'n'; //y del accelerometro & giroscopio
  Orientation[2] = 'u'; //z del accelerometro & giroscopio
}

/**
  * @brief  Get gyroscope sensor orientation
  * @param  Orientation Pointer to sensor orientation
  * @retval None
  */
void BSP_SENSOR_GYR_GetOrientation(char *Orientation)
{
  Orientation[0] = 'e'; //x del accelerometro & giroscopio
  Orientation[1] = 'n'; //y del accelerometro & giroscopio
  Orientation[2] = 'u'; //z del accelerometro & giroscopio
}

/**
  * @brief  Get magnetometer sensor orientation
  * @param  Orientation Pointer to sensor orientation
  * @retval None
  */
void BSP_SENSOR_MAG_GetOrientation(char *Orientation)
{
  Orientation[0] = 'w'; //x del magnetometro
  Orientation[1] = 's'; //y del magnetometro
  Orientation[2] = 'u'; //z del magnetometro
}


/**
 * @brief  Initialize DWT register for counting clock cycles purpose
 * @param  None
 * @retval None
 */
 void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
}

/**
 * @brief  Start counting clock cycles
 * @param  None
 * @retval None
 */
 void DWT_Start(void)
{
  DWT->CYCCNT = 0; /* Clear count of clock cycles */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable counter */
}

/**
 * @brief  Stop counting clock cycles and calculate elapsed time in [us]
 * @param  None
 * @retval Elapsed time in [us]
 */
 uint32_t DWT_Stop(void)
{
  volatile uint32_t cycles_count = 0U;
  uint32_t system_core_clock_mhz = 0U;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
  cycles_count = DWT->CYCCNT; /* Read count of clock cycles */

  /* Calculate elapsed time in [us] */
  system_core_clock_mhz = SystemCoreClock / 1000000U;
  return cycles_count / system_core_clock_mhz;
}

 /**
  * @brief  Load calibration parameter from memory
  * @param  dataSize length ot the data
  * @param  data pointer to the data
  * @retval (1) fail, (0) success
  */
 char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data)
 {
   return (char)1;
 }

 /**
  * @brief  Save calibration parameter to memory
  * @param  dataSize length ot the data
  * @param  data pointer to the data
  * @retval (1) fail, (0) success
  */
 char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data)
 {
   return (char)1;
 }

#endif  /* APPLICATION_USER_MI_MEMS_H_ */

/************************ (C) COPYRIGHT Sergio Vera Muñoz --- TFG 2020   --- *****END OF FILE****/


