/**
  ******************************************************************************
  * @file    wifi_net.c
  * @author  MCD Application Team
  * @brief   Wifi-specific NET initialization.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifdef USE_WIFI

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wifi.h"
#include "iot_flash_config.h"

/* Private defines -----------------------------------------------------------*/

#ifdef ES_WIFI_PRODUCT_NAME_SIZE
#define WIFI_PRODUCT_INFO_SIZE                      ES_WIFI_PRODUCT_NAME_SIZE
#define WIFI_PAYLOAD_SIZE                           ES_WIFI_PAYLOAD_SIZE
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
int net_if_init(void * if_ctxt);
int net_if_deinit(void * if_ctxt);
int net_if_reinit(void * if_ctxt);

/* Functions Definition ------------------------------------------------------*/
int net_if_init(void * if_ctxt)
{
  const char *ssid = "WIFI_TFGSVM";
  const char  *psk = "Heliodorum98";
  WIFI_Ecn_t security_mode = WIFI_ECN_WPA2_PSK;
  char moduleinfo[WIFI_PRODUCT_INFO_SIZE];
  WIFI_Status_t wifiRes;
  uint8_t macAddress[6];
  int wifiConnectCounter = 0;
  bool skip_reconf = false;

  printf("\n*** Conexion WiFi ***\n\n");

  skip_reconf = (checkWiFiCredentials(&ssid, &psk, (uint8_t *) &security_mode) == HAL_OK);

  if (skip_reconf == false)
  {
    printf("Debe introducir la configuracion de su red WiFi para continuar.\n");
    do
    {
      updateWiFiCredentials();
    } while (checkWiFiCredentials(&ssid, &psk, (uint8_t *) &security_mode) != HAL_OK);
  }
  
  /*  Wifi Module initialization */
  printf("Inicializando el modulo WiFi...\n");
  
  wifiRes = WIFI_Init();
  if ( WIFI_STATUS_OK != wifiRes )
  {
    printf("Fallo en la inicializacion del modulo WiFi\n");
    return -1;
  }
    
  /* Retrieve the WiFi module mac address to confirm that it is detected and communicating. */
  WIFI_GetModuleName(moduleinfo);
  printf("Modulo WiFi inicializado con exito:  %s",moduleinfo);
  
  WIFI_GetModuleID(moduleinfo);
  printf("Modelo: %s",moduleinfo);
  
  WIFI_GetModuleFwRevision(moduleinfo);
  printf("FW: %s\n",moduleinfo);
  
  printf("Obteniendo la direccion MAC del modulo WiFi:");
  wifiRes = WIFI_GetMAC_Address( (uint8_t*)macAddress);
  if ( WIFI_STATUS_OK == wifiRes)
  {
    printf(" %02x:%02x:%02x:%02x:%02x:%02x\n",
         macAddress[0], macAddress[1], macAddress[2],
         macAddress[3], macAddress[4], macAddress[5]);
  }
  else
  {
       printf("Fallo al obtener la direccion MAC del modulo WiFi\n");
  }
  /* Connect to the specified SSID. */


  printf("\n   Buscando el punto de Acceso WiFi con ssid: %s.......\n", ssid);
  do 
  {
    printf("\n Conectando al hotspot: %s    Intento %d...",ssid, ++wifiConnectCounter);
    wifiRes = WIFI_Connect(ssid, psk, security_mode);
    if (wifiRes == WIFI_STATUS_OK) break;
  }  while (wifiConnectCounter <= 17);	//REALIZA 17 INTENTOS COMO MAXIMO
  
  if (wifiRes == WIFI_STATUS_OK)
  {
    printf("\nCONECTADO al HOTSPOT WIFI de:  %s\n",ssid);
  }
  else
  {
    printf("\nFallo al conectar al AP %s\n",ssid);
  }
  
  return (wifiRes == WIFI_STATUS_OK)?0:-1;
}


int net_if_deinit(void * if_ctxt)
{
  return 0;
}


int net_if_reinit(void * if_ctxt)
{
  int ret = 0;
  const char *ssid = "WIFI_TFGSVM";
  const char  *psk = "Heliodorum98";
  WIFI_Ecn_t security_mode = WIFI_ECN_WPA2_PSK;
  WIFI_Status_t wifiRes;
  
  /* El añadir las funciones de WIFI_Disconnect() y WIFI_ResetModule() no surten efecto,
   *  y dan errores a la segunda iteracion de la funcion, lo cual resulta comprensible*/

  wifiRes = WIFI_Init();
  if (wifiRes != WIFI_STATUS_OK)
  {
    msg_error("\n\t WIFI_Init() failed.\n");
  }
  
  if (checkWiFiCredentials(&ssid, &psk, (uint8_t *) &security_mode) != HAL_OK)
  {
    ret = -1;
  }
  else
  {
	  //sin entrar en un bucle de intentos ni de timeout, solo tratando de reconectar
    wifiRes = WIFI_Connect(ssid, psk, security_mode);

    if (wifiRes == WIFI_STATUS_OK)
    {
      printf("\nReconectado al HotSpot: %s\n",ssid);
    }
    else
    {
      printf("\nFallado al reconectar con el HotSpot: %s . Tiempo limite superado\n",ssid);
      ret = -1;
    }
  }
    
  return ret;
}

#endif /* USE_WIFI */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
