  /******************************************************************************
  * @file    GenericMQTT.h
  * @author  Sergio Vera Muñoz
  * @brief   Libreria de funciones de uso de protocolo MQTT para conexión con
  *  		 broker en la nube. Fichero creado para la organización de funciones
  *  		 del proyecto de TFG sobre la placa de desarrollo de STM32 L4 -> B-L475E-IOT01A
  ******************************************************************************
  * @attention
  *
  *  Copyright (c) STM32  - TFG: "Sensor IoT para integración de
  *  generacion fotovoltáica en vehículos eléltricos". ETSIDI - UPM
  * All rights reserved
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW.
  ******************************************************************************
  */
#ifndef APPLICATION_USER_GENERICMQTT_H_
#define APPLICATION_USER_GENERICMQTT_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "iot_flash_config.h"
#include "msg.h"
#include "timingSystem.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "MQTTClient.h"
#include "cJSON.h"

/* Defines Privados ------------------------------------------------------------*/

#define MODEL_MAC_SIZE                    13
#define MODEL_DEFAULT_MAC                 "0102030405"
#define MODEL_DEFAULT_LEDON               true

#define MQTT_SEND_BUFFER_SIZE             600

#define MQTT_CMD_TIMEOUT                  5000
#define MAX_SOCKET_ERRORS_BEFORE_NETIF_RESET  3	//este parámetro da el numero de intentos de conexion fallidos

#define MQTT_TOPIC_BUFFER_SIZE            100  /**< Maximum length of the application-defined topic names. */
#define MQTT_MSG_BUFFER_SIZE              MQTT_SEND_BUFFER_SIZE /**< Maximum length of the application-defined MQTT messages. */


/* Private typedef -----------------------------------------------------------*/

typedef struct {
  char      mac[MODEL_MAC_SIZE];      /*< To be read from the netif */
  uint32_t  ts;           /*< Tick count since MCU boot. */
  int16_t   ACC_Value[3]; /*< Accelerometer */
  float     GYR_Value[3]; /*< Gyroscope */
  int16_t   MAG_Value[3]; /*< Magnetometer */

} pub_data_t;

typedef struct {
  char *HostName;
  char *HostPort;
  char *ConnSecurity;
  char *MQClientId;
  char *MQUserName;
  char *MQUserPwd;
} device_config_t;

/** TLS connection security level */
typedef enum {
  CONN_SEC_UNDEFINED = -1,
  CONN_SEC_NONE = 0,          /**< Clear connection */
  CONN_SEC_SERVERNOAUTH = 1,  /**< Encrypted TLS connection, with no authentication of the remote host: Shall NOT be used in a production environment. */
  CONN_SEC_SERVERAUTH = 2,    /**< Encrypted TLS connection, with authentication of the remote host. */
  CONN_SEC_MUTUALAUTH = 3     /**< Encrypted TLS connection, with mutual authentication. */
} conn_sec_t;


/* Declaraicion de variables -----------------------------------------------*/

static unsigned char mqtt_send_buffer[MQTT_SEND_BUFFER_SIZE];
  	  	 unsigned char mqtt_read_buffer[1]; 	//no se usará
bool b_mqtt_connected = false;
device_config_t * device_config = NULL;
conn_sec_t connection_security  = CONN_SEC_UNDEFINED;
const char * ca_cert      = NULL;
const char * device_cert  = NULL;
const char * device_key   = NULL;

const static bool g_continueRunning = true;			//para que el progama entre en bucle infinito
static bool g_publishData = true;				//para entrar de primeras en el bucle de publicacion de datos
static int g_connection_needed_score = 1;

pub_data_t pub_data = { MODEL_DEFAULT_MAC, 0 };

static char mqtt_pubtopic[MQTT_TOPIC_BUFFER_SIZE];
static char mqtt_msg[MQTT_MSG_BUFFER_SIZE];



/* Prototipos privados de funciones -----------------------------------------------*/

int stiot_publish(void * mqtt_ctxt, const char * topic, const char * msg);
int32_t comp_left_ms(uint32_t init, uint32_t now, uint32_t timeout);
int network_read(Network* n, unsigned char* buffer, int len, int timeout_ms);
int network_write(Network* n, unsigned char* buffer, int len, int timeout_ms);

int parse_and_fill_device_config(device_config_t ** pConfig, const char *string);
void free_device_config(device_config_t * config);
int string_allocate_from_token(char ** pDestString, char * tokenName, const char * sourceString);
/* The init/deinit netif functions are called from cloud.c.
 * However, the application needs to reinit whenever the connectivity seems to be broken. */
extern int net_if_reinit(void * if_ctxt);


/* Funciones externas de otros ficheros-----------------------------------------------------------*/


/* Declaraciones de dichas funciones -----------------------------------------------*/


int cloud_device_enter_credentials(void)
{
  iot_config_t iot_config;
  int ret = 0;

  memset(&iot_config, 0, sizeof(iot_config_t));

  strcpy( iot_config.device_name, CREDENCIALES_SERV_IOT);

  printf("\nEnter the connection string of your device:\n"
         "template with MQTT authentication:    HostName=xxx;HostPort=xxx;ConnSecurity=x;MQClientId=xxx;MQUserName=xxx;MQUserPwd=xxx;\n"
         "template without MQTT authentication: HostName=xxx;HostPort=xxx;ConnSecurity=x;MQClientId=xxx;\n");

  getInputString(iot_config.device_name, USER_CONF_DEVICE_NAME_LENGTH);
  msg_info("read: --->\n%s\n<---\n", iot_config.device_name);

  if(setIoTDeviceConfig(&iot_config) != 0)
  {
    ret = -1;
    msg_error("Failed programming the IoT device configuration to Flash.\n");
  }

  return ret;
}

bool app_needs_device_keypair()
{
  const char * config_string = NULL;
  device_config_t * device_config = NULL;
  conn_sec_t security = CONN_SEC_UNDEFINED;

  if(getIoTDeviceConfig(&config_string) != 0)
  {
    msg_error("Failed retrieving the device configuration string.\n");
  }
  else
  {
    if (parse_and_fill_device_config(&device_config, config_string) == 0)
    {
      security = (conn_sec_t) atoi(device_config->ConnSecurity);
      free_device_config(device_config);
    }
    else
    {
      msg_error("Could not parse the connection security settings from the configuration string.\n");
    }
  }

  return (security == CONN_SEC_MUTUALAUTH) ? true : false;
}

/** Function to read data from the socket opened into provided buffer
 * @param - Address of Network Structure
 *        - Buffer to store the data read from socket
 *        - Expected number of bytes to read from socket
 *        - Timeout in milliseconds
 * @return - Number of Bytes read on SUCCESS
 *         - -1 on FAILURE
 **/
int network_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  int bytes;

  bytes = net_sock_recv((net_sockhnd_t) n->my_socket, buffer, len);
  if(bytes < 0)
  {
    msg_error("net_sock_recv failed - %d\n", bytes);
    bytes = -1;
  }

   return bytes;
}

/** Function to write data to the socket opened present in provided buffer
 * @param - Address of Network Structure
 *        - Buffer storing the data to write to socket
 *        - Number of bytes of data to write to socket
 *        - Timeout in milliseconds
 * @return - Number of Bytes written on SUCCESS
 *         - -1 on FAILURE
 **/
int network_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  int rc;

  rc = net_sock_send((net_sockhnd_t) n->my_socket, buffer, len);
  if(rc < 0)
  {
    msg_error("net_sock_send failed - %d\n", rc);
    rc = -1;
  }

  return rc;
}

/**
 * MQTT publish API abstraction called by the metering loop.
 */
int stiot_publish(void * mqtt_ctxt, const char * topic, const char * msg)
{
  int rc;
  MQTTMessage mqmsg;
  memset(&mqmsg, 0, sizeof(MQTTMessage));
  mqmsg.qos = QOS0;
  mqmsg.payload = (char *) msg;
  mqmsg.payloadlen = strlen(msg);

  rc = MQTTPublish(mqtt_ctxt, topic, &mqmsg);
  if (rc != MQSUCCESS)
  {
    msg_error("\n\nFallido al publicar la Payload: %s \n en el Tema: %s\n", (char *)(mqmsg.payload), topic);
  }
  return rc;
}


/** Look for a 'key=value' pair in the passed configuration string, and return a new buffer
 *  holding the 'value' field.
 */
int string_allocate_from_token(char ** pDestString, char * tokenName, const char * sourceString)
{
  int ret = 0;
  char * key = NULL;
  char * value = NULL;

  if ((key = strstr(sourceString, tokenName)) != NULL)
  {
    int size = 0;
    value = key + strlen(tokenName);    /* '=' key=value separator is part of tokenName. */
    if ((key = strstr(value, ";")) != NULL)
    {
      size = key - value;
    }
    *pDestString = malloc(size + 1);
    if (*pDestString != NULL)
    {
      memcpy(*pDestString, value, size);
      (*pDestString)[size] = '\0';
    }
    else
    {
      msg_error("Allocation failed\n");
    }
  }

  return ret;
}


/** Allocate and return a device_config_t structure initialized with the values defined by the passed
 *  configuration string.
 *  The buffers holding the structure and those fields are allocated dynamically by the callee, and
 *  must be freed after usage by free_device_config().
 */
int parse_and_fill_device_config(device_config_t ** pConfig, const char *string)
{
  int ret = -1;
  device_config_t * config = NULL;

  if (strlen(string) > USER_CONF_DEVICE_NAME_LENGTH)
  {
    msg_error("Cannot parse the configuration string:  It is not null-terminated!\n");
  }
  else
  {
    if (pConfig == NULL)
    {
      msg_error("Null parameter\n");
    }
    else
    {
      config = malloc(sizeof(device_config_t));
      memset(config, 0, sizeof(device_config_t));

      ret = string_allocate_from_token(&config->HostName, "HostName=", string);
      ret |= string_allocate_from_token(&config->HostPort, "HostPort=", string);
      ret |= string_allocate_from_token(&config->ConnSecurity, "ConnSecurity=", string);
      ret |= string_allocate_from_token(&config->MQClientId, "MQClientId=", string);
      ret |= string_allocate_from_token(&config->MQUserName, "MQUserName=", string);
      ret |= string_allocate_from_token(&config->MQUserPwd, "MQUserPwd=", string);

      if (ret != 0)
      {
        msg_error("Failed parsing the device configuration string.\n");
        free_device_config(config);
      }
      else
      {
        *pConfig = config;
        ret = 0;
      }
    }
  }

  return ret;
}


/** Free a device_config_t allocated by parse_and_fill_device_config().
 */
void free_device_config(device_config_t * config)
{
  if (config != NULL)
  {
    if (config->HostName != NULL) free(config->HostName);
    if (config->HostPort != NULL) free(config->HostPort);
    if (config->ConnSecurity != NULL) free(config->ConnSecurity);
    if (config->MQClientId != NULL) free(config->MQClientId);
    if (config->MQUserName != NULL) free(config->MQUserName);
    if (config->MQUserPwd != NULL) free(config->MQUserPwd);

    free(config);
  }
  else
  {
    msg_warning("Attemped to free a non-allocated config structure.\n");
  }
}



#endif /* APPLICATION_USER_GENERICMQTT_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE************************************************/
