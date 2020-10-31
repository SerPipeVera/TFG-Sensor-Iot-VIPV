/**
  ******************************************************************************
  * @file    AppIoT_TFG_VIPV.c
  * @author  Sergio Vera Muñoz
  * @brief   Fichero con la Aplicación principal del Sensor IoT para VIPV. Realiza
  * 		 la conexion a internet, recaba datos y los publica.
  ******************************************************************************
  * @attention
  *
  *  Copyright (c) 2020 Sergio Vera - TFG: "Sensor IoT para integración de
  *  generacion fotovoltáica en vehículos eléltricos". ETSIDI - UPM - IES
  * All rights reserved
  *
  * THIS SOFTWARE IS PROVIDED BY SERGIOVERAELECTRONICS & STMICROELECTRONICS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING,
  *  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "AppIoT_TFG_VIPV.h"


/* Private variables ---------------------------------------------------------*/

static bool estado = CONECTADO;
#ifdef ENABLE_LOWPWR
bool modo_BajoConsumo = false, ocioso = true;
#endif
static uint16_t contador_lectura = 0, contador_MEMS = 0;

static megaDato vectorLecturaDato[N_ELEMENTOS] = { {0.0f} };	//inicializacion a 0 de todo el vector

static bool flag_lectura_datos=false, flag_publi_datos = false, flag_recupera_datos = false;
static bool flag_lecturaMEMS = false;

static float alebeo_sum = 0.0f, cabeceo_sum = 0.0f, guino_sum = 0.0f;

RTC_TimeTypeDef sTiempo_actual;		//Variables para el RTC
RTC_DateTypeDef sDia_actual;
float Hora_Amanecer_Oficial = 0.0f;  //por defecto, que no duerma nada
float Hora_Atardecer_Oficial = 24.0f;	//esto en relacion al GMT, la franja que usa el RTC

fifo miFIFO;	//Estructura FIFO para la recuperación de datos
megaDato mimegaDato = {0.0f};	//Estrucutra de dato con todas las magnitudes a medir

MQTTClient client;	//Variables para implementar la conexión MQTT a través de un socket
Network network;
net_sockhnd_t socket;

/**
 * @brief   Funcion principal del programa, la cual establece la conexión a internet, abre los sokets,
 * establece la conexión MQTT con el servidor IoT y lleva a cabo las comprobaciones de errores oportunas.
 * El bucle principal es infinito, con bucles itnernos propios condicionados a variables de salida de éstos.
 * Se basa en llamadas a otras rutinas y funciones para distribuir tareas. En caso de error severo, informa
 * al usuario por pantalla y resetea el programa completo.
 * @param   void: no recibe parametros
 * @retval  no devuelve parametros.
 */
void aplicacion_ClienteMQTT_XCLD_IoT(void)
{
  int ret = 0;	//valor de retorno de algunas funciones, para evaluarlo
  bool conex_correcta = false;
  iniciado_Programa = false;

  memset(&mimegaDato, 0, sizeof(mimegaDato));


  if ( inicializa_ConexionIoT() == true)  {	//si es correcto

    iniciado_Programa = true;
    get_AmanecerAtardecer(&Hora_Amanecer_Oficial, &Hora_Atardecer_Oficial, LATITUD_STD, LONGITUD_STD) ;
    	/* De partida, sin estar listo el modulo de GPS, calculamos a priori si es de noche o de dia en el IES */

    do { 	/*++++++++++++++++++++++ B U C L E    P R I N C I P A L    D E L	  P R O G R A M A ++++++++++++++++++++++++++++++++*/

      /*Asegura los protocolos y niveles de seguidad en la conexion con el shocket*/
       ret = check_protocoloConexion();

       conex_correcta = inicia_ClienteMQTT(ret);

      if (conex_correcta)  {

    	 switch_Temporizadores(ENCENDER_TIMERS);

         bucle_Principal();  /*-------------------------BUCLE INTERNO DE ENVÍO DE DATOS----------------------------*/

         switch_Temporizadores(APAGAR_TIMERS);

      }
      else   {

    	  if ( reconecta_WiFi() ) { //si ha habido exito
    		  HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_SET);	//LED de conexión Wi-Fi}
    	  }
      }

      desconectaConexionMQTT();

    } while (g_connection_needed_score > 0);	/* FIN BUCLE PRINCIPAL DE LA APLICACIÓN*/

  }	// fin del if configuracion inicial correcta

  else {

  free_device_config(device_config);
  platform_deinit();

    msg_info("\nLlamando a HAL_NVIC_SystemReset(). Se reseteara el programa...\n");
    HAL_Delay(1500);
    HAL_NVIC_SystemReset();

  }

}		//FIN DE LA FUNCIÓN PRINCIPAL


/**
 * @brief   Funcion que implementa un bucle interno de Lectura-Publicacion de Datos
 * Recaba datos continuamente, hace media de ellos y los publica mientras no haya errores.
 *  Si todo es correcto, permanece leyendo y publicando datos continuamente en bucle,
 *  marcando su salida la variable de estado de conexion global "g_connection_needed_score"
 *  Se basa en llamadas a otras rutinas para distribuir tareas, implementando 3 hilos de ejecucción
 * @param   void: no recibe parametros
 * @retval  no devuelve parametros.
 */
void bucle_Principal(void)  {


#ifdef ENABLE_LOWPWR
  uint32_t n_ocio = 0;
#endif

  do
  {
#ifdef ENABLE_LOWPWR
	ocioso = true;
#endif
#ifdef ENABLE_PULSADOR
	uint8_t command = BP_NOT_PUSHED;
	command = Boton_GetNumPush();

    if (command == BP_SINGLE_PUSH || command == BP_MULTIPLE_PUSH)                  /* If long button push, toggle the telemetry publication. */
    {
      g_publishData = !g_publishData;
      HAL_GPIO_TogglePin(GPIOC, ARD_A1_LEDWIFI_Pin);   //LED conexión Wi-Fi
      msg_info("\n%s del bucle de publicacion de datos.\n", (g_publishData == true) ? "DENTRO" : "FUERA");
    }
#endif

    /***********************************************************************************************************************/
    /*******************************   HILO DE EJECUCCIÓN DE LECTURA DE DATOS **********************************************/
    /***********************************************************************************************************************/
    if ( flag_lectura_datos )  {	  /* Lee los datos de todos los sensores */

    	hilo1_Lectura();

    }



    /*********************************************************************************************************************************/
    /***********************   HILO DE EJECUCCIÓN DE PUBLICACION DE DATOS EN THINGSPEAK **********************************************/
    /*********************************************************************************************************************************/
    if ( flag_publi_datos && (contador_lectura>0) && (g_publishData == true) )	/*Publica los datos de la media*/
    {
    	hilo2_Publicacion();
    }



    /*********************************************************************************************************************************/
    /********************   HILO DE EJECUCCIÓN DE RECUPERACIÓN DE DATOS EN LA FIFO DE MEMORIA SRAM ***********************************/
    /*********************************************************************************************************************************/
    if ( flag_recupera_datos && estaFIFOvacia(&miFIFO)  && (g_publishData == true) )
    {
    	if ( hilo3_Reconexion() ) {	//s ha recuperado la conexion a internet, sale del bucle principal para rehacerla
    		return ;
    	}
	}



    /*********************************************************************************************************************************/
    /********************   HILO DE COMPUTACIÓN ALGORITMO FUSIÓN MEMS Y FILTRO KALMAN ************************************************/
    /*********************************************************************************************************************************/
     if( flag_lecturaMEMS  )
    {
    	computa_algoritmoMEMS();
    }

#ifdef ENABLE_LOWPWR
     if (ocioso)  {	//en caso de que no haya entrado a ninguno de los 3 hilos, suma 1 a la variable n_ocio

    	 if(n_ocio >= NMAX_VUELTAS_OCIO &&  (!modo_BajoConsumo) ) {

    		modo_BajoConsumo = true;
    		entrar_LowPowerMode(); //modo de bajo consumo, despertado por los otros hilos de trabajo

    	}else { n_ocio++ ;}

    }else {
    		n_ocio = 0;  modo_BajoConsumo = false;  //lo resetea
    		}
#endif

  } while ( g_continueRunning );


} //fin de la funcion bucle lectura envio datos


/**
 * @brief   Rutina que implementa la tarea de lectura de datos y contador de muestras del sensor. Es
 * la definición del 1er hilo de ejecucción del bucle principal disparado por LPTIMER1.
 * @param   void: no recibe parametros
 * @retval  no devuelve parametros
 */
void hilo1_Lectura(void)
{

#ifdef ENABLE_LOWPWR
	if(modo_BajoConsumo) {  salir_LowPowerMode();  } //saliendo del modo de bajo consumo
#endif

     recabar_Datos( &vectorLecturaDato[contador_lectura]  );

		flag_lectura_datos = false; //resetea flag
		contador_lectura++;
		contador_MEMS = 0; //reseteo contador MEMS al segundo

		if(contador_lectura >= (N_ELEMENTOS-1)){
		   contador_lectura = (N_ELEMENTOS-1);	//va desde 0 a N-1
		}
}


/**
 * @brief   Rutina que implementa la tarea de publicación de la media de las muestras de datos.
 * En primer lugar, calcula la media del vector de datos, y tras compribar que no existen datos pendientes
 * en el buffer FIFO, realiza su publicación inmediata. En caso contrario, los almacena en la FIFO. Ademas
 * de eso, realiza la comprobación de la hora local para determinar si el dispositivo tiene que entrar en el
 * modo de bajo consumo al estar de noche. Se trata de la 2ª rutina de ejecución del Bucle principal
 * @param   void: no recibe parametros
 * @retval  no devuelve parametros
 */
void hilo2_Publicacion(void)
{

#ifdef ENABLE_LOWPWR
	if(modo_BajoConsumo) { salir_LowPowerMode();  } //saliendo del modo de bajo consumo
#endif

    	flag_publi_datos = false;

    	printf("\n$$$$$$$$$$$$$$$ THREAD DE PUBLICION DE DATOS EN THINGSPEAK $$$$$$$$$$$$$$$\n");
    	calcula_mediaVector(&mimegaDato, vectorLecturaDato, contador_lectura);
    	printf("El N%c de lecturas con la que se ha calculado la Media estadistica para el dato es: %d \n", SUPER_O, contador_lectura+1);
    	contador_lectura = 0; //reseteo del contadores

#ifdef ENABLE_SLEEP
      /*Antes de continuar, compurba si ya es de noche para seguir captando y enviando datos */
      if( SUENYO == get_PeriodoDia(&Hora_Amanecer_Oficial, &Hora_Atardecer_Oficial) && estaFIFOvacia(&miFIFO)==0 )
    	{
      	printf("\n La hora actual indica que es de Noche, el dispositivo entrara en Sleep Mode...\n");
      	entraSleepMode(Hora_Amanecer_Oficial);
      	//una vez despierta, resetea el programa para reiniciar todo de nuevo
         }
      else
#endif
    	{
		  if( !estaFIFOvacia(&miFIFO) ){	//solo trata de publicar si la lista esta vacía

			  if( publica_DatosThingSpeak(&mimegaDato) == false)
			  {
				 insertarFIFO(&miFIFO, mimegaDato);
				 estado = DESCONECTADO;
				 HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_RESET); //LED conexión Wi-Fi
				 printf("Dato INSERTADO en la FIFO por fallo de conexion en publica_DatosThingSpeak()\n");
				 printf("El numero de nodos en la FIFO es: %d \n", estaFIFOvacia(&miFIFO) );

			  }else {
				  estado = CONECTADO;
				  HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_SET); //LED conexión Wi-Fi
			  }

		  }else{	//si la lista tiene elementos
			 insertarFIFO(&miFIFO, mimegaDato);
			 printf("Dato INSERTADO en la FIFO por encontrarse la FIFO pendiente de envio\n");
			 printf("El numero de nodos en la FIFO es: %d \n", estaFIFOvacia(&miFIFO) );
		  }
      }
}

/**
 * @brief   Rutina que implementa la reconexión al hotspot tras una publciación fallida de los datos.
 * Realiza las pertinentes comprobaciones de conexión, intentos de reconexión e indicaciones exteriores
 * del estado de conexión mediante el LED de conexión Wi-Fi. Si la reconexión ha surtido efecto, devuelve un
 * valor verdadero para que se salga del bucle pirincipal para rehacer la conexión MQTT. En caso contratio, devuelve
 * un valor falso. En caso de que haya que recuperar datos desde la FIFO, los publica a una frecuencia mayor. disparada
 * por el Timer 6. Se trata de la 3ª rutina de ejecución del Bucle principal
 * @param   void: no recibe parametros
 * @retval  estado tras el intento de reconexión
 */
bool hilo3_Reconexion(void)
{

#ifdef ENABLE_LOWPWR
	if(modo_BajoConsumo) {  salir_LowPowerMode();  }  //saliendo del modo de bajo consumo
#endif

    	flag_recupera_datos = false;

    	printf("\n$$$$$$$$$$$$$$$ THREAD DE RECUPERACION DE DATOS DE CONEXION $$$$$$$$$$$$$$$\n");
    	if( estado==DESCONECTADO )  {	//si se encuentra desconectado, trata de reconectar

    		estado = reconecta_WiFi() ;

    		if( estado==CONECTADO )
    		{
    			 HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_SET); //LED conexión Wi-Fi
    			return true;  //sale del bucle principal a rehacer la conexion MQTT
    		}

    		HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_RESET); //LED conexión Wi-Fi
    	}

    	else{	//si ya esta conectado, trata de publicar los datos de la FIFO

			if( publica_DatosThingSpeak( obtenerDatoFIFO(&miFIFO) ) == true)	//si lo publica, lo elimina
			{
				printf("Dato PUBLICADO RECUPERADO con exito, eliminandolo de la fifo...\n");
				eliminarDatoFIFO(&miFIFO);
				estado = CONECTADO;
				 HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_SET); //LED conexión Wi-Fi

			}else {
				estado = DESCONECTADO;
				HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_RESET); //LED conexión Wi-Fi
				}

		}
		printf("El numero de nodos en la FIFO es: %d \n", estaFIFOvacia(&miFIFO) );

		return false; //no sale del bucle principal

}


/**
 * @brief   Funcion para realizar el envío de datos a través de el módulo establecido, el socket,
 * y la configuración IoT de servidor y canales preestablecidos. Lleva a cabo las oportunas comprobaciones
 * de errores, informando al usuario.
 * @param   In:   miDato    estructura del dato a publicar con todas sus magnitudes
 * @retval  Verdadero si exito en la publicación, falso en caso de error
 */
bool publica_DatosThingSpeak(megaDato* miDato)  {

	int resultado = -1;
	bool retorno = true;	//suponemos que no hay problemas a priori
    char* payload = "";

    if( miDato == NULL) {
    	return false;
    }

#ifdef ENABLE_IMPRIMIR_MUESTRAS
    imprimir_Dato(*miDato);
#endif

#ifdef PUBLI_DATOS_THINGSPEAK
    for(int n_canal = 1; n_canal<=2 ; n_canal++)   {	//Bucle de publicacion en los 2 canales

    	printf("\t\tPublicacion de Datos en el Canal %d...\n", n_canal);

    	switch (n_canal)  {

    	case 1: /*+++++++++++++++++++++++++++++++++  CANAL 1 DE THINGSPEAK ++++++++++++++++++++++++++++++++++++++++++++*/
    		snprintf(mqtt_pubtopic, MQTT_TOPIC_BUFFER_SIZE, CANAL1_THINSPEAK_WR_APIKEY);

    		    if( miDato->ubicacion_fix )	//en funcion de si tiene o no la ubicacion disponible, publicara una cosa u otra
    		    {
    		    	payload =   "field1=%f&field2=%f&field3=%f&field4=%f&field5=%f&field6=%f&field7=%f&field8=%f&lat=%f&long=%f&elevation=%f&created_at=%04d-%02d-%02dT%02d:%02d:%02dZ"	;
    		    	//en formato RTC ISO 8601: &created_at=2011-07-18T01:02:03Z  Se consigue con el especificador de printf ( %02d : 2 cifras a rellenar con ceros)
    		    	resultado=  snprintf( mqtt_msg, MQTT_MSG_BUFFER_SIZE,  payload ,
    		        			miDato->irradiancia[0], miDato->irradiancia[1] ,miDato->irradiancia[2], miDato->irradiancia[3], miDato->irradiancia[4] ,
    							miDato->temperatura, miDato->presion, miDato->humedad,
    							miDato->latitud, miDato->longitud, miDato->altitud,
    							miDato->agno,  miDato->mes, miDato->dia ,  miDato->hora, miDato->min, miDato->seg  );
    		    }else
    		    {
    		    	payload =   "field1=%f&field2=%f&field3=%f&field4=%f&field5=%f&field6=%f&field7=%f&field8=%f&created_at=%04d-%02d-%02dT%02d:%02d:%02dZ"	;
    		    	//en formato RTC ISO 8601: &created_at=2011-07-18T01:02:03Z  Se consigue con el especificador de printf ( %02d : 2 cifras a rellenar con ceros)
    		    	resultado=  snprintf( mqtt_msg, MQTT_MSG_BUFFER_SIZE,  payload ,
    		        			miDato->irradiancia[0], miDato->irradiancia[1] ,miDato->irradiancia[2], miDato->irradiancia[3], miDato->irradiancia[4] ,
    							miDato->temperatura, miDato->presion, miDato->humedad,
    							miDato->agno,  miDato->mes, miDato->dia ,  miDato->hora, miDato->min, miDato->seg  );
    		    }
    	break;

    	case 2:   /*+++++++++++++++++++++++++++++++++  CANAL 2 DE THINGSPEAK ++++++++++++++++++++++++++++++++++++++++++++*/
    		snprintf(mqtt_pubtopic, MQTT_TOPIC_BUFFER_SIZE, CANAL2_THINSPEAK_WR_APIKEY);

    		    if( miDato->ubicacion_fix )	//en funcion de si tiene o no la ubicacion disponible, publicara una cosa u otra
    		    {
    		    	payload =   "field1=%f&field2=%f&field3=%f&field4=%f&field5=%f&field6=%f&field7=%f&lat=%f&long=%f&elevation=%f&created_at=%04d-%02d-%02dT%02d:%02d:%02dZ"	;
    		    	//en formato RTC ISO 8601: &created_at=2011-07-18T01:02:03Z  Se consigue con el especificador de printf ( %02d : 2 cifras a rellenar con ceros)
    		    	resultado=  snprintf( mqtt_msg, MQTT_MSG_BUFFER_SIZE,  payload ,
    		        			miDato->latitud, miDato->longitud, miDato->altitud, miDato->velocidad,
    							miDato->alebeo, miDato->cabeceo,  miDato->guino_brujula,
    							miDato->latitud, miDato->longitud, miDato->altitud,
    							miDato->agno,  miDato->mes, miDato->dia ,  miDato->hora, miDato->min, miDato->seg  );
    		    }else
    		    {
    		    	payload =   "field5=%f&field6=%f&field7=%f&created_at=%04d-%02d-%02dT%02d:%02d:%02dZ"	;
    		    	//en formato RTC ISO 8601: &created_at=2011-07-18T01:02:03Z  Se consigue con el especificador de printf ( %02d : 2 cifras a rellenar con ceros)
    		    	resultado=  snprintf( mqtt_msg, MQTT_MSG_BUFFER_SIZE,  payload ,
    		    				miDato->alebeo, miDato->cabeceo,  miDato->guino_brujula,
    							miDato->agno,  miDato->mes, miDato->dia ,  miDato->hora, miDato->min, miDato->seg  );
    		    }
    	break;

    	default:
    	break;
    	}	//fin switch case

        if ( (resultado < 0) || (resultado >= MQTT_MSG_BUFFER_SIZE) )
        {
          msg_error("\n\nError de formato de mensaje Telemetrico.\n");
          retorno &= false;
        }
        else
        {
          resultado = stiot_publish(&client, mqtt_pubtopic, mqtt_msg);  /* Wrapper for MQTTPublish() */

          if (resultado == MQSUCCESS)
          {
        	 // HAL_ResumeTick();	//Habilita ticks
            /* Notificación visual de publciación exitosa de mensajes:LED blink. */
        	  for (uint8_t i=0 ;i<10; i++) {
        		  HAL_GPIO_TogglePin(GPIOC, ARD_A1_LEDWIFI_Pin);
        		  HAL_Delay(15);
        	  }//msg_info("#Publicado en el Tema MQTT: %s \n ->Payload del mensaje enviado: %s\n", mqtt_pubtopic, mqtt_msg);
            /*Puede resultar caragante imprimir todos los mensajes varias veces*/
        	 // HAL_SuspendTick();
          }
          else
          {
            msg_error("\n\nPublicacion Telemetrica fallida. Mensaje error: \n");
            g_connection_needed_score++;
            retorno &= false;
          }

          resultado = MQTTYield(&client, 500);
          if (resultado != MQSUCCESS)
          {
            msg_error("\n\nYield fallido. Mensaje error:\n");
            g_connection_needed_score++;
            retorno &= false;
          }
        }

    }	//fin for(canales)

    if (retorno) printf("\n##### Publicacion EXITOSA de todos los Datos en todos los Canales del servidor ThingSpeak #####\n\n");
    else printf("\nErrores al publicar los Datos, se agregara el dato a la FIFO...\n");

#endif
    return retorno;

}


/**
 * @brief   Funcion para calcular la media aritmética de las muestras a lo largo de un minuto.
 * @param   mediaDatos:   estructura de retorno para devolver la media de todas las magnitudes
 * @param   p_vectorLecturas:   vector de muestras de donde saca la media
 * @param   n_elem:   contador del nº elementos del vector
 * @retval  void no devuelve nada
 */
void calcula_mediaVector(megaDato* mediaDatos, megaDato* p_vectorLecturas, uint8_t n_elem )   {

	uint8_t restador = 0;
	megaDato suma;

	if(n_elem==0){
		printf("Invocada funcion de calcula_mediaVector sin elementos en el vector\n");
		mediaDatos = NULL;	//acuse de recibo
		return ;
	}
	/* Lo primero de todo, obtiene la fecha y hora  a partir del ultimo elemento del vector */
	mediaDatos->agno = (p_vectorLecturas+n_elem-1)->agno;
	mediaDatos->mes = (p_vectorLecturas+n_elem-1)->mes;
	mediaDatos->dia = (p_vectorLecturas+n_elem-1)->dia;
	mediaDatos->hora = (p_vectorLecturas+n_elem-1)->hora;
	mediaDatos->min = (p_vectorLecturas+n_elem-1)->min;
	mediaDatos->seg = (p_vectorLecturas+n_elem-1)->seg;

	memset(&suma, 0, sizeof(suma));	//inicializa a 0

	for (uint8_t i=0; i<n_elem; i++)
	{

		suma.irradiancia[0] += (p_vectorLecturas+i)->irradiancia[0];
		suma.irradiancia[1] += (p_vectorLecturas+i)->irradiancia[1];
		suma.irradiancia[2] += (p_vectorLecturas+i)->irradiancia[2];
		suma.irradiancia[3] += (p_vectorLecturas+i)->irradiancia[3];
		suma.irradiancia[4] += (p_vectorLecturas+i)->irradiancia[4];

		suma.temperatura += (p_vectorLecturas+i)->temperatura;
		suma.humedad += (p_vectorLecturas+i)->humedad;
		suma.presion += (p_vectorLecturas+i)->presion;

		suma.alebeo += (p_vectorLecturas+i)->alebeo;
		suma.cabeceo += (p_vectorLecturas+i)->cabeceo;
		suma.guino_brujula += (p_vectorLecturas+i)->guino_brujula;


		if (noesNAN((p_vectorLecturas+i)->longitud) && noesNAN((p_vectorLecturas+i)->latitud) && noesNAN((p_vectorLecturas+i)->altitud) && noesNAN((p_vectorLecturas+i)->velocidad)
			&& (p_vectorLecturas+i)->longitud != 0.0f && (p_vectorLecturas+i)->latitud != 0.0f && (p_vectorLecturas+i)->altitud != 0.0f	) {
			suma.latitud += (p_vectorLecturas+i)->latitud;
			suma.longitud += (p_vectorLecturas+i)->longitud;
			suma.altitud += (p_vectorLecturas+i)->altitud;
			suma.velocidad += (p_vectorLecturas+i)->velocidad;
		}
		else{
			restador ++;	//añade 1 al numero de datos fallidos en localizacion
		}
	}

		if(restador <= (n_elem/2)) {			/*Si el numero de medidas correctas es mas de la mitad */
			mediaDatos->ubicacion_fix = true;
		}else {  mediaDatos->ubicacion_fix = false; }

		mediaDatos->irradiancia[0] = suma.irradiancia[0] / n_elem;
		mediaDatos->irradiancia[1] = suma.irradiancia[1] / n_elem;
		mediaDatos->irradiancia[2] = suma.irradiancia[2] / n_elem;
		mediaDatos->irradiancia[3] = suma.irradiancia[3] / n_elem;
		mediaDatos->irradiancia[4] = suma.irradiancia[4] / n_elem;

		mediaDatos->temperatura = suma.temperatura / n_elem;
		mediaDatos->humedad = suma.humedad  / n_elem;
	    mediaDatos->presion = suma.presion  / n_elem;

	    mediaDatos->longitud = suma.longitud / (n_elem-restador);	//obtiene la media de todos los elementos menos los que no se hayan añadido
	    mediaDatos->latitud = suma.latitud / (n_elem-restador);
	    mediaDatos->altitud = suma.altitud / (n_elem-restador);
	    mediaDatos->velocidad = suma.velocidad / (n_elem-restador);

	    mediaDatos->alebeo = suma.alebeo / n_elem;
		mediaDatos->cabeceo = suma.cabeceo / n_elem;
		mediaDatos->guino_brujula = suma.guino_brujula / n_elem ;


	    memset(p_vectorLecturas, 0, (N_ELEMENTOS-1)*sizeof(megaDato));	//Reseteo del vector de muestras

}


/**
 * @brief   Funcion para muestrear todas las magnitudes a leer. Es invocada una vez por segundo.
 * Lleva a cabo la conversión de ciertas magnitudes a sus unidades correspondientes, y la
 * comprobación de errores a priori.
 * @param   miLectura:   dirección de memoria a devolver todas las magnitudes en los campos de la estructura
 * @retval  void no devuelve nada
 */
void recabar_Datos(megaDato* miLectura){

	float latit_raw=NAN, longit_raw=NAN, altit_raw=NAN,speed_raw=NAN, temp_raw=NAN, hum_raw=NAN, pres_raw=NAN;


	 if (HAL_RTC_GetTime(&hrtc, &sTiempo_actual, RTC_FORMAT_BIN) != HAL_OK ) {	//prioritario, tomar hora actual
	    	printf("Error al dar las obterner hora-fecha actual del RTC.\n");
	 }
	 if (HAL_RTC_GetDate(&hrtc, &sDia_actual, RTC_FORMAT_BIN) != HAL_OK ) {
	    	printf("Error al dar las obterner hora-fecha actual del RTC.\n");
	 }
	 else{		//lo primero que hace es tomar la fecha y hora actual
		 miLectura->agno = 2000 + sDia_actual.Year;
		 miLectura->mes = sDia_actual.Month;
		 miLectura->dia = sDia_actual.Date ;
		 miLectura->hora = sTiempo_actual.Hours;
		 miLectura->min = sTiempo_actual.Minutes;
		 miLectura->seg = sTiempo_actual.Seconds;
	 }

	HAL_ResumeTick();
	//HAL_GPIO_WritePin(GPIOC, ARD_A2_LEDON_Pin, GPIO_PIN_SET); //indicador visual

	hum_raw = BSP_HSENSOR_ReadHumidity();
	pres_raw = BSP_PSENSOR_ReadPressure();
	temp_raw = BSP_TSENSOR_ReadTemp();

	if (temp_raw != 0.0f && hum_raw != 0.0f && pres_raw!=0.0f ) {	//comprobacion errores
		miLectura->temperatura = temp_raw;
		miLectura->presion = pres_raw;
		miLectura->humedad = hum_raw;
	}


	miLectura->alebeo = alebeo_sum / (float)contador_MEMS;
	miLectura->cabeceo = cabeceo_sum / (float)contador_MEMS;
	miLectura->guino_brujula = guino_sum / (float)contador_MEMS;

	alebeo_sum = 0.0f;  cabeceo_sum = 0.0f;   guino_sum = 0.0f; //Reseteo de acumuladores de medias parciales


	miLectura->ubicacion_fix = decodificadorNMEA(buffc_DMA_UART, &latit_raw, &longit_raw, &altit_raw, &speed_raw);

	if(miLectura->ubicacion_fix || ((noesNAN(latit_raw) && noesNAN(longit_raw))) )	//comprobación errores
	{
		if(noesNAN(latit_raw) && latit_raw!= 0.0f) miLectura->latitud = latit_raw;
		if(noesNAN(longit_raw) && longit_raw!=0.0f) miLectura->longitud = longit_raw;
		if(noesNAN(altit_raw) && altit_raw!=0.0f) miLectura->altitud= altit_raw;
		if(noesNAN(speed_raw)) miLectura->velocidad = speed_raw;
	}

	mideRadiacion(miLectura->irradiancia);	//llamada a función a parte para las irradiancias


#ifdef ENABLE_IMPRIMIR_MUESTRAS	//imprime cada muestra si el usuario compilador lo desea

		printf("\x1b[2J" "\x1b[f"); //limpiar buffer y ventana de TeraTerm

	    printf("\n\t-------------- Datos Leidos por el uC STM32-L475-VGT6 ----------------\n"
	    		"Irradiancia modulo FV 1:   %f\n"
	    		"Irradiancia modulo FV 2:   %f\n"
	    		"Irradiancia modulo FV 3:   %f\n"
	    		"Irradiancia modulo FV 4:   %f\n"
	    		"Irradiancia modulo FV 5:   %f\n"
	    		"Temperatura interior del sensor:  %f\n"
	    		"Presion interior del sensor:      %f\n"
	    		"Humedad interior del sensor:      %f\n"
	    		"Latitud geografica         :      %f\n"
	    		"Longitud geografica        :      %f\n"
	    		"Altitud geografica         :      %f\n"
	    		"Velocidad desplazamiento   :      %f\n"
	    		"Alabeo    X :                     %f\n"
	    		"Cabeceo   Y :                     %f\n"
	    		"Gui%cada   Z :                     %f\n"
	    		"Fecha y hora de la medicion:      %02d-%02d-%04d  %02d:%02d:%02d \n",
				miLectura->irradiancia[0], miLectura->irradiancia[1], miLectura->irradiancia[2], miLectura->irradiancia[3], miLectura->irradiancia[4],
				miLectura->temperatura, miLectura->presion,miLectura->humedad,
				miLectura->latitud, miLectura->longitud, miLectura->altitud, miLectura->velocidad,
				miLectura->alebeo, miLectura->cabeceo, 165, miLectura->guino_brujula,
				miLectura->dia , miLectura->mes,  miLectura->agno , miLectura->hora , miLectura->min, miLectura->seg
	    		);
#endif

		//HAL_SuspendTick();
		//HAL_GPIO_WritePin(GPIOC, ARD_A2_LEDON_Pin, GPIO_PIN_RESET); //indicador visual

}


/**
 * @brief   Funcion para medir los datos de irradiancia y realizar la secuencia de conmutaciones de éstos.
 * Puede también tomar una muestra de la corriente que llega al BMS para recargar el sensor una vez termina
 * la secuencia.
 * @param   vectIrradiancia:   dirección de memoria (vector) para devolver todas las irradiancias medidas
 * @retval  void no devuelve nada
 */
void mideRadiacion(float vectIrradiancia[])
{
	float nivelmedioADC = 0.0f,  offset_ADC = 0.0f; //nivel de Offset medido en el ADC

	/*Estado de partida*/
	  HAL_GPIO_WritePin(ARD_D10_MFT_GPIO_Port, ARD_D10_MFT_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin|ARD_D7_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin|ARD_D8_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

	for(uint8_t npv = 0 ; npv<= NMAX_MODULOS; npv++)
	{
		float tensionADC = 0.0f, corrienteFV = 0.0f, irradianciaFV = 0.0f;
		float suma_media = 0.0f;
		uint64_t contador = 0,  mseg ;

		switch (npv)		//activa y desactiva los GPIO de los interruptores oportunos
		{
			case 0:		//para medir el nivel del offset
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_RESET);
			break;
			case 1:  //modulo FV 1
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_RESET);
			break;

			case 2:  //modulo FV 2
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_RESET);
			break;

			case 3:	//modulo FV 3
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_RESET);
			break;

			case 4:	//modulo FV 4
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_RESET);
			break;

			case 5: 	//modulo FV 5
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_SET);
			break;

			default:	//en caso de erro, todo a 1
				 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_SET);
			break;
		}

		HAL_Delay(T_ESPERA);	//5 milisegundo de Delay entre medidas, para conmutar los interruptores y estabilizar medidas del ADC.


		mseg = HAL_GetTick();

		do{	// medidas medias redundantes del ADC además del oversampling.
			suma_media += ADC1_buffer;
			contador++;
		}while (HAL_GetTick() - mseg < T_MEDICION);

		nivelmedioADC =  (float) suma_media / contador;

		if( npv == 0 ) {
			offset_ADC =  nivelmedioADC;
		}
		else{

		tensionADC = ( (float) abs(nivelmedioADC - offset_ADC)/NIVELES_ADC ) * VREF_ADC;  //en miliVoltios

		corrienteFV =  tensionADC /  (SENS_HALL * FACTOR_OPAMP);	//en mA

		irradianciaFV = corrienteFV *  CTE_CALIBR_FV[npv-1] ;	//en W

		vectIrradiancia[npv-1] = irradianciaFV;
		}
	}

	 HAL_GPIO_WritePin(GPIOA, ARD_D4_Pin, GPIO_PIN_SET);		//estado de Energy Harvesting de los módulos FV, recargando la bateria
	 HAL_GPIO_WritePin(GPIOB, ARD_D5_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, ARD_D6_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, ARD_D7_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin, GPIO_PIN_SET);

	 HAL_GPIO_WritePin(ARD_D10_MFT_GPIO_Port, ARD_D10_MFT_Pin, GPIO_PIN_RESET);

}



/*
  * @brief Función de intento de reconexión a la red Wi-Fi preconfigurada. Si se produce algún error, o se supera
  * un timeout de 3200 mseg en la busqueda de dicha red, se retornará falso, se lo contario, se intentará establecer conexión
  * @param None
  * @retval exito en la reconexión
  */
bool reconecta_WiFi(void)  {

	msg_info("\n\nEl enlace con la red WiFi %s caido. Tratando de reconectar...\n", (g_connection_needed_score > MAX_SOCKET_ERRORS_BEFORE_NETIF_RESET) ? "puede estar" : "esta");

	int ret = net_reinit(hnet, (net_if_reinit));

	if ( ret != 0 )
	{
	  msg_error("\nLa reconexion con la red ha fracasado.\n");
	  HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_RESET);	//LED de conexión Wi-Fi}
	  return false;
	}
	else
	{
	  msg_info("\nLa reconexion con la red WiFi ha sido EXITOSA.\n");
	  g_connection_needed_score = 1;
	  HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_SET);	//LED de conexión Wi-Fi}
	  return true;
	}
}


/*
  * @brief Función simple de impresión por puerto UART1 al ordenador un determinado mensaje con el Dato recabado
  * @param Estrura con el macroDato recibido
  * @retval None
  */
void imprimir_Dato(megaDato Dato)  {

	printf("\x1b[2J" "\x1b[f"); //limpiar buffer y ventana de TeraTerm

    printf("\n************************ DATOS PUBLICADOS EN LA NUBE DE THINGSPEAK ************************\n"
    		"-------------------------------- CANAL 1 ---------------------------------\n"
   	    		"Campo 1: Irradiancia modulo FV 1:          %f W\n"
   	    		"Campo 2: Irradiancia modulo FV 2:          %f W\n"
   	    		"Campo 3: Irradiancia modulo FV 3:          %f W\n"
   	    		"Campo 4: Irradiancia modulo FV 4:          %f W\n"
   	    		"Campo 5: Irradiancia modulo FV 5:          %f W\n"
	    		"Campo 6: Temperatura interior del sensor:  %f %cC\n"
	    		"Campo 7: Presion interior del sensor:      %f mbar\n"
	    		"Campo 8: Humedad interior del sensor:      %f %%\n"

    		"-------------------------------- CANAL 2 ---------------------------------\n"
				"Campo 1: Latitud geografica   :            %f %c\n"
				"Campo 2: Longitud geografica  :            %f %c\n"
				"Campo 3: Altitud geografica   :            %f m\n"
    			"Campo 4: Velocidad media      :            %f km/h\n"
    			"Campo 5: Incliacion Alebeo  X :            %f %c\n"
    			"Campo 6: Incliacion Cabeceo Y :            %f %c\n"
				"Campo 7: Orientacion Norte  Z :            %f %c\n"

   	    		"  Fecha y hora de la medicion :        %02d-%02d-%04d   %02d:%02d:%02d \n\n",
				Dato.irradiancia[0], Dato.irradiancia[1], Dato.irradiancia[2], Dato.irradiancia[3], Dato.irradiancia[4],
   				Dato.temperatura, SUPER_O, Dato.presion,Dato.humedad,
				Dato.latitud, SUPER_O, Dato.longitud, SUPER_O, Dato.altitud, Dato.velocidad,
				Dato.alebeo, SUPER_O, Dato.cabeceo, SUPER_O, Dato.guino_brujula, SUPER_O,
   				Dato.dia , Dato.mes,  Dato.agno , Dato.hora , Dato.min, Dato.seg
   	    		);
}


/**
 * @brief   Rutina que implementa la  ejecución del algoritmo de estimación de la posición del MEMS de la placa.
 * Se implementa en una función a parte de l de lectura por necesitar una frecuencia de iteración muy superior a la
 * de lectura. 	Los valores devueltos son añadidos a un sumador para que posteriormente la funcion de lectura calcule
 * la media de todos elos cada segundo.
 * @param   void
 * @retval  void
 */
void computa_algoritmoMEMS(void)
{

	float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

#ifdef ENABLE_LOWPWR
	if(modo_BajoConsumo) {  salir_LowPowerMode();  }  //saliendo del modo de bajo consumo
#endif

	 MX_MEMS_Process(&roll, &pitch, &yaw);	//función de computo
	 flag_lecturaMEMS = false;
	 alebeo_sum += roll;
	 cabeceo_sum += pitch;
	 guino_sum +=  yaw;
	 contador_MEMS ++;

}


/**
 * @brief   Funcion que deshace la conexión MQTT, abre el socket de la red. Sirve para rehacer posteriormente
 * la conexión con el servidor, ya que es la unica forma de limpiar una conexión MQTT perdida sin inducir a
 * errores. Realiza además comprobación de errores. Suele ser invocada ante caidas de conexión a la red
 * @param   void
 * @retval  bool: false si existen errores en la desconexión o true si ha sido exitosa
 */
bool desconectaConexionMQTT(void)  {

	int ret = 0;

	 if (b_mqtt_connected == true)
		{
		  ret = MQTTDisconnect(&client);
		  if (ret != MQSUCCESS)
		  {
			msg_error("\nMQTTDisconnect() fallo.\n");
		  }
		  b_mqtt_connected = false;
		}
	  if (NET_OK !=  net_sock_close(socket))
		{
		  msg_error("\nnet_sock_close() fallo.\n");
		}

	  if (NET_OK != net_sock_destroy(socket))
	  {
		msg_error("\nnet_sock_destroy() fallo.\n");
	  }
	return ret;
}


/**
 * @brief   Funcion que asegura los protocolos y niveles de seguidad en la conexion con el socket.
 * Además, realiza comprobaciones con el certificado TSL de seguridad provisto por STM en las liberías
 * de X-CLD-GEN. Al igual que el resto de funciones, lleva a cabo comprobación de errores para avisar al usuario
 * @param   void
 * @retval  int: enumeación de tipo de error cometido, 0 si ninguno
 */
int check_protocoloConexion(void)  {

    int ret = net_sock_create(hnet, &socket, (connection_security == CONN_SEC_NONE) ? NET_PROTO_TCP :NET_PROTO_TLS);

    if (ret != NET_OK)
    {
      msg_error("\nNo se pudo crear el socket.\n");
    }
    else
    {
      switch(connection_security)
      {
        case CONN_SEC_MUTUALAUTH:
          ret |= ((checkTLSRootCA() != 0) && (checkTLSDeviceConfig() != 0) )
            || (getTLSKeys(&ca_cert, &device_cert, &device_key) != 0);
          ret |= net_sock_setopt(socket, "tls_server_name", (void *) device_config->HostName, strlen(device_config->HostName) + 1);
          ret |= net_sock_setopt(socket, "tls_ca_certs",    (void *) ca_cert,                 strlen(ca_cert) + 1);
          ret |= net_sock_setopt(socket, "tls_dev_cert",    (void *) device_cert,             strlen(device_cert) + 1);
          ret |= net_sock_setopt(socket, "tls_dev_key",     (void *) device_key,              strlen(device_key) + 1);
          break;
        case CONN_SEC_SERVERNOAUTH:
          ret |= net_sock_setopt(socket, "tls_server_noverification", NULL, 0);
          ret |= (checkTLSRootCA() != 0)
            || (getTLSKeys(&ca_cert, NULL, NULL) != 0);
          ret |= net_sock_setopt(socket, "tls_server_name", (void *) device_config->HostName, strlen(device_config->HostName) + 1);
          ret |= net_sock_setopt(socket, "tls_ca_certs",    (void *) ca_cert,                 strlen(ca_cert) + 1);
            break;
        case CONN_SEC_SERVERAUTH:
          ret |= (checkTLSRootCA() != 0)
            || (getTLSKeys(&ca_cert, NULL, NULL) != 0);
          ret |= net_sock_setopt(socket, "tls_server_name", (void *) device_config->HostName, strlen(device_config->HostName) + 1);
          ret |= net_sock_setopt(socket, "tls_ca_certs",    (void *) ca_cert,                 strlen(ca_cert) + 1);
          break;
        case CONN_SEC_NONE:
          break;
        default:
          msg_error("\nModo de seguridad en la conectividad invalido. - %d\n", connection_security);
      }
      ret |= net_sock_setopt(socket, "sock_noblocking", NULL, 0);
    }

    if (ret != NET_OK)
    {
      msg_error("\nNo se pudo obtener la configuracion de la seguridad en la conexion y fijar las opciones de socket.\n");
    }else
    {
      ret = net_sock_open(socket, device_config->HostName, atoi(device_config->HostPort), 0);
    }

    return ret;
}

/**
 * @brief   Funcion de inicialización de conexión MQTT con el servidor IoT en la nube. Lee desde memoria flash
 * la configuración preestablecida de conexión MQTT. Al igual que el resto de funciones, lleva a cabo
 * comprobación de errores para avisar al usuario. En caso de que el broker admita laconexión MQTT con el cliente
 * establecido, se devuelve true. En caso de algún error de conexión o credenciales, false.
 * @param   int: paramento de comprobación de errores previos
 * @retval  bool: true si no existen errores de ningún tipo en las conexiones, false en caso contrario.
 */
bool inicia_ClienteMQTT(int ret)	{

    if (ret != NET_OK)	//Error al abrir el Shocket en el servidor IoT que le hemos facilitado
    {
      msg_error("\nNo se pudo abrir un socket en la direccion %s  con puerto %d.\n", device_config->HostName, atoi(device_config->HostPort));
      g_connection_needed_score++;
      return false;
    }

    else{	//conexión cliente servidor con el Broker de ThingSpeak

      network.my_socket = socket;
      network.mqttread = (network_read);
      network.mqttwrite = (network_write);

      MQTTClientInit(&client, &network, MQTT_CMD_TIMEOUT, mqtt_send_buffer, MQTT_SEND_BUFFER_SIZE, mqtt_read_buffer, 1);

      /* MQTT connect */
      MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
      options.clientID.cstring = device_config->MQClientId;
      options.username.cstring = device_config->MQUserName;
      options.password.cstring = device_config->MQUserPwd;

      ret = MQTTConnect(&client, &options);
      if (ret != 0)		//Chequea que la conexión MQTT es buena
      {
        msg_error("\nMQTTConnect() fallo: %d\n", ret);
      }
      else				//Chequea que la conexión con el campo MQTT es buena
      {
        g_connection_needed_score = 0;
        b_mqtt_connected = true;
        ret = MQTTYield(&client, 500);
      }
      if (ret != MQSUCCESS)
      {
        msg_error("\nFallo del Yield.\n");
        return false;
      }

      else{
       return true;
      }
	}

}


/**
 * @brief   Funcion de inicio de conexión con la plataforma, la red Wi-Fi el módulo y su correspodiente puerto SPI3.
 *  A continuación, identifica la red a la que se conecta, al igual que el dispositivo cliente, obteniendo su
 *  ID y dirección MAC.Lee desde memoria flash la configuración preestablecida de  autentificación con la red W-Fi.
 *   Al igual que el resto de funciones, lleva a cabo comprobación de errores para avisar al usuario
 * @param   void
 * @retval  bool: true si no existen errores de ningún tipo en las conexiones, false en caso contrario.
 */
bool inicializa_ConexionIoT(void)   {

	int ret = 0;
	const char * connectionString   = NULL;

    ret = platform_init();

	  if (ret != 0)
	  {
	    msg_error("\nFallo al inicializar la plataforma de conexión a Internet.\n");
	    HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_RESET);	//LED de conexión Wi-Fi}
	  }
	  else
	  {
	    ret = (getIoTDeviceConfig(&connectionString) != 0);
	    ret |= (parse_and_fill_device_config(&device_config, connectionString) != 0);

	    connection_security = (conn_sec_t) atoi(device_config->ConnSecurity);
	    HAL_GPIO_WritePin(GPIOC, ARD_A1_LEDWIFI_Pin, GPIO_PIN_SET);	//LED de conexión Wi-Fi}
	  }

	  if (ret != 0)
	  {
	    msg_error("\nNo se pudo recuperar la cadena de conexion de la configuracion del usuario desde el almacenamiento.\n");
	  }
	  else
	  {
	    net_macaddr_t mac = { 0 };
	    if (net_get_mac_address(hnet, &mac) == NET_OK)
	    {
	      snprintf(pub_data.mac, MODEL_MAC_SIZE , "%02X%02X%02X%02X%02X%02X",
	               mac.mac[0], mac.mac[1], mac.mac[2], mac.mac[3], mac.mac[4], mac.mac[5]);
	    }
	    else
	    {
	      msg_warning("\nNo se pudo recuperar la direccion MAC pare establecer la ID del dispositivo \n");
	      snprintf(pub_data.mac, MODEL_MAC_SIZE -1 , "UnknownMAC");
	    }
	  }
	  return (ret == 0);
}

void switch_Temporizadores(bool estado)
{
	if(estado == ENCENDER_TIMERS) {

  	  if ( HAL_LPTIM_TimeOut_Start_IT(&hlptim1, PERIODO_LPTIM, TIMEOUT_LPTIM1) != HAL_OK)	//Puesta a punto de temporizadores
  	  	  { Error_Handler(); }
  	  if ( HAL_LPTIM_TimeOut_Start_IT(&hlptim2, PERIODO_LPTIM, TIMEOUT_LPTIM2) != HAL_OK)
  	      { Error_Handler(); }
  	  if ( HAL_TIM_Base_Start_IT(&htim6) != HAL_OK )
  	  	  { Error_Handler(); }
	}
	if(estado == APAGAR_TIMERS) {

		  if ( HAL_LPTIM_TimeOut_Stop_IT(&hlptim1) != HAL_OK  )
		  	  { Error_Handler(); }
		  if (  HAL_LPTIM_TimeOut_Stop_IT(&hlptim2) != HAL_OK )
		  	  { Error_Handler(); }
		  if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK)
			  { Error_Handler(); }
	}
}

/**
 * @brief   Funcion de callback de las interrupciones de los TIMERS de bajo consumo. Implementa una lógica
 * de temporización de los 3 principales hilos de ejecucción mediante el uso de banderas y contadores de
 * eventos. Gracias a ello, cronometra la publicacion, recuperacion y lectura de datos.
 * @param   In:   LPTIM_HandleTypeDef *hlptim     	variable estructura del temporizador que llama la ISR
 * @retval  void
 */
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	static uint8_t contador_publi = 0, contador_reconex = 0;

	if(hlptim == &hlptim1) {	//primer temporizador de muestreo
		flag_lectura_datos = true;
	}

	if (hlptim == &hlptim2) {	//segundo temporizador de publicacion/recuperacion

		contador_publi++;

		if(contador_publi >= (PERIODO_PUBLI_DATOS/PERIODO_MIN_LPTIM2)) {	//si supera los 60/10 = 6 vueltas
			 flag_publi_datos = true;
			 contador_publi = 0;
		}

		if ( estado == DESCONECTADO || estaFIFOvacia(&miFIFO)) {	//si se encuentra desconectado

			contador_reconex++;

			if (contador_reconex >= (PERIODO_RECUPERA_DATOS/PERIODO_MIN_LPTIM2) ) {
				flag_recupera_datos = true;
				contador_reconex = 0;
			}
		}
	}

}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	if(htim == &htim6) {

		flag_lecturaMEMS = true;

	}
}


/************************ (C) COPYRIGHT Sergio Vera Muñoz --- TFG 2020   --- *****END OF FILE****/
