/*
 * Copyright © 2014 Kosma Moczek <kosma@cloudyourcar.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */

#include "minmea.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <time.h>
#include "main.h"

#define boolstr(s) ((s) ? "true" : "false")

static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

uint8_t minmea_checksum(const char *sentence)
{
    // Support senteces with or without the starting dollar sign.
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

bool minmea_check(const char *sentence, bool strict)
{
    uint8_t checksum = 0x00;

    // Sequence length is limited.
    if (strlen(sentence) > MINMEA_MAX_LENGTH + 3)
        return false;

    // A valid sentence starts with "$".
    if (*sentence++ != '$')
        return false;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
        checksum ^= *sentence++;

    // If checksum is present...
    if (*sentence == '*') {
        // Extract checksum.
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        // Check for checksum mismatch.
        if (checksum != expected)
            return false;
    } else if (strict) {
        // Discard non-checksummed frames in strict mode.
        return false;
    }

    // The only stuff allowed at this point is a newline.
    if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
        return false;

    return true;
}

static inline bool minmea_isfield(char c) {
    return isprint((unsigned char) c) && c != ',' && c != '*';
}

bool minmea_scan(const char *sentence, const char *format, ...)
{
    bool result = false;
    bool optional = false;
    va_list ap;
    va_start(ap, format);

    const char *field = sentence;
#define next_field() \
    do { \
        /* Progress to the next field. */ \
        while (minmea_isfield(*sentence)) \
            sentence++; \
        /* Make sure there is a field there. */ \
        if (*sentence == ',') { \
            sentence++; \
            field = sentence; \
        } else { \
            field = NULL; \
        } \
    } while (0)

    while (*format) {
        char type = *format++;

        if (type == ';') {
            // All further fields are optional.
            optional = true;
            continue;
        }

        if (!field && !optional) {
            // Field requested but we ran out if input. Bail out.
            goto parse_error;
        }

        switch (type) {
            case 'c': { // Single character field (char).
                char value = '\0';

                if (field && minmea_isfield(*field))
                    value = *field;

                *va_arg(ap, char *) = value;
            } break;

            case 'd': { // Single character direction field (int).
                int value = 0;

                if (field && minmea_isfield(*field)) {
                    switch (*field) {
                        case 'N':
                        case 'E':
                            value = 1;
                            break;
                        case 'S':
                        case 'W':
                            value = -1;
                            break;
                        default:
                            goto parse_error;
                    }
                }

                *va_arg(ap, int *) = value;
            } break;

            case 'f': { // Fractional value with scale (struct minmea_float).
                int sign = 0;
                int_least32_t value = -1;
                int_least32_t scale = 0;

                if (field) {
                    while (minmea_isfield(*field)) {
                        if (*field == '+' && !sign && value == -1) {
                            sign = 1;
                        } else if (*field == '-' && !sign && value == -1) {
                            sign = -1;
                        } else if (isdigit((unsigned char) *field)) {
                            int digit = *field - '0';
                            if (value == -1)
                                value = 0;
                            if (value > (INT_LEAST32_MAX-digit) / 10) {
                                /* we ran out of bits, what do we do? */
                                if (scale) {
                                    /* truncate extra precision */
                                    break;
                                } else {
                                    /* integer overflow. bail out. */
                                    goto parse_error;
                                }
                            }
                            value = (10 * value) + digit;
                            if (scale)
                                scale *= 10;
                        } else if (*field == '.' && scale == 0) {
                            scale = 1;
                        } else if (*field == ' ') {
                            /* Allow spaces at the start of the field. Not NMEA
                             * conformant, but some modules do this. */
                            if (sign != 0 || value != -1 || scale != 0)
                                goto parse_error;
                        } else {
                            goto parse_error;
                        }
                        field++;
                    }
                }

                if ((sign || scale) && value == -1)
                    goto parse_error;

                if (value == -1) {
                    /* No digits were scanned. */
                    value = 0;
                    scale = 0;
                } else if (scale == 0) {
                    /* No decimal point. */
                    scale = 1;
                }
                if (sign)
                    value *= sign;

                *va_arg(ap, struct minmea_float *) = (struct minmea_float) {value, scale};
            } break;

            case 'i': { // Integer value, default 0 (int).
                int value = 0;

                if (field) {
                    char *endptr;
                    value = strtol(field, &endptr, 10);
                    if (minmea_isfield(*endptr))
                        goto parse_error;
                }

                *va_arg(ap, int *) = value;
            } break;

            case 's': { // String value (char *).
                char *buf = va_arg(ap, char *);

                if (field) {
                    while (minmea_isfield(*field))
                        *buf++ = *field++;
                }

                *buf = '\0';
            } break;

            case 't': { // NMEA talker+sentence identifier (char *).
                // This field is always mandatory.
                if (!field)
                    goto parse_error;

                if (field[0] != '$')
                    goto parse_error;
                for (int f=0; f<5; f++)
                    if (!minmea_isfield(field[1+f]))
                        goto parse_error;

                char *buf = va_arg(ap, char *);
                memcpy(buf, field+1, 5);
                buf[5] = '\0';
            } break;

            case 'D': { // Date (int, int, int), -1 if empty.
                struct minmea_date *date = va_arg(ap, struct minmea_date *);

                int d = -1, m = -1, y = -1;

                if (field && minmea_isfield(*field)) {
                    // Always six digits.
                    for (int f=0; f<6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char dArr[] = {field[0], field[1], '\0'};
                    char mArr[] = {field[2], field[3], '\0'};
                    char yArr[] = {field[4], field[5], '\0'};
                    d = strtol(dArr, NULL, 10);
                    m = strtol(mArr, NULL, 10);
                    y = strtol(yArr, NULL, 10);
                }

                date->day = d;
                date->month = m;
                date->year = y;
            } break;

            case 'T': { // Time (int, int, int, int), -1 if empty.
                struct minmea_time *time_ = va_arg(ap, struct minmea_time *);

                int h = -1, i = -1, s = -1, u = -1;

                if (field && minmea_isfield(*field)) {
                    // Minimum required: integer time.
                    for (int f=0; f<6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char hArr[] = {field[0], field[1], '\0'};
                    char iArr[] = {field[2], field[3], '\0'};
                    char sArr[] = {field[4], field[5], '\0'};
                    h = strtol(hArr, NULL, 10);
                    i = strtol(iArr, NULL, 10);
                    s = strtol(sArr, NULL, 10);
                    field += 6;

                    // Extra: fractional time. Saved as microseconds.
                    if (*field++ == '.') {
                        uint32_t value = 0;
                        uint32_t scale = 1000000LU;
                        while (isdigit((unsigned char) *field) && scale > 1) {
                            value = (value * 10) + (*field++ - '0');
                            scale /= 10;
                        }
                        u = value * scale;
                    } else {
                        u = 0;
                    }
                }

                time_->hours = h;
                time_->minutes = i;
                time_->seconds = s;
                time_->microseconds = u;
            } break;

            case '_': { // Ignore the field.
            } break;

            default: { // Unknown.
                goto parse_error;
            }
        }

        next_field();
    }

    result = true;

parse_error:
    va_end(ap);
    return result;
}

bool minmea_talker_id(char talker[3], const char *sentence)
{
    char type[6];
    if (!minmea_scan(sentence, "t", type))
        return false;

    talker[0] = type[0];
    talker[1] = type[1];
    talker[2] = '\0';

    return true;
}

enum minmea_sentence_id minmea_sentence_id(const char *sentence, bool strict)
{
    if (!minmea_check(sentence, strict))
        return MINMEA_INVALID;

    char type[6];
    if (!minmea_scan(sentence, "t", type))
        return MINMEA_INVALID;

    if (!strcmp(type+2, "RMC"))
        return MINMEA_SENTENCE_RMC;
    if (!strcmp(type+2, "GGA"))
        return MINMEA_SENTENCE_GGA;
    if (!strcmp(type+2, "GSA"))
        return MINMEA_SENTENCE_GSA;
    if (!strcmp(type+2, "GLL"))
        return MINMEA_SENTENCE_GLL;
    if (!strcmp(type+2, "GST"))
        return MINMEA_SENTENCE_GST;
    if (!strcmp(type+2, "GSV"))
        return MINMEA_SENTENCE_GSV;
    if (!strcmp(type+2, "VTG"))
        return MINMEA_SENTENCE_VTG;
    if (!strcmp(type+2, "ZDA"))
        return MINMEA_SENTENCE_ZDA;

    return MINMEA_UNKNOWN;
}

bool minmea_parse_rmc(struct minmea_sentence_rmc *frame, const char *sentence)
{
    // $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
    char type[6];
    char validity;
    int latitude_direction;
    int longitude_direction;
    int variation_direction;
    if (!minmea_scan(sentence, "tTcfdfdffDfd",
            type,
            &frame->time,
            &validity,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->speed,
            &frame->course,
            &frame->date,
            &frame->variation, &variation_direction))
        return false;
    if (strcmp(type+2, "RMC"))
        return false;

    frame->valid = (validity == 'A');
    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;
    frame->variation.value *= variation_direction;

    return true;
}

bool minmea_parse_gga(struct minmea_sentence_gga *frame, const char *sentence)
{
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!minmea_scan(sentence, "tTfdfdiiffcfcf_",
            type,
            &frame->time,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->fix_quality,
            &frame->satellites_tracked,
            &frame->hdop,
            &frame->altitude, &frame->altitude_units,
            &frame->height, &frame->height_units,
            &frame->dgps_age))
        return false;
    if (strcmp(type+2, "GGA"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}

bool minmea_parse_gsa(struct minmea_sentence_gsa *frame, const char *sentence)
{
    // $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
    char type[6];

    if (!minmea_scan(sentence, "tciiiiiiiiiiiiifff",
            type,
            &frame->mode,
            &frame->fix_type,
            &frame->sats[0],
            &frame->sats[1],
            &frame->sats[2],
            &frame->sats[3],
            &frame->sats[4],
            &frame->sats[5],
            &frame->sats[6],
            &frame->sats[7],
            &frame->sats[8],
            &frame->sats[9],
            &frame->sats[10],
            &frame->sats[11],
            &frame->pdop,
            &frame->hdop,
            &frame->vdop))
        return false;
    if (strcmp(type+2, "GSA"))
        return false;

    return true;
}

bool minmea_parse_gll(struct minmea_sentence_gll *frame, const char *sentence)
{
    // $GPGLL,3723.2475,N,12158.3416,W,161229.487,A,A*41$;
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!minmea_scan(sentence, "tfdfdTc;c",
            type,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->time,
            &frame->status,
            &frame->mode))
        return false;
    if (strcmp(type+2, "GLL"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}

bool minmea_parse_gst(struct minmea_sentence_gst *frame, const char *sentence)
{
    // $GPGST,024603.00,3.2,6.6,4.7,47.3,5.8,5.6,22.0*58
    char type[6];

    if (!minmea_scan(sentence, "tTfffffff",
            type,
            &frame->time,
            &frame->rms_deviation,
            &frame->semi_major_deviation,
            &frame->semi_minor_deviation,
            &frame->semi_major_orientation,
            &frame->latitude_error_deviation,
            &frame->longitude_error_deviation,
            &frame->altitude_error_deviation))
        return false;
    if (strcmp(type+2, "GST"))
        return false;

    return true;
}

bool minmea_parse_gsv(struct minmea_sentence_gsv *frame, const char *sentence)
{
    // $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    // $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
    // $GPGSV,4,2,11,08,51,203,30,09,45,215,28*75
    // $GPGSV,4,4,13,39,31,170,27*40
    // $GPGSV,4,4,13*7B
    char type[6];

    if (!minmea_scan(sentence, "tiii;iiiiiiiiiiiiiiii",
            type,
            &frame->total_msgs,
            &frame->msg_nr,
            &frame->total_sats,
            &frame->sats[0].nr,
            &frame->sats[0].elevation,
            &frame->sats[0].azimuth,
            &frame->sats[0].snr,
            &frame->sats[1].nr,
            &frame->sats[1].elevation,
            &frame->sats[1].azimuth,
            &frame->sats[1].snr,
            &frame->sats[2].nr,
            &frame->sats[2].elevation,
            &frame->sats[2].azimuth,
            &frame->sats[2].snr,
            &frame->sats[3].nr,
            &frame->sats[3].elevation,
            &frame->sats[3].azimuth,
            &frame->sats[3].snr
            )) {
        return false;
    }
    if (strcmp(type+2, "GSV"))
        return false;

    return true;
}

bool minmea_parse_vtg(struct minmea_sentence_vtg *frame, const char *sentence)
{
    // $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    // $GPVTG,156.1,T,140.9,M,0.0,N,0.0,K*41
    // $GPVTG,096.5,T,083.5,M,0.0,N,0.0,K,D*22
    // $GPVTG,188.36,T,,M,0.820,N,1.519,K,A*3F
    char type[6];
    char c_true, c_magnetic, c_knots, c_kph, c_faa_mode;

    if (!minmea_scan(sentence, "tfcfcfcfc;c",
            type,
            &frame->true_track_degrees,
            &c_true,
            &frame->magnetic_track_degrees,
            &c_magnetic,
            &frame->speed_knots,
            &c_knots,
            &frame->speed_kph,
            &c_kph,
            &c_faa_mode))
        return false;
    if (strcmp(type+2, "VTG"))
        return false;
    // check chars
    if (c_true != 'T' ||
        c_magnetic != 'M' ||
        c_knots != 'N' ||
        c_kph != 'K')
        return false;
    frame->faa_mode = (enum minmea_faa_mode)c_faa_mode;

    return true;
}

bool minmea_parse_zda(struct minmea_sentence_zda *frame, const char *sentence)
{
  // $GPZDA,201530.00,04,07,2002,00,00*60
  char type[6];

  if(!minmea_scan(sentence, "tTiiiii",
          type,
          &frame->time,
          &frame->date.day,
          &frame->date.month,
          &frame->date.year,
          &frame->hour_offset,
          &frame->minute_offset))
      return false;
  if (strcmp(type+2, "ZDA"))
      return false;

  // check offsets
  if (abs(frame->hour_offset) > 13 ||
      frame->minute_offset > 59 ||
      frame->minute_offset < 0)
      return false;

  return true;
}

int minmea_gettime(struct timespec *ts, const struct minmea_date *date, const struct minmea_time *time_)
{
    if (date->year == -1 || time_->hours == -1)
        return -1;

    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    if (date->year < 80) {
        tm.tm_year = 2000 + date->year - 1900;  // 2000-2079
    } else if (date->year >= 1900) {
        tm.tm_year = date->year - 1900; // 4 digit year, use directly
    } else {
        tm.tm_year = date->year;    // 1980-1999
    }
    tm.tm_mon = date->month - 1;
    tm.tm_mday = date->day;
    tm.tm_hour = time_->hours;
    tm.tm_min = time_->minutes;
    tm.tm_sec = time_->seconds;

    //time_t timestamp = timegm(&tm); /* See README.md if your system lacks timegm(). */
    time_t timestamp = mktime(&tm); /* Parece que sí es el caso, ha habido que tirar de README */

    if (timestamp != (time_t)-1) {
        ts->tv_sec = timestamp;
        ts->tv_nsec = time_->microseconds * 1000;
        return 0;
    } else {
        return -1;
    }
}


/**
  * @brief  Función de segmentación de cadena de caracteres en el buffer UART4. La segmentación se lleva a cabo
  * comparando trozos de cadena con las cabeceras de las sentencias NMEA, y la psoterior copia de estas.
  * @param  char* src =  Buffer circular del puerto UART4 en bruto, actualizado por DMA2, puntero a ella.
  * @param size_t old_posi,size_t posi   punteros del DMA para captar el mensaje completo NMEA.
  * @param sentenciaNMEA mifraseNMEA[] vector con cada una de las frases NMEA a devolver correctamente segmentadas
  * @retval Nº de frases segmentadas correctamente por la función
  */
uint8_t segmentarNMEA(char* src,  size_t old_posi,size_t posi,  sentenciaNMEA mifraseNMEA[])
{
  char* posicion_ini[2] ={NULL, NULL};
  char indicador[TAM_INDICADOR];
  uint16_t long_frase =0;
  uint16_t long_buff =0;

  uint8_t nfrases_correct = 0; //para comprobar si las cabeceras de las funciones son correctas

  size_t tam = (posi>old_posi)? (posi-old_posi) : (TAM_BUFNMEA-old_posi+posi ) ;
	if(posi-old_posi==0) tam=MINMEA_MAX_LENGTH*NMAX_FRASES;

  char* dest = (char*) malloc( tam * sizeof(char) + 1 );

  //Primero de todo, segmenta el Buffer para extraer el mensaje en funcion de los punteros old_pos y pos

  if (posi != old_posi) {                       /* Check change in received data */

      if (posi > old_posi) {                    /* Current position is over previous one */
      	//modo normal, lineal del buffer
      		strncpy(dest, src+old_posi, posi - old_posi);		// hay que hacerlo obligatoriamente con punteros a char

      } else {   //modo desbordado del buffer
      	//copia hasta el final del buffer
      		strncpy(dest, src+old_posi, TAM_BUFNMEA-old_posi );

          //hasta el final del buffer
          if (posi > 0) {
          	strncpy(dest + TAM_BUFNMEA - old_posi + 1, src+0, posi);
          }
      }
  }
  else
  {
	  strncpy(dest, src, MINMEA_MAX_LENGTH*NMAX_FRASES );
  }
  if (old_posi == TAM_BUFNMEA) {  //Por desbordamiento
      old_posi = 0;
  }

  //Una vez se tiene el mensaje NMEA completo, se segmenta

  posicion_ini[0] = strchr(dest,'$');	//Busca posicion de la priera frase que comienza por '$'
  posicion_ini[1] = posicion_ini[0]+1;	//y su contiguo

  if ( posicion_ini[0] != NULL )
  {
	uint8_t n_frase =0;

   do{

	   posicion_ini[1] = strchr(posicion_ini[0]+1,'$');  //Busca la siguiente posicion de la priera frase que comienza por '$'
	   	   if( posicion_ini[1] != NULL)   long_frase = posicion_ini[1] - posicion_ini[0];
	   	   else   long_frase = strchr(posicion_ini[0],'\0') - posicion_ini[0];

	   	   long_buff += long_frase;

	   		  if(long_frase>=TAM_INDICADOR && long_frase<MINMEA_MAX_LENGTH)   //si la longitud de la frase NMEA es correcta
	   		  {
	   			  strncpy(indicador, posicion_ini[0], TAM_INDICADOR);
	   			  if( strncmp(indicador,"$GPRMC", TAM_INDICADOR)==0  || strncmp(indicador,"$GPGSV",TAM_INDICADOR)==0 || strncmp(indicador,"$GPGGA", TAM_INDICADOR)==0 || strncmp(indicador,"$GPGSA", TAM_INDICADOR)==0)
	   			  {	//copia de la cadena buscada
	   				  mifraseNMEA[nfrases_correct].cadena = posicion_ini[0];
	   				  mifraseNMEA[nfrases_correct].longitud = long_frase;
	   				  mifraseNMEA[nfrases_correct].correcta = true;
	   				  nfrases_correct++;
	   			  }
	   			  else {	//nada
	    	  	    	  mifraseNMEA[nfrases_correct].correcta = false;
	   			  }

	   		  	  }

   posicion_ini[0] = posicion_ini[1];  //desplaza los cursores en el buffer para buscar la siguiente cadena
   posicion_ini[1]++;
   n_frase++;

   } while( ! (posicion_ini[1] == NULL || n_frase>= NMAX_FRASES || long_buff>=TAM_BUFNMEA)); // continuará busando frases NMEA mientras se agote el Buffer o haya como maximo NMAX_FRASES
   free(dest);
   return nfrases_correct;     // seria return --n_frase;
  }

  else 	{
	  free(dest);
	  return 0; // si no ha encontrado ningún $ en el buffer
  }
}

/**
  * @brief  Función completa para decodificar el mensaje NMEA cuando lo solicite el MCU. La función hace uso del resto de
  * la liberería una vez consigue segmentar las frases del mesnaje.
  * @param  char bufferNMEA[TAM_BUFNMEA]  =  Buffer circular del puerto UUART4 en bruto, actualizado por DMA2
  *  @param float* latit, float* longit, float* altit, float* speed = direcciones de memoria de las variables donde
  *  se almacenaran los datos
  * @retval Fijación de los datos: Corrobora si los datos obtenidos son fiables en cuanto a erroes de
  * latitud, lingitud, altitud... o si son inestables debido a la falta de tiempo o de cobertura.
  */
bool decodificadorNMEA(char bufferNMEA[TAM_BUFNMEA], float* latit, float* longit, float* altit, float* speed)
{
    int ubicacion_fijada=0, n_satelites =0;	//para corrroborar la fijación de los datos

	extern  size_t old_pos, pos;  //declarados en el main
	extern UART_HandleTypeDef huart4;

	char frasesNMEA[NMAX_FRASES][MINMEA_MAX_LENGTH] = {'\0'}; //8 frases de tamaño 80, como máximo

	sentenciaNMEA pfrasesNMEA[NMAX_FRASES];  //punteros a las cadenas entrantes
	uint8_t nfrases_correct = 0;
	bool retorno = true; //valor de retorno de la función

	do {		//Semáforo para tratar condiciones de carrera entre MCU y DMA en el Buffer_UART4, prioridad al DMA
		if( HAL_OK == UART_CheckIdleState(&huart4) ) {
			nfrases_correct = segmentarNMEA(bufferNMEA, old_pos, pos, pfrasesNMEA);
			break; //salimos del bucle
		}
		else {
			HAL_Delay(IDLE_DELAY);  } //Una pequeña espera para que termine la transmisión UART
	}while( HAL_OK != UART_CheckIdleState(&huart4) );

	if( ! nfrases_correct )
	{
		//printf("ERROR de segmentacion del Buffer NMEA, no hay frases correctas a procesar\n"); //se obvia
		return false;
	}

	for(uint8_t i=0; i<nfrases_correct; i++ )
	{
		if (pfrasesNMEA[i].correcta)	//Bucle que analiza cada una de las frases segmentadas anteriormente
		{
		strncpy( frasesNMEA[i], pfrasesNMEA[i].cadena , pfrasesNMEA[i].longitud );
		}

        switch (minmea_sentence_id(frasesNMEA[i], false))
        {

            case MINMEA_SENTENCE_RMC: {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, frasesNMEA[i])) {
                           *latit =  minmea_tocoord(&frame.latitude);
                           *longit = minmea_tocoord(&frame.longitude);
                           *speed = 1.851984f * minmea_tofloat(&frame.speed);
                           /*(knots)millas nauticas por hora-> 1.851984 km/h  */
                     retorno &= true;
                }
                else {
                    //printf("$xxRMC sentence is not parsed\n");
                    retorno &= false;
                }
            } break;

            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, frasesNMEA[i])) {
                	ubicacion_fijada =  frame.fix_quality;
                	*altit = minmea_tofloat(&frame.altitude);
                    retorno &= true;
                }
                else {
                    //printf("$xxGGA sentence is not parsed\n");
                    retorno &= false;
                }
            } break;

            case MINMEA_SENTENCE_GSV: {
                struct minmea_sentence_gsv frame;
                if (minmea_parse_gsv(&frame, frasesNMEA[i])) {
                    n_satelites = frame.total_sats;
                    retorno &= true;
                }
                else {
                   // printf("$xxGSV sentence is not parsed\n");
                    retorno &= false;
                }
            } break;

            case MINMEA_SENTENCE_GSA: {
                //printf("$xxGSA sentence is not used\n");	//bueno no pasa nada por no avisar, se obvia
                retorno &= true;
            } break;

            case MINMEA_INVALID: {
                //printf("$xxxxx sentence is not valid\n");
                retorno &= false;
            } break;

            default: {
                //printf("$xxxxx sentence is not parsed\n");
                retorno &= false;
            } break;
        }
	}		//fin del for de las frases
	//fin de la función

	/*  Condiciones de fijación de los datos, arbitrarias en base a pruebas realizadas  */
	if(n_satelites>=7 && ubicacion_fijada>0)
		return true;
	else
		return (retorno | ubicacion_fijada);			//si todo ha ido bien, devolverá true

  }
