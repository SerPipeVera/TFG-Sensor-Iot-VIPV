/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : FIFO.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 Sergio Vera
  *
  * This software component is licensed by Sergio VeraT under free license
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensors_data.h"
#include <stdlib.h>
#include <stdio.h>


/*-----------------Estructuras de la lista dinamica, comportamiento FIFO: First Input First Output------------------*/

typedef struct elemento
{
	int indice ;
	megaDato Dato;
	struct elemento *siguiente;
}nodo;

typedef struct listaenlazada
{
	nodo* cabecera ;   //Primer elemento de la lista enlazada
	int tam_fifo ;
}fifo;


/*----------------Declaraciones de las funciones para manejar la lista dinamica-----------------------*/

nodo* crearNodo(megaDato miDato);
void liberarNodo(nodo* miNodo);
bool insertarFIFO(fifo* mififo, megaDato miDato );
megaDato* obtenerDatoFIFO(fifo* mififo);
bool eliminarDatoFIFO(fifo* mififo) ;
int estaFIFOvacia(fifo* mififo);


/*----------------Definciciones de las funciones para manejar la lista dinamica-----------------------*/


/* A no usar por el usuario, invocar a la funcion insertarFIFO()*/
nodo* crearNodo(megaDato miDato)  {
	nodo* nuevo = NULL;

	nuevo = (nodo*) malloc( sizeof(nodo) );

		 if( nuevo == NULL) {
			 free(nuevo);
			 return NULL;
		 }


		 nuevo->Dato = miDato;	//esta es una de las maravillas que permite C, copiado directo de estructuras
		 nuevo->siguiente = NULL;	//será el ultimo de la lista

		return nuevo;
}

/* A no usar por el usuario, invocar a la funcion eliminaDatoFIFO()*/
void liberarNodo(nodo* miNodo) {

	free(miNodo);
}

/* Crea un nodo al final de la lista enlazada y lo rellena con el dato,
 *  a invocar como primera funcion*/
bool insertarFIFO(fifo* mififo, megaDato miDato )  {

	nodo* miNodo = crearNodo(miDato);
	int contador = 0;

	if (miNodo == NULL){
		printf("Fallo al intertar insertar el nodo en la FIFO. Memoria Heap dinamica insuficiente, se perdera el dato. \n");
		return false;
	}
	mififo->tam_fifo ++;

	if(mififo->cabecera == NULL) {  //si la lista está vacía, se pone como cabeza al nodo cabecera
		mififo->cabecera = miNodo;

	}
	else {
		nodo* puntero = mififo->cabecera;

		while(puntero->siguiente != NULL) //recorremos la lista enlazada
		{
			puntero = puntero->siguiente; //s desplaza una posicion
			contador++;
		}

		puntero->siguiente = miNodo; //una vez esté al final, se
	}

	miNodo->indice = contador; 	//numero de elementos recorridos

	return true;	//exito
}


/*Devuelve el dato almacenado en la cabecera de la lista enlazada,
 * si está vacía, devuelve NULL. A invocar la segunda función*/
megaDato* obtenerDatoFIFO(fifo* mififo)  {

	if(mififo->cabecera == NULL) {	//si la lista esta vacía
		printf("Fallo al extraer dato de la FIFO. Lista dinamica esta vacia. \n");
		return NULL;
	}
	else {
		return &(mififo->cabecera->Dato);		//devuelve el dato
	}
}


/*Elimina el primer nodo y si dato de la lista enlazada.
 *  Si la lista está vacía, devuelve false. A invocar como última función*/
bool eliminarDatoFIFO(fifo* mififo)  {

	if(mififo->cabecera == NULL) {	//si la lista esta vacía
			printf("Error al eliminar dato de la FIFO. Lista dinamica esta vacia. \n");
			return false;
		}

	nodo* eliminar = mififo->cabecera;	//nodo auxiliar
	mififo->cabecera = mififo->cabecera->siguiente; //ahora la cabeza apunta al siguiente elemento
	liberarNodo(eliminar);	//lo libera
	mififo->tam_fifo --;
	return true;
}

/* Devuelve el numero de elementos en la FIFO*/
int estaFIFOvacia(fifo* mififo)  {

	return mififo->tam_fifo;
}

/************************ (C) COPYRIGHT  Sergio Vera TFG 2020 *****END OF FILE****/
