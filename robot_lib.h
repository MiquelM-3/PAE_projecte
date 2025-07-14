/*
 * robot_lib.h
 *
 *  Created on: 24 Apr 2025
 *      Author: super
 */
#include "msp.h"

#ifndef ROBOT_LIB_H
#define ROBOT_LIB_H


//Structura de packet de retorn. Te un byte de tornada (error), un de timeout (per saber si sha quedat penada
//la transmissio, i despres el statusPacket que es el que ens envia la UART
typedef struct
{
    uint8_t StatusPacket[32];
    uint8_t error;
    uint8_t byteTimeOut;
} RxReturn ;

typedef uint8_t byte;

//VARIABES COMPARTIDES

//referencia es la referencia per on tindrem la paret. Ref = 0 vol dir que resseguim per l'esquerra, i ref = 1 que resseguim dreta
extern byte referencia;
//caminant es una variable que indica si hem aturat o no el motor (per exemple, per un input de l'usuari)
extern byte caminant;
//backflip indicara 0 si no hem de fer un gir de 90 graus i 1 si l'hem de fer
extern byte backflip;

//setejar la velocitat del motor
void set_velocitat(uint16_t velocitat);

// Moviments (bastant explicatius)
void moure_endavant(void);
void moure_enrere(void);
void girar_esquerra(void);
void girar_dreta(void);
void aturar(void);

// Sensors
RxReturn llegir_sensor(uint8_t id_sensor);

//Inicialitzacio de timers, GPIO's i UART
void init_UART(void);
void init_timer_A0(void);
void init_timer_A1(void);
void init_joystick(void);
void init_botton(void);
void init_buzzer(void);

//Funcions del brunzidor
void tocar_himne_FCB(void);

//Funcions de temps
void Reset_gir_temps(void);
byte control_gir_temps(int segons);

//Comunicació amb el robot
RxReturn ComunicationPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]);

#endif /* ROBOT_LIB_H */
