#include "robot_lib.h"
#include "stdio.h"
#include "lib_PAE.h"
#include "msp.h"

#define TXD2_READY (UCA2IFG & UCTXIFG) // interrupt flag reg and flag de tx

//Defines pels ports de la UART
#define Port_UARTx_SEL0 P3SEL0
#define Port_UARTx_SEL1 P3SEL1
#define DIR_UART BIT0

//Identificadors dels ids dels motors esquerra dreta i sensor
#define MOTOR_ESQUERRA_ID 2
#define MOTOR_DRETA_ID 4
#define SENSOR_ID 100

//Numero d'instruccio que enviem a la UART
#define WR_INST_BIT 3

typedef uint8_t byte;

uint8_t DatoLeido_UART;
//Time Out per rebre informacio de la UART
uint16_t contador_T1_TIMEOUT;

//Velocitat actual del robot
uint16_t velocitat_actual;

//Boolea per saber si hem rebut de la UART
byte Byte_recibido_bool;

//Timer que va contant quant de temps portem girant (o executant alguna accio, ho fem servir com a delay)
byte timer_gir;

byte referencia = 0; //Esquerra = 0; dreta = 1; deafault = 2
byte caminant = 1; //Parat = 0; caminant = 1
byte backflip = 0; //0 no gir; 1 gir



//------------------------------------------TXCPACKET--------------------------------

/*TxPacket() 3 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte. torna la mida del "Return packet"
 * Funcio:
 *
 */
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]) {

    char error[] = "adr. no permitida";
    if ((Parametros[0] < 6) && (bInstruction == 3)){//si se intenta escribir en una direccion <= 0x05,
        //emitir mensaje de error de direccion prohibida:
          halLcdPrintLine(error, 8, INVERT_TEXT);
          //y salir de la funcion sin mas:
          return 0;
    }

    byte bCount,bCheckSum,bPacketLength;
    byte TxBuffer[32];

    Sentit_Dades_Tx();                                      //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Transmetre)

    TxBuffer[0] = 0xff;                                     //Primers 2 bytes que indiquen inici de trama FF, FF.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID;                                      //ID del mòdul al que volem enviar el missatge
    TxBuffer[3] = bParameterLength+2;                       //Length(Parameter,Instruction,Checksum)
    TxBuffer[4] = bInstruction;                             //Instrucció que enviem al Mòdul

    for(bCount = 0; bCount < bParameterLength; bCount++) {  //Comencem a generar la trama que hem d’enviar
        TxBuffer[bCount+5] = Parametros[bCount];
    }

    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2; //Aixo hauria de ser +3. EL +3 fa per ID; LENGHT, INSTRUCTION (pero posem +4 per com fem els bucles)
                                          //El +2 es pels 2 byts dinici de trama

    //el bcount comença per 2 perque els dos primers bytes son d'inici de trama.
    for(bCount = 2; bCount < bPacketLength-1; bCount++) {   //Càlcul del checksum
        bCheckSum += TxBuffer[bCount];
    }

    TxBuffer[bCount] = ~bCheckSum;                          //Escriu el Checksum (complement a 1)
    for(bCount = 0; bCount < bPacketLength; bCount++) {     //Aquest bucle és el que envia la trama al Mòdul Robot
        TxUACx(TxBuffer[bCount]);
    }

    //UCBUSY es BIT0, por tanto estamos cogiendo el BIT0 del registro UCA2STATW que indica si hay una operación en marcha
    while((UCA2STATW & UCBUSY));                           //Espera fins que s’ha transmès el últim byte

    Sentit_Dades_Rx();                                      //Posem la línia de dades en Rx perquè el mòdul Dynamixel envia resposta

    return(bPacketLength);
}

//------------------------------------------------------------------------------

//-------------------------------------CANVI SENTIT COMUNICACIONS----------------------------------

//Configuració del Half Duplex dels motors: Recepció
void Sentit_Dades_Rx(void) {
    P3OUT &= ~BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 0 (Rx)
}

//Configuració del Half Duplex dels motors: Transmissió
void Sentit_Dades_Tx(void) {
    P3OUT |= BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Tx)
}

/* funció TxUACx(byte): envia un byte de dades per la UART 2 */

void TxUACx(uint8_t bTxdData) {
    while(!TXD2_READY); // Espera a que estigui preparat el buffer de transmissió
    UCA2TXBUF = bTxdData;
}

//------------------------------------------------------------------------------

//-------------------------------------RXPACKET----------------------------------
/*
Aquest exemple no és complert, en principi RxPacket() torna una estructura “Status packet" que bàsicament
consisteix en un array amb “Status Packet” + un byte indicant si hi ha un TimeOut. Això s’ha fet així perquè en C no
es pot posar un array com paràmetre de tornada.
Per altre banda, la part mostrada només llegeix els primers 4 bytes del status packet.

Això és perquè el quart byte indica precisament quants bytes queden per llegir,
el que vol dir que s’ha de fer un altre bucle “for” semblant per
llegir els bytes que falten....
Evidentment, es podria fer d’altres maneres, per exemple enviant com paràmetre el número de bytes a llegir...
*/

RxReturn RxPacket(void) {
    RxReturn respuesta; //La resposta que rebem del robot
    byte bCount, bLenght, bChecksum; //
    byte Rx_time_out=0;
    byte check_sum=0;

    respuesta.error=0;


    //volem timeout quan hi hagi 1 milisegon; la rutina dinterrupcio fa cada 10 microsegons

    Sentit_Dades_Rx();                         //Ponemos la linea half duplex en Rx;
    Activa_TimerA1_TimeOut();

    //Primer llegim 3 bytes
    for(bCount = 0; bCount < 4; bCount++) {     //bRxPacketLength; bCount++)
        Reset_Timeout(); //posa el cont a 0

        Byte_recibido_bool=0;                       //No_se_ha_recibido_Byte();

        while (!Byte_recibido_bool) {                //Se_ha_recibido_Byte())
            Rx_time_out = TimeOut(1000);          // tiempo en decenas de microsegundos
            if (Rx_time_out){
                respuesta.byteTimeOut = 1;
                break;              //sale del while
            }
        }

        if (Rx_time_out) break;                  //sale del for si ha habido Timeout

        //Si no, es que todo ha ido bien, y leemos un dato:
        respuesta.StatusPacket[bCount] = DatoLeido_UART; //Get_Byte_Leido_UART();
    }

    //Després llegim tot
    if (!Rx_time_out){
        bLenght = DatoLeido_UART; //Aquest daqui es [3], lultim que hem llegit.

        for (bCount = 0; bCount < bLenght; bCount++){
            Reset_Timeout(); //posa el cont a 0

            Byte_recibido_bool=0;                       //No_se_ha_recibido_Byte();

            while (!Byte_recibido_bool) {                //Se_ha_recibido_Byte())
                Rx_time_out = TimeOut(1000);          // tiempo en decenas de microsegundos
                if (Rx_time_out){
                    respuesta.byteTimeOut = 1;
                    break;              //sale del while
                }
            }

            if (Rx_time_out) break;                  //sale del for si ha habido Timeout

            //Si no, es que todo ha ido bien, y leemos un dato:
            //Posem un +4 perque ens estem saltant els primers 4 parametres que ya hem llegit.
            respuesta.StatusPacket[bCount+4] = DatoLeido_UART; //Get_Byte_Leido_UART();
        }

        if (!Rx_time_out){
            check_sum+= respuesta.StatusPacket[2]; //afegim el ID
            check_sum+= respuesta.StatusPacket[3]; //afegim la lenght

            for (bCount = 0; bCount < bLenght-1; bCount++){ //menys el checksum (que es lultim)
                check_sum+= respuesta.StatusPacket[bCount+4]; //afegim tots els parametres
            }
            check_sum=~check_sum;
        }

        respuesta.byteTimeOut = 0;
    }


    //Comprovem el checksum, i també si l'error no és 0 (llavors hi ha algun error).
    if(check_sum != respuesta.StatusPacket[bLenght + 4 -1] || respuesta.StatusPacket[4] != 0){
        respuesta.error = 1;
        //TODO: En un futur podriem gestionar diferents errors. Si per exemple lerror es dinvalid parameters farem una cosa
        //Si per exemple tenim el checksum malament podriem tornar a provar denviarho.
    }else{
        respuesta.error=0;
    }

    Desactiva_TimerA1_TimeOut();

    return respuesta;
}

//--------------------------------------------------------------------------------------


//------------------------------COMUNICATION PACKET-------------------------------------

//Funcio que fa un txpacket i un rxpacket. També te en compte si hi ha hagut algun error, i torna a fer la comunicacio.
RxReturn ComunicationPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]) {
    TxPacket(bID,bParameterLength,bInstruction,Parametros);
    RxReturn retornpacket= RxPacket();

    //Fem bucle fins que no hi hagi error
    while(retornpacket.error==1 || retornpacket.byteTimeOut==1){
        TxPacket(bID,bParameterLength,bInstruction,Parametros);
        retornpacket= RxPacket();
    }
    return retornpacket;
}

//-------------------------------------------------------------------------------------



//------------------------------------------MOTOR---------------------------------------

RxReturn EncendreMotor(byte motor_id) {

    //Torque enable = 1
    byte torque_enable[2] = {24, 1};
    return ComunicationPacket(motor_id, 2, WR_INST_BIT, torque_enable);

    //Configurar modo roda continua (CW y CCW angle limits a 0)
    /*
    byte cw_limit_low[2]  = {6, 0};
    byte cw_limit_high[2] = {7, 0};
    byte ccw_limit_low[2] = {8, 0};
    byte ccw_limit_high[2] = {9, 0};

    ComunicationPacket(motor_id, 2, WR_INST_BIT, cw_limit_low);
    ComunicationPacket(motor_id, 2, WR_INST_BIT, cw_limit_high);
    ComunicationPacket(motor_id, 2, WR_INST_BIT, ccw_limit_low);
    ComunicationPacket(motor_id, 2, WR_INST_BIT, ccw_limit_high);
     */
}

RxReturn AturaMotor(byte motor_id) {
    // Torque Enable = 0
    byte torque_disable[2] = {24, 0};

    return  ComunicationPacket(motor_id, 2, WR_INST_BIT, torque_disable);
}



RxReturn SetVelocitatMotor(byte motor_id, uint16_t velocitat) {
    byte speed[3] = {32, (byte) (velocitat & 0xFF), (byte) ((velocitat >> 8) & 0xFF)};   // Less significant bits

    //byte speed_high[2] = {33, (velocitat >> 8) & 0xFF};   // Most significant bits
    //ComunicationPacket(motor_id, 2, WR_INST_BIT, speed_high);

    return ComunicationPacket(motor_id, 3, WR_INST_BIT, speed);
}

//-----------------------------BUZZER CONFIG--------------------------------

void init_buzzer(void){
    // Configurar P2.7 como salida digital
    P2->SEL0 &= ~BIT7;
    P2->SEL1 &= ~BIT7;
    P2->DIR |= BIT7;
}

//-------------------------------------------------------------------------



//-------------------------------BOTO CONFIG--------------------------------

void init_botton(void){
    //Configuramos boton (5.1)
       //***************************
       P5SEL0 &= ~(BIT1);    //Els polsadors son GPIOs
       P5SEL1 &= ~(BIT1);    //Els polsadors son GPIOs

       P5DIR &= ~(BIT1);    //Un polsador es una entrada
       P5REN |= (BIT1);     //Pull-up/pull-down pel pulsador
       P5OUT |= (BIT1); //Donat que l'altra costat es GND, volem una pull-up

       P5IE |= (BIT1);      //Interrupcions activades
       P5IES &= ~(BIT1);    // amb transicio L->H
       P5IFG = 0;                  // Netegem les interrupcions anteriors

       NVIC->ICPR[1] |= 1 << (PORT5_IRQn & 31);
       NVIC->ISER[1] |= 1 << (PORT5_IRQn & 31);
}

//---------------------------------------------------------------------------



//--------------------------------TIMER CONFIG-------------------------------
/**
 * Funcio copiada del campus; inicialitza el timer.
 */
void init_timer_A0(void){

    TIMER_A0->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__UP;
    TIMER_A0->CCR[0] = (1 << 13) - 1;
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0

    NVIC->ICPR[0] |= BIT8; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT8; //y habilito las interrupciones del puerto
}

//Timer pel timeOUT
void init_timer_A1(void){

    TIMER_A1->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__UP;
    TIMER_A1->CCR[0] = 0;
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0

    NVIC->ICPR[0] |= 1 << (TA1_0_IRQn & 31); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= 1 << (TA1_0_IRQn & 31); //y habilito las interrupciones del puerto

}

void Activa_TimerA1_TimeOut(void){
    //TIMER_A1->CTL |= TACLR;
    //TIMER_A1->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__UP;
    TIMER_A1->CCR[0] =  240; //SI va a 24Mhz, fent 24Mhz / 240 = 0.1Mhz que és 10microsegons (1/0.1 = 10)
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0
}

void Desactiva_TimerA1_TimeOut(void){
    //TIMER_A1->CTL |= TACLR;
    //TIMER_A1->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__UP;
    TIMER_A1->CCR[0] = 0;
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0
}


void Reset_Timeout(void){
    contador_T1_TIMEOUT = 0;
}

byte TimeOut(int umbral){
    //contador_T1_TIMEOUT=contador_T1_TIMEOUT;
    return umbral <= contador_T1_TIMEOUT;
}

void Reset_gir_temps(void){
    timer_gir=0;
}

byte control_gir_temps(int unitat_temps){
    return unitat_temps <= timer_gir;
}

//----------------------------------------------------------------------


//----------------------------UART CONFIG--------------------------------

void init_UART(void){
    // Reg UCA2CTLW0
    // 15       14      13      12      11      10      9       8
    // Parity   Parity  MSB     lenght  stop bit UCMODE(2b)     UCSYNC
    // enable   select  1st sel 7 o 8b  1 o 2b  00= UART        async o sync(1)
    // 7        6       5       4       3       2       1       0
    // clk source(2b)   RX err  RX break Sleep  Tx addr Tx break SW reset Enabled
    // 00 UCLK...       char IE char IE 1=sleep 0=d 1=addr      UCSWRST

   UCA2CTLW0 |= UCSWRST;                // reset de la USCI

   //Pag 923 del document
   //Configurem la usci amb una variable predefinida
   //UCSYINC=0 ->indica mode asincron de configuracio
   //UCMODEx=0 ->seleccionem la UART
   //UCSPB=0 ->  1 bit de stop
   //UC7BIT=0 -> 8bits de dades de transmissio
   //UCMSB=0 -> bit de menys pes es el primer
   //UCPEN=0 -> bit de paritat NO
   //UCPAR=x -> No fem servir bit de paritat
   //clock smclk (24MHz amb la funcio que tenim)
   UCA2CTLW0 |= UCSSEL__SMCLK;

   UCA2MCTLW = UCOS16; // per fer el sobremostreig x16

   //UCA2BRW = 13;     // CONFIGURATION UART TO 115200bps (antiga)
   //UCA2MCTLW &= ~(0x25<<8);

   //Tenim que SMCLK va a 24Mhz. Volem un baud rate de 500kbps=0.5Mbps, i estem fent sobremostreig de 16.
   //24/0.5=48 ; ara, 48/16=3, on el 16 es el sobremostreig
    UCA2BRW = 3;

   //Incialitzem el pin d'adreçament de dades per a la linia half-duplex dels motors
   P3DIR |= DIR_UART;                   // PORT P3.0 com a sortida (Data direction: Selector Tx/Rx)
   P3SEL0 &= ~DIR_UART;                 // PORT P3.0 com I/O digital (GPIO)
   P3SEL1 &= ~DIR_UART;
   P3OUT &= ~DIR_UART;                  // Inicialitzem port P3.0 a 0 (Rx)

   //Configurem els pins de la UART
   Port_UARTx_SEL0 |= BIT2 | BIT3;      //I/O funcio: P3.2 = UART2RX, P3.3 = UART2TX
   Port_UARTx_SEL1 &= ~ (BIT2 | BIT3);

   //Reactivem la linia de comunicacions serie
   UCA2CTLW0 &= ~UCSWRST;

   //Interrupcions
   EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
   EUSCI_A2->IE |= EUSCI_A_IE_RXIE;

   NVIC->ICPR[0] |= 1 <<((EUSCIA2_IRQn) & 31); // Comprovem que no hi ha int residual pendent a la USCI
   NVIC->ISER[0] |= 1 <<((EUSCIA2_IRQn) & 31); // Habilitem les int. de la USCI
}

//-------------------------------------------------------------------------


//------------------------------BUZZER INSTRUCCIONS------------------------

void timer32_delay_us(uint32_t microseconds) {
    // Timer32 usa 3 MHz como default (SMCLK = 3 MHz)
    TIMER32_1->LOAD = microseconds * 3 - 1;  // 3 cycles per microsecond
    TIMER32_1->CONTROL = TIMER32_CONTROL_ENABLE | TIMER32_CONTROL_MODE | TIMER32_CONTROL_PRESCALE_0;
    TIMER32_1->INTCLR = 0;  // Limpiar interrupción

    while((TIMER32_1->RIS & 1) == 0);  // Esperar que cuente a cero
}

void simple_delay_ms(uint32_t ms) {
    volatile uint32_t count;
    while (ms--) {
        count = 3000; // Ajusta este valor para que 3000 ciclos sean ~1 ms
        while (count--) ;
    }
}

void playTone(uint32_t freq, uint32_t duration_ms) {
    uint32_t period_us = 1000000 / freq;
    uint32_t on_time = period_us / 4;        // 25% encendido
    uint32_t off_time = period_us - on_time; // 75% apagado
    uint32_t cycles = (duration_ms * 1000) / period_us;

    uint32_t i;
    for(i = 0; i < cycles; i++) {
        P2->OUT |= BIT7;
        timer32_delay_us(on_time);
        P2->OUT &= ~BIT7;
        timer32_delay_us(off_time);
    }
}

void tocar_himne_FCB(void) {
    uint16_t notes[] = {
            329, 392, 523, 0,  // MI, SOL, DO'
            329, 392, 523, 0,  // MI, SOL, DO'
            329, 392, 523, 587, 523, 493, // MI, SOL, DO', RE', DO', SI
            440, 493, 523, 493, 440, 392, // LA, SI, DO', SI, LA, SOL
            349, 392, 440, 392, 349, 329, // FA, SOL, LA, SOL, FA, MI
            293, 329, 370, 392, 370, 392, 440, 493, // RE, MI, FA#, SOL, FA#, SOL, LA, SI
            523, 493, 440, 392, 293, 392, 493, 440,493,440,493,440,493,440,493,440,493,440,493,440,493,440,493, 392 // DO', SI, LA, SOL, RE, SOL, SI, LA (trino), SOL

        };

    uint16_t durations[] = {
            1125, 375, 375, 1687,
            1125, 375, 375, 1687,
            750, 188, 375, 375, 188, 1125,
            562, 188, 375, 188, 188, 1125,
            562, 188, 375, 188, 188, 1125,
            750, 375, 375, 188, 188, 188, 188, 750,
            375, 282, 94, 188, 188, 188, 188, 47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47, 375
        };

    int length = sizeof(notes) / sizeof(notes[0]);
    int i;
    for(i = 0; i < length; i++) {
        if (notes[i] == 0) {
            // Silencio: solo espera el tiempo de la duración
            simple_delay_ms(durations[i]);
        } else {
            playTone(notes[i] / 4, durations[i] * 10);
        }
    }
}


void tocar_himne_CDT(void) {
    uint16_t notes[] = {
            329, 392, 523, 0,  // MI, SOL, DO'
            329, 392, 523, 0,  // MI, SOL, DO'
            329, 392, 523, 587, 523, 493, // MI, SOL, DO', RE', DO', SI
            440, 493, 523, 493, 440, 392, // LA, SI, DO', SI, LA, SOL
            349, 392, 440, 392, 349, 329, // FA, SOL, LA, SOL, FA, MI
            293, 329, 370, 392, 370, 392, 440, 493, // RE, MI, FA#, SOL, FA#, SOL, LA, SI
            523, 493, 440, 392, 293, 392, 493, 440,493,440,493,440,493,440,493,440,493,440,493,440,493,440,493, 392 // DO', SI, LA, SOL, RE, SOL, SI, LA (trino), SOL

        };

    uint16_t durations[] = {
            1125, 375, 375, 1687,
            1125, 375, 375, 1687,
            750, 188, 375, 375, 188, 1125,
            562, 188, 375, 188, 188, 1125,
            562, 188, 375, 188, 188, 1125,
            750, 375, 375, 188, 188, 188, 188, 750,
            375, 282, 94, 188, 188, 188, 188, 47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47, 375
        };

    int length = sizeof(notes) / sizeof(notes[0]);
    int i;
    for(i = 0; i < length; i++) {
        if (notes[i] == 0) {
            // Silencio: solo espera el tiempo de la duración
            simple_delay_ms(durations[i]);
        } else {
            playTone(notes[i] / 4, durations[i] * 10);
        }
    }
}
//----------------------------------------------------------------------------




//--------------------------RUTINAS DE INTERRUPCIÓN-------------------------
/**
 * Rutina d'interrupcio del timer de gir
 */
void TA0_0_IRQHandler(void)
{
    const uint8_t LED1_ON_OFF = 0x01;
    TA0CCTL0 &= ~CCIFG;
    P1OUT ^= LED1_ON_OFF;

    //Timer de gir
    timer_gir++;
}

/*
 * Rutina d'interrupcio del timer per fer el timeout
 */
void TA1_0_IRQHandler(void)
{
    TA1CCTL0 &= ~CCIFG;
    contador_T1_TIMEOUT++;
}

/*
 * Interrupcio de la UART2 quan rep informacio
 */
void EUSCIA2_IRQHandler(void)
{
    // we see if the interrupt is related to reception
    EUSCI_A2->IFG &=~ EUSCI_A_IFG_RXIFG; // Clear interrupt
    UCA2IE &= ~UCRXIE; //disabled interrupts in RX

    //UCA2RXBUF estara el dato que volem llegir
    DatoLeido_UART = UCA2RXBUF;

    Byte_recibido_bool=1;
    UCA2IE |= UCRXIE;

}

/*
 * Interrupcio del boto per canviar la referencia
 */
void PORT5_IRQHandler(void){
    uint8_t flag = P5IV; //guardem el vector d'interrupcions i el netegem
    char buffer[64];

    P5IE &= ~(BIT1);

    //Posem referencia a 1 i backflip a 1
    //De normal la referencia estarà a 0
    if(referencia == 0){
        referencia = 1;
        backflip = 1;
        sprintf(buffer, "%s", "REF: dreta");
    }else if(referencia == 1){
        referencia = 0;
        backflip = 1;
        sprintf(buffer, "%s", "REF: esquerra");
    }

    halLcdClearLine(4);
    halLcdPrintLine(buffer, 4, NORMAL_TEXT);

    P5IE |= (BIT1);
}

//--------------------------------------------------------------------------


//------------------------------IMPLEMENTACIONS LLIBRERIA---------------------------------

void set_velocitat(uint16_t velocitat){
    SetVelocitatMotor(MOTOR_ESQUERRA_ID, velocitat ^ 1024);
    SetVelocitatMotor(MOTOR_DRETA_ID, velocitat);
    velocitat_actual=velocitat;
}

// Moviments
void moure_endavant(void) {
    set_velocitat(velocitat_actual & (~0x400));
    EncendreMotor(MOTOR_ESQUERRA_ID);
    EncendreMotor(MOTOR_DRETA_ID);
    caminant=1;
}

void moure_enrere(void) {
    set_velocitat(velocitat_actual | 0x400);
    EncendreMotor(MOTOR_ESQUERRA_ID);
    EncendreMotor(MOTOR_DRETA_ID);
    caminant=1;
}

void girar_dreta(void){
    SetVelocitatMotor(MOTOR_ESQUERRA_ID, velocitat_actual & (~0x400));
    SetVelocitatMotor(MOTOR_DRETA_ID, velocitat_actual & (~0x400));


    EncendreMotor(MOTOR_ESQUERRA_ID);
    EncendreMotor(MOTOR_DRETA_ID);
}

void girar_esquerra(void){
    SetVelocitatMotor(MOTOR_ESQUERRA_ID, velocitat_actual | 0x400);
    SetVelocitatMotor(MOTOR_DRETA_ID, velocitat_actual | 0x400);


    EncendreMotor(MOTOR_ESQUERRA_ID);
    EncendreMotor(MOTOR_DRETA_ID);
}

void aturar(void){
    SetVelocitatMotor(MOTOR_ESQUERRA_ID, 0 ^ 1024);
    SetVelocitatMotor(MOTOR_DRETA_ID, 0);

    //AturaMotor(MOTOR_ESQUERRA_ID);
    //AturaMotor(MOTOR_DRETA_ID);
    caminant=0;
}

// Sensors
RxReturn llegir_sensor(uint8_t id_sensor) {
    char buffer[64];  // Espacio suficiente para la cadena a imprimir
    byte param[2];
    const char* direccion = "";  // Para guardar el texto correspondiente al sensor

    switch(id_sensor) {
    case 0:
        param[0] = 0x1A;
        param[1] = 0x01;
        direccion = "izq";
        break;
    case 1:
        param[0] = 0x1B;
        param[1] = 0x01;
        direccion = "frente";
        break;
    case 2:
        param[0] = 0x1C;
        param[1] = 0x01;
        direccion = "der";
        break;
    default:
        direccion = "desconocido";
        break;
    }

    RxReturn rxReturn = ComunicationPacket(0x64, 0x02, 0x02, param);
    sprintf(buffer, "S. %s: %03u", direccion, rxReturn.StatusPacket[5]);
    //Ponemos como numero de línea el id del sensor para que así cada uno se imprima en una línea diferente
    halLcdPrintLine(buffer, id_sensor, NORMAL_TEXT);
    return rxReturn;
}


//----------------------------------------------------------------------------------
