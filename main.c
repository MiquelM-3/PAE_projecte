#include "msp.h"
#include "lib_PAE.h"
#include "robot_lib.h"


#define CONF_OUTPUT_LED1 0x01  //0000 0001

//Aquests define serveixen per poder canviar els motors rapidament
#define MOTOR_ESQUERRA_ID 2
#define MOTOR_DRETA_ID 4
#define SENSOR_ID 100

//Indiquen ubicacio del robot
#define ESQUERRA 0
#define DRETA 1

//Indiquen distancia del sensor fins a l'obstacle (rang: 0<x<255)
#define CERCANIA_PARET_MAXIM_LIMIT 254
#define CERCANIA_PARET_MAXIM 180
#define CERCANIA_PARET_MITJA 70
#define CERCANIA_PARET_MINIM 20
#define WR_INST_BIT 3

typedef uint8_t byte;

uint8_t inputUSUARI; //Input usuari per saber si ha d'anar esquerra o dreta

/**
 * Funcio copiada del campus; inicialitza el led1.
 * Deixem el led1 perquè està per debugging!
 */
void init_LED1(void){
    const uint8_t CONF_GPIO_LED1 = 0xFE;
    const uint8_t INIT_STATE_OFF = 0xFE;
    // Configuration of GPIO connection for LED1
    P1SEL0 &= CONF_GPIO_LED1;
    P1SEL1 &= CONF_GPIO_LED1;
    // Configuration of pin0's port 1 as output
    P1DIR |= CONF_OUTPUT_LED1;
    P1OUT &= INIT_STATE_OFF;
}

//--------------------------------------------------------------------------


//-----------------------------------MAIN-----------------------------------

void main(void)
  {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer


    //Inicialtizacions
    init_ucs_24MHz();

    //Init dispositius GPIO
    init_LED1();
    init_botton();
    init_buzzer();

    //Init timers
    init_timer_A0();
    init_timer_A1();

    //Init UART
    init_UART();

    __enable_interrupt();

    //Configura els angles
    byte cw_limit_low[16]  = {6, 0,0,0,0};
    ComunicationPacket(MOTOR_ESQUERRA_ID, 5, WR_INST_BIT, cw_limit_low);
    ComunicationPacket(MOTOR_DRETA_ID, 5, WR_INST_BIT, cw_limit_low);


    //Posa els leds (serveix per debuging)
    byte array_led[2]={25,0};//25 es direccio, 0 o 1 si volem encendre o apagar
    ComunicationPacket(MOTOR_ESQUERRA_ID,2,WR_INST_BIT,array_led); //aquesta encen o apaga el led
    ComunicationPacket(MOTOR_DRETA_ID,2,WR_INST_BIT,array_led); //aquesta encen o apaga el led

    halLcdInit(); //Inicializar y configurar la pantallita
    halLcdClearScreenBkg(); //Borrar la pantalla, rellenando con el color de fondo


    //Aturem el robot per si estava en marxa prèviament
         aturar();

    //Inicialment setegem una velocitat dels dos motors, i fem que avanci
    set_velocitat(400);
    moure_endavant();


    //Cridem el bucle principal
    buclecodiproj();

    //Llegim el sensor en bucle infinit; mai hauriem d'arribar aqui!
    while(1){
        llegir_sensor(0);
        llegir_sensor(1);
        llegir_sensor(2);
        aturar();
    }


}
//--------------------------------------------------------------------------

/**
 * Aquesta funció llegeix els sensors i els imprimeix
 */
void imprimir_sensors(){
    llegir_sensor(0);
    llegir_sensor(1);
    llegir_sensor(2);
}

/*
 * Aquesta funció busca la primera paret (en linia recta). Un cop buscada la primera paret,
 * s'espera un input de l'usuari, i a segons d'aquest input
 * anirà cap a l'esquerra o cap a la dreta
 */
int buscar_primera_paret(){
    RxReturn packetsensor_endavant;
    char buffer[64];

    byte paret_trobada = 0;
    //Assumirem que la primera paret està al davant
    while(!paret_trobada){
        packetsensor_endavant= llegir_sensor(1);
        //Ens hem trobat la primera paret que esta endavant
        if(packetsensor_endavant.StatusPacket[5]>CERCANIA_PARET_MAXIM){
            //ATURAREM EL MOTOR I ENS ESPERAREM FINS A UN TEMPS PETIT QUE LUSUARI PUGUI FER INPUT
            aturar();
            Reset_gir_temps();
            while(!control_gir_temps(6)){imprimir_sensors();}
            paret_trobada = 1;
        }
    }
    //A segons de la referencia girarem a l'esquerra o a la dreta
    if(referencia==0){
        executar_gir(DRETA,5);
        sprintf(buffer, "%s", "REF: esquerra");
    }else{
        executar_gir(ESQUERRA,5);
        sprintf(buffer, "%s", "REF: dreta");
    }

    halLcdClearLine(4);
    halLcdPrintLine(buffer, 4, NORMAL_TEXT);
    tocar_himne_FCB();
    return 0;
}

/*
 * Funcio: girar en la direccio indicada fins que passi la unitat de temps dita
 * POSICIO_GIR: esquerra o dreta (0 o 1)
 * UNITAT TEMPS: unitat de temps que estarà executant el gir
 */

void executar_gir(int posicio_gir, int unitat_temps){
    //Tenim dos casos, o be esquerra o be dreta
    if(posicio_gir==ESQUERRA){
        girar_esquerra();
        Reset_gir_temps();
        while(!control_gir_temps(unitat_temps)){imprimir_sensors();}

    }else if(posicio_gir==DRETA){
        girar_dreta();
        Reset_gir_temps();
        while(!control_gir_temps(unitat_temps)){imprimir_sensors();}
    }
}

/*
 * Funcio: girar en la direccio indicada fins que detecti una pared indicada per maxim_sensor o fins que s'esgoti el temps
 *
 * Params: posicio_gir = cap a on volem girar
 *         unitat_temps = quantes unitats de temps serà el MAXIM que girarem
 *         maxim_sensor = fins a on haurem de girar
 *
 */
void executar_gir_sensors(int posicio_gir, int unitat_temps, int maxim_sensor){
    int sensor_no_detectat = 1;
    RxReturn packetsensor_dreta;
    RxReturn packetsensor_esquerra;

    if(posicio_gir==ESQUERRA){
        girar_esquerra();
        Reset_gir_temps();
        while(!control_gir_temps(unitat_temps) && sensor_no_detectat){
            imprimir_sensors();
            packetsensor_dreta=llegir_sensor(2);
            //llegirem els sensors; si estem girant cap a l'esquerra aixo vol dir que voldrem tenir la paret de la dreta
            //Si el sensor de la dreta esta entre [-20 + maxim_sensor, maxim_sensor + 20] aixo vol dir que estarem en un rang acceptable
            if(packetsensor_dreta.StatusPacket[5] > -10 + maxim_sensor && packetsensor_dreta.StatusPacket[5] < maxim_sensor + 10){
                sensor_no_detectat = 0;
            }
        }
    }else if(posicio_gir==DRETA){
        girar_dreta();
        Reset_gir_temps();
        while(!control_gir_temps(unitat_temps) && sensor_no_detectat){
            imprimir_sensors();
            packetsensor_esquerra=llegir_sensor(0);
            //llegirem els sensors; si estem girant cap a la dreta aixo vol dir que voldrem tenir la paret de lesquerra
            //Si el sensor de lesq esta entre [-20 + maxim_sensor, maxim_sensor + 20] aixo vol dir que estarem en un rang acceptable
            if(packetsensor_esquerra.StatusPacket[5] > -10 + maxim_sensor && packetsensor_esquerra.StatusPacket[5] < maxim_sensor + 10){
                sensor_no_detectat = 0;
            }
        }
    }
}


/*
 * Bucle principal del projecte.
 * S'encarrege de rebre els sensors i a segons del seu estat i de la referència que tinguem cridar la funció de gir adequada
 * Implementació de pooling bàsica, amb if-else.
 */
void buclecodiproj(){

    //Returns de tres sensors
    RxReturn packetsensor_endavant;
    RxReturn packetsensor_esquerra;
    RxReturn packetsensor_dreta;

    //Primer bucle per anar a trobar la pared
    buscar_primera_paret();
    moure_endavant();

    //El primer backflip no el tindrem en compte. El backflip serveix per quan canviem de direcció
    if(backflip){
        backflip = 0;
    }

    //bucle infinit principal del programa
    int i=1;
    while(i){
        //Haurem de llegir ara dos sensors, el del davant i el de la dreta / el de lesquerra
        packetsensor_endavant=llegir_sensor(1);
        packetsensor_esquerra=llegir_sensor(0);
        packetsensor_dreta=llegir_sensor(2);

        //Si hem activat el joystick
        if(backflip){
            //Si ref == 0, aleshores volem paret cap a l'esquerra (i la tenim a la dreta)
            if ( referencia == 0){
                executar_gir_sensors(DRETA, 16, 40);
            }else{ //sino hem de girar cap a l'altre costat
                executar_gir_sensors(ESQUERRA, 16, 40);
            }
            //ja haurem fet el gir aixi que resetegem
            backflip = 0;
        }

        if(caminant){//no hem clicat el boto de parar

            //Anem cap endavant. Aquesta instrucció pot ser una mica redundant ates que a priori sempre que girem posem el moure_endavant
            //Tanmateix la posem aquí per seguretat, ja que sempre que comencem el bucle haurem d'anar cap endavant
            moure_endavant();

            switch(referencia){
            //CASE 0: TENIM UNA PARET A LESQUERRA
            case 0:
                if(packetsensor_endavant.StatusPacket[5]>CERCANIA_PARET_MAXIM){
                    //girarem perque haurem de resseguir
                    //executar_gir(DRETA,6); //aixo vol dir que s'haura de girar cap a la dreta
                    executar_gir(DRETA, 6);
                    //continuem movent cap endavant
                    moure_endavant();
                    Reset_gir_temps();
                    while(!control_gir_temps(4)){imprimir_sensors();}
                }
                else if(packetsensor_esquerra.StatusPacket[5]<CERCANIA_PARET_MINIM){ //Si deixem de tenir la paret a la esquerra
                    executar_gir(ESQUERRA,1); //aixo vol dir que s'haura de girar cap a la dreta

                    moure_endavant(); //continuem cap endavant
                    Reset_gir_temps();
                    while(!control_gir_temps(2)){imprimir_sensors();}
                } //Si tenim una paret al davant i a la dreta massa aprop haurem de girar una mica
                else if(packetsensor_esquerra.StatusPacket[5]>CERCANIA_PARET_MITJA & packetsensor_endavant.StatusPacket[5]>CERCANIA_PARET_MINIM){
                    //girarem perque haurem de resseguir
                    //executar_gir(DRETA,4); //aixo vol dir que s'haura de girar cap a l'esquerra

                    executar_gir_sensors(DRETA, 4, 40);
                    moure_endavant(); //continuem cap endavant
                    Reset_gir_temps();
                    while(!control_gir_temps(2)){imprimir_sensors();}
                }
                else if(packetsensor_esquerra.StatusPacket[5]>CERCANIA_PARET_MITJA){
                    //girarem perque haurem de resseguir
                    //executar_gir(DRETA,2); //aixo vol dir que s'haura de girar cap a l'esquerra
                    executar_gir_sensors(DRETA, 2, 40);
                    //continuem movent cap endavant
                    moure_endavant();
                    Reset_gir_temps();
                    while(!control_gir_temps(3)){imprimir_sensors();}
                }
                break;

            //TENIM UNA PARET A LA DRETA
            case 1:
                if(packetsensor_endavant.StatusPacket[5]>CERCANIA_PARET_MAXIM){
                    //girarem perque haurem de resseguir
                    //executar_gir(ESQUERRA,6); //aixo vol dir que s'haura de girar cap a l'esquerra
                    executar_gir(ESQUERRA, 6);

                    //continuem movent cap endavant
                    moure_endavant();
                    Reset_gir_temps();
                    while(!control_gir_temps(4)){imprimir_sensors();}
                }
                else if(packetsensor_dreta.StatusPacket[5]<CERCANIA_PARET_MINIM){ //Si deixem de tenir la paret a la dreta
                    executar_gir(DRETA,1); //aixo vol dir que s'haura de girar cap a la dreta

                    moure_endavant(); //continuem cap endavant
                    Reset_gir_temps();
                    while(!control_gir_temps(2)){imprimir_sensors();}
                } //Si tenim una paret al davant i a la dreta massa aprop haurem de girar una mica
                else if(packetsensor_dreta.StatusPacket[5]>CERCANIA_PARET_MITJA & packetsensor_endavant.StatusPacket[5]>CERCANIA_PARET_MINIM){
                    //girarem perque haurem de resseguir
                    executar_gir_sensors(ESQUERRA, 4, 40); //aixo vol dir que s'haura de girar cap a l'esquerra

                    moure_endavant(); //continuem cap endavant
                    Reset_gir_temps();
                    while(!control_gir_temps(2)){imprimir_sensors();}
                } //Aixo vol dir que hem pillat una paret a la dreta molt, aixi q toca girar cap a l'altra banda
                else if(packetsensor_dreta.StatusPacket[5]>CERCANIA_PARET_MITJA){
                    //girarem perque haurem de resseguir
                    executar_gir_sensors(ESQUERRA, 2, 40); //aixo vol dir que s'haura de girar cap a l'esquerra

                    //continuem movent cap endavant
                    moure_endavant();
                    Reset_gir_temps();
                    while(!control_gir_temps(3)){imprimir_sensors();}
                }
                break;
            case 2:
                break;
            }
        }else{ //caminant==0;
            aturar();
        }
    };
}
