

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib2.h"

#include "LibreriaRobotSEPA.h"


uint32_t reloj=0;                                     //Reloj del sistema
int PeriodoPWM;                                       //Periodo para que los PWM se configuren a 50 Hz
volatile int Max_vel = 40000;                         //Valor máximo del PWM determinado experimentalmente para que la velocidad del coche sea máxima
volatile int Min_vel = 1000;                          //Valor mínimo del PWM de tal manera que la velocidad del coche es mínima pero no nula (determinada experimentalmente)
int velCoche;                                         //Variable que almacena el periodo PWM para configurar la velocidad del coche
int velBrazo;                                         //Variable que almacena el periodo PWM para configurar la velocidad del brazo
int PuertoIN1,PinIN1,PuertoIN2,PinIN2,OutPWM;         //Variables para almacenar

int MotorElegido = 2;                                 //Valor que almacena la articulación del brazo que se va a mover con el joystick
                                                      //Su valor vendrá dado por el valor que le indique el mando

int mMenos,mMas;                                      //Pendientes de las rectas de mapeos de velocidades de coche y brazo
int bMenos,bMas;                                      //Ordenadas en el origen de las rectas de mapeos de velocidades de coche y brazo
int EjeXIzda=0, EjeYIzda=0, EjeXDcha=0, EjeYDcha=0;   //Variables donde se almacenan las lecturas de los joystick del mando

volatile int UARTflag=0;                              //Flag de interrupción que se activa cuando se recibe una transferencia de datos
                                                      //completa del mando (cada ciclo incluye las lecturas de los 2 ejes de cada joystick,
                                                      //y la articulación del brazo elegida para moverse con el joystick del brazo)

volatile int i=0;                                     //Variable para recorrer el buffer de recepción de la comunicación serie. Debe estar fuera
                                                      //de la rutina de interrupción para mantener el valor entre distintas recepciones (interrupciones)

volatile int buffer[10]={0,0,0,0,0,0,0,0,0,0};        //Buffer para almacenar una transmisión completa del mando. El mando envía en este orden:
                                                      //Caracter de control, EjeXIzda8bitsMásSig,EjeXIzda8bitsMenosSig, EjeYIzda8bitsMásSig,EjeYIzda8bitsMenosSig,
                                                      //EjeXDcha8bitsMásSig,EjeXDcha8bitsMenosSig, EjeYDcha8bitsMásSig,EjeYDcha8bitsMenosSig, MotorElegido.
                                                      //El caracter de control ('A') sirve para sincronizar la emisión y la recepción.

volatile int d=0;                                     //Distancia en cm medida por el ultrasonidos
volatile int t=0;                                     //Tiempo en microsegundos que cuenta el Timer1
volatile int t1=0;                                    //Tiempo en microsegundos que tarda el echo en pasar de 0 a 1 (medir el ancho de echo)
volatile int t2=0;                                    //Tiempo en microsegundos que tarda el echo en pasar de 1 a 0 (medir el ancho de echo)
volatile int flag=0;                                  //Flag de interrupción para la rutina de interrupción del ultrasonidos


//RUTINA DE INTERRUPCIÓN TIMER1 (ULTRASONIDOS): contar tiempo en microsegundos
void IntUSeg(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Borra flag
    t++;
}

//RUTINA DE INTERRUPCIÓN ECHO (ULTRASONIDOS): su finalidad es medir el ancho de la onda de echo que se recibe (medir incremento t2-t1)
void rutina_interrupcionEcho(void)
{
    if(flag==0 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_7))
    {
        flag=1;
        t1=t;
    }
    else if (flag==1 && !GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_7))
    {
        flag=2;
        t2=t;
    }

    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_7);
}

//FUNCIÓN DE INTERRUPCIÓN DE LA UART6: interrumpirá al micro cada vez que llegue un dato discreto
//(no una transmisión completa) del mando (Bluetooth)
void IntUART(void)
{
    buffer[i]=UARTCharGet(UART6_BASE);
    if(buffer[0]==65)
    {
        i++;
    }
    if(i>9)
    {
        UARTflag=1;
        i=0;
    }
    UARTIntClear(UART6_BASE, UART_INT_RX); // Borra flag

}


int main(void){

    reloj=SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);



    //HABILITAR PUERTOS GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    //CONFIGURAR UART0 PARA COMUNICACIÓN SERIE CON EL ORDENADOR (DEPURACIÓN)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, reloj);

    //CONFIGURAR UART6 PARA COMUNICACIÓN BLUETOOTH (SLAVE)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //UARTStdioConfig(6, 115200, reloj);  //La UART que use el bluetooth se tiene que configurar a mano para que no use la librería uartstdio.c
    UARTConfigSetExpClk(UART6_BASE, reloj, 115200,
                        (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_WLEN_8));
    UARTEnable(UART6_BASE);

    //CONFIGURACIÓN DE TODOS LOS PWM
    //Configuracion PWM a través de una función de la librería "LibreriaRobotSEPA.c", que incluye las líneas necesarias para
    //configurar un PWM a 50 Hz
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOF, PWM0_BASE, GPIO_PF1_M0PWM1, GPIO_PORTF_BASE, GPIO_PIN_1, PWM_GEN_0, PWM_OUT_1, PWM_OUT_1_BIT);
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOF, PWM0_BASE, GPIO_PF2_M0PWM2, GPIO_PORTF_BASE, GPIO_PIN_2, PWM_GEN_1, PWM_OUT_2, PWM_OUT_2_BIT);
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOF, PWM0_BASE, GPIO_PF3_M0PWM3, GPIO_PORTF_BASE, GPIO_PIN_3, PWM_GEN_1, PWM_OUT_3, PWM_OUT_3_BIT);
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOG, PWM0_BASE, GPIO_PG0_M0PWM4, GPIO_PORTG_BASE, GPIO_PIN_0, PWM_GEN_2, PWM_OUT_4, PWM_OUT_4_BIT);
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOG, PWM0_BASE, GPIO_PG1_M0PWM5, GPIO_PORTG_BASE, GPIO_PIN_1, PWM_GEN_2, PWM_OUT_5, PWM_OUT_5_BIT);
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOK, PWM0_BASE, GPIO_PK4_M0PWM6, GPIO_PORTK_BASE, GPIO_PIN_4, PWM_GEN_3, PWM_OUT_6, PWM_OUT_6_BIT);
    configuraPWM50Hz(SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_GPIOK, PWM0_BASE, GPIO_PK5_M0PWM7, GPIO_PORTK_BASE, GPIO_PIN_5, PWM_GEN_3, PWM_OUT_7, PWM_OUT_7_BIT);

    //CONFIGURACIÓN DE LOS MAPEOS
    //Mapeo: usaremos estas pendientes y ordenadas en el origen para mapear las lecturas de los joystick a velocidades de los motores
    mMenos=(Min_vel-Max_vel)/1850;
    mMas=(Max_vel-Min_vel)/(4096-2300);
    bMenos=Max_vel;
    bMas=Min_vel-mMas*2300;
    //////////////////////////////


    //CONFIGURACIÓN DE TODAS LAS SEÑALES DIGITALES
    //SALIDAS MOTORES DEL BRAZO
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1);  //MOTOR 1  IN1_M1 = PE0   IN2_M1 = PE1
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4|GPIO_PIN_5);  //MOTOR 2  IN1_M2 = PL4   IN2_M2 = PL5
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);             //MOTOR 3  IN1_M3 = PM7
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);             //MOTOR 3                 IN2_M3 = PP5
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);             //MOTOR 4  IN1_M4 = PA7
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);             //MOTOR 4                 IN2_M4 = PM0
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_2|GPIO_PIN_3);  //MOTOR 5  IN1_M5 = PK2   IN2_M5 = PK3

    //SALIDAS MOTORES DEL COCHE
    GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_2|GPIO_PIN_3);  //MOTOR IZDA  IN1_IZDA = PH3   IN2_IDZA = PH2
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_5);//MOTOR DCHA  IN1_DCHA = PD7   IN2_DCHA = PE3

    //ENTRADAS DEL BLUETOOTH: ya configuradas en la UART6

    //ENTRADA Y SALIDA DEL ULTRASONIDOS
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7); //Pin echo
    GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6); //Pin trigger

    //CONFIGURACIÓN DEL TIMER1 (ULTRASONIDOS)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //Habilita T1
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM); //T1 a 120MHz
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); //T1 periodico y 32bits
    TimerLoadSet(TIMER1_BASE, TIMER_A, reloj/1000000-1);

    //CONFIGURAR INTERRUPCIÓN DE LA UART
    UARTIntRegister(UART6_BASE,IntUART);
    IntEnable(INT_UART6);
    UARTIntEnable(UART6_BASE, UART_INT_RX);
    IntMasterEnable();

    //CONFIGURAR INTERRUPCIÓN PIN ECHO ULTRASONIDOS
    GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_BOTH_EDGES); // Definir tipo int: flancos de subida y de bajada
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_7);                 // Habilitar pin de interrupción: pin echo, PC7
    GPIOIntRegister(GPIO_PORTC_BASE, rutina_interrupcionEcho);  //Registrar (definir) la rutina de interrupción
    IntEnable(INT_GPIOC);                                       //Habilitar interrupción del pto C

    //CONFIGURAR INTERRUPCIÓN DEL TIMER1
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER1_BASE,TIMER_A,IntUSeg);
    IntEnable(INT_TIMER1A);
    IntMasterEnable();

    //CONFIGURAR PERIFÉRICOS QUE QUEDARÁN ACTIVOS DURANTE EL MODO DE BAJO CONSUMO
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART6);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralClockGating(true);

    //BUCLE INFINITO
    d=0;                    //Inicializar distancia del ultrasonidos a 0 en cada iteración
    IntDisable(INT_GPIOC);  //Deshabilitar interrupciones del echo del ultrasonidos mientras no se esté usando

    while(1)
    {
        while(UARTflag==0)SysCtlSleep();   //Mientras la UART6 no reciba nada, estamos en modo sleep
        UARTflag=0;                        //Justo cuando el micro se despierte, desactivamos el flag de la UART

        EjeXIzda=((buffer[1]<<8) | (buffer[2]));             //Si en la rutina de interrupción se han recibido todas las órdenes
        EjeYIzda=((buffer[3]<<8) | (buffer[4]));             //del mando en una cadena llamada "buffer", aquí la deshacemos para
        EjeXDcha=((buffer[5]<<8) | (buffer[6]));             //obtener todas las variables que ha enviado el mando
        EjeYDcha=((buffer[7]<<8) | (buffer[8]));
        MotorElegido=buffer[9];

        //MONITORIZACIÓN DE VARIABLES POR LA UART0 (COMUNICACIÓN SERIE CON EL ORDENADOR)
        //Borrar pantalla
        UARTprintf("\033[8;1H                                                                                       \n");
        UARTprintf("                                                                                                  ");
        //Sacar por UART0 las posiciones de los mandos recibidas
        UARTprintf("\033[8;1HEJEXIZDA = %d EJEYIZDA = %d MOTORELEGIDO = %d                \n", EjeXIzda,EjeYIzda,MotorElegido);
        UARTprintf("EJEXDCHA = %d EJEYDCHA = %d  VelCoche=%d                                ", EjeXDcha,EjeYDcha,velCoche);

        //CONTROL DEL ROBOT: AVANCE DEL COCHE CONDICIONADO A QUE EL ULTRASONIDOS DETECTE UNA DISTANCIA MAYOR QUE 15 cm
        //DE ESTA FORMA SE EVITAN COLISIONES DEL ROBOT

        //El giro y el retroceso siguen permitidos, para salvar el obstáculo

        //CONTROL COCHE: la dirección del coche se decidirá con el joystick de la izquierda
        if(EjeYIzda < 1850){                           //Si el joystick está en la parte superior, avanzar
            if(d>15) velCoche=mMenos*EjeYIzda+bMenos;  //Mapear velocidad del coche con el eje del Joystick
            else velCoche = 10;                        //Dar al coche velocidad nula para avanzar si tengo un objeto delante
            Avanzar(velCoche);                         //Función de la librería "LibreriaRobotSEPA.c"
        }

        if(EjeYIzda > 2300){              //Si el joystick está en la parte inferior, retroceder
            velCoche=mMas*EjeYIzda+bMas;  //Mapear velocidad del coche con el eje del Joystick
            Retroceder(velCoche);         //Función de la librería "LibreriaRobotSEPA.c"
        }

        if(EjeYIzda > 1850 && EjeYIzda < 2300 && EjeXIzda > 1850 && EjeXIzda < 2300){  //Si el joystick está en la posición central, parar el coche
            velCoche=0;                                                                //Realmente Parar() hace que velCoche = 10 (cantidad determinada
                                                                                       //experimentalmente para detenerse), pero dejamos aquí esta variable
                                                                                       //monitorizar por puerto serie.
            Parar();                                                                   //Función de la librería "LibreriaRobotSEPA.c"
        }

        if(EjeXIzda > 2300){              //Si el joystick está en la parte derecha, girar a la derecha
            velCoche=mMas*EjeXIzda+bMas;  //Mapear velocidad del coche con el eje del Joystick
            GiroDerecha(velCoche);        //Función de la librería "LibreriaRobotSEPA.c"
        }

        if(EjeXIzda < 1850){                  //Si el joystick está en la parte izquierda, girar a la izquierda
            velCoche=mMenos*EjeXIzda+bMenos;  //Mapear velocidad del coche con el eje del Joystick
            GiroIzquierda(velCoche);          //Función de la librería "LibreriaRobotSEPA.c"
        }


        //CONTROL BRAZO

        //En función del motor elegido, en el control que viene a continuación actuaremos sobre una articulación u otra.
        //Para ello, inicializaremos estas variables intermedias, que usaremos en el código de control que aparece a continuación.
        switch(MotorElegido){
        case 1:
            PuertoIN1 = GPIO_PORTE_BASE; //E0
            PuertoIN2 = GPIO_PORTE_BASE; //E1
            PinIN1 = GPIO_PIN_0;
            PinIN2 = GPIO_PIN_1;
            OutPWM = PWM_OUT_2;
            break;

        case 2:
            PuertoIN1 = GPIO_PORTL_BASE;   //L4
            PuertoIN2 = GPIO_PORTL_BASE;   //L5
            PinIN1 = GPIO_PIN_4;
            PinIN2 = GPIO_PIN_5;
            OutPWM = PWM_OUT_3;
            break;

        case 3:
            PuertoIN1 = GPIO_PORTM_BASE;  //M7
            PuertoIN2 = GPIO_PORTP_BASE;  //P5
            PinIN1 = GPIO_PIN_7;
            PinIN2 = GPIO_PIN_5;
            OutPWM = PWM_OUT_5;
            break;

        case 4:
            PuertoIN1 = GPIO_PORTA_BASE;  //A7
            PuertoIN2 = GPIO_PORTM_BASE;  //M0
            PinIN1 = GPIO_PIN_7;
            PinIN2 = GPIO_PIN_0;
            OutPWM = PWM_OUT_6;
            break;

        case 5:
            PuertoIN1 = GPIO_PORTK_BASE;  //K2
            PuertoIN2 = GPIO_PORTK_BASE;  //K3
            PinIN1 = GPIO_PIN_2;
            PinIN2 = GPIO_PIN_3;
            OutPWM = PWM_OUT_7;
            break;
        }

        //Código de control de la articulación x (x = de 1 a 5) del brazo del robot:
        //La velocidad del motor x se decidirá con el eje horizontal del joystick, y el motor a mover a través de la pantalla del mando
        //Control motor x (eje horizontal)
        if(EjeXDcha > 2300){  //Si la velocidad es positiva, girar en un sentido
            GPIOPinWrite(PuertoIN1,PinIN1,PinIN1);         //IN1Dcha a 1
            GPIOPinWrite(PuertoIN2,PinIN2,0);              //IN2Dcha a 0
            velBrazo=mMas*EjeXDcha+bMas;                   //Mapear velocidad del coche con el eje del Joystick
            PWMPulseWidthSet(PWM0_BASE, OutPWM, velBrazo); //Dar velocidad al motor x del brazo
        }

        if(EjeXDcha < 1850){                               //Si la velocidad es negativa, girar en el otro sentido
            GPIOPinWrite(PuertoIN1,PinIN1,0);              //IN1Dcha a 0
            GPIOPinWrite(PuertoIN2,PinIN2,PinIN2);         //IN2Dcha a 1
            velBrazo=mMenos*EjeXDcha+bMenos;               //Mapear velocidad del coche con el eje del Joystick
            PWMPulseWidthSet(PWM0_BASE, OutPWM, velBrazo); //Dar velocidad al motor x del brazo
        }

        if(EjeXDcha > 1850 && EjeXDcha < 2300){            //Si el joystick está en la posición central, parar el motor
            //GPIOPinWrite(PuertoIN1,PinIN1,0);            //IN1Dcha a 0   Da igual el sentido del motor, su velocidad será cero
            //GPIOPinWrite(PuertoIN2,PinIN2,PinIN2);       //IN2Dcha a 1
            velBrazo = 10;                                 //Valor experimental del PWM para detener el motor
            PWMPulseWidthSet(PWM0_BASE, OutPWM, velBrazo); //Dar velocidad al motor x del brazo
        }

        //MANEJO DEL SENSOR ULTRASONIDOS
        flag=0;                                                 //Bajar el flag implica inicio de la medida de distancia
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6,0);             //Poner a 0 Trigger
        TimerEnable(TIMER1_BASE, TIMER_A);                      //Comenzar a contar el tiempo
        while(t<4)SysCtlSleep();                                //Pondremos el Trigger durante 4us para asegurar un disparo limpio
        TimerDisable(TIMER1_BASE, TIMER_A);                     //Detener la cuenta de tiempo

        t=0;                                                    //Tras un comienzo de disparo limpio, reseteamos el tiempo y lo medimos
                                                                //para generación del pulso del Trigger (por ejemplo, 10 us)

        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_PIN_6);    //Poner a 1 Trigger
        TimerEnable(TIMER1_BASE, TIMER_A);                      //Comenzar a contar el tiempo
        while(t<10)SysCtlSleep();                               //Durante 10 us, mantendremos Trigger a 1
        TimerDisable(TIMER1_BASE, TIMER_A);                     //Detener la cuenta de tiempo
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6,0);             //Poner a 0 Trigger

        t=0;                                                    //Reseteamos la variable tiempo para ahora usarla para el cálculo de la distancia

        TimerEnable(TIMER1_BASE, TIMER_A);                      //Comenzar a contar el tiempo
        IntEnable(INT_GPIOC);                                   //Habilitar interrupción del echo del ultrasonidos
        while(t<11800 && (flag==0 || flag==1))SysCtlSleep();    //11800 useg es el máximo tiepo que esperamos echo, si no se recibe en ese tiempo,
                                                                //entonces distancia mayor a 2m (rango sensor) vsonido=343m/seg

        IntDisable(INT_GPIOC);                                  //Deshabilitar interrupciones del echo del ultrasonidos mientras no se esté usando
        TimerDisable(TIMER1_BASE, TIMER_A);                     //Detener la cuenta del tiempo

        //Si el flag de la interrupción del ultrasonidos vale finalmente dos tras la rutina de interrupción, quiere decir
        //que el echo ha vuelto a tiempo al módulo de ultrasonidos. Si tiene otro valor, es que no ha regresado, y hemos salido
        //del SysCtlSleep anterior por timeout.
        if(flag==2)
        {
            d=(t2-t1)/59;  //Cálculo de la distancia (ver memoria)
            UARTprintf("Distancia: %d                                                            \n",d);
        }
        else
        {
            d=200;          //Distancia de dos metros o superior
            UARTprintf("Distancia: Distancia superior a 2m                                        \n");
        }
        t=0;                //Resetear el tiempo para la siguiente iteración
    }
}

