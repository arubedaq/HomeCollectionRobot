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
#include "FT800_TIVA.h"



//Variables para mapear el valor del joystick para dibujar una copia del joystick en pantalla
float mIzdaX, mIzdaY, mDchaX, mDchaY, bIzdaX, bIzdaY, bDchaX, bDchaY;   //Pendientes y ordenadas en el origen para mapear
float MaxEje, MinEje;                                                   //Valores máximo y mínimo de los ejes horizontal y vertical de los joystick
int PixelX, PixelY;                                                     //Posición final donde se dibuja el joystick en pantalla

//Variables que se envían por comunicación serie (Bluetooth)
int DatoXDcha;
int DatoYDcha;
int DatoXIzda;
int DatoYIzda;
unsigned char MotorElegido = 1;
//////////////////////////////////////////////
uint32_t RELOJ=0; //Reloj del sistema

unsigned char ADC0flag = 0; //Flag para rutina de interrupción del ADC0


// =======================================================================
// INICIALIZACIÓN DE VARIABLES PARA EL MANEJO DE LA PANTALLA
// =======================================================================
#define dword long
#define byte char



#define PosMin 750
#define PosMax 1000

#define XpMax 286
#define XpMin 224
#define YpMax 186
#define YpMin 54

unsigned int Yp=120, Xp=245;
// =======================================================================
// Variable Declarations
// =======================================================================

char chipid = 0;                        // Holds value of Chip ID read from the FT800

unsigned long cmdBufferRd = 0x00000000;         // Store the value read from the REG_CMD_READ register
unsigned long cmdBufferWr = 0x00000000;         // Store the value read from the REG_CMD_WRITE register
unsigned int t=0;
// ############################################################################################################
// User Application - Initialization of MCU / FT800 / Display
// ############################################################################################################

unsigned long POSX, POSY, BufferXY;
unsigned long POSYANT=0;
unsigned int CMD_Offset = 0;
unsigned long REG_TT[6];
const int32_t REG_CAL[6]={21696,-78,-614558,498,-17021,15755638};
const int32_t REG_CAL5[6]={32146, -1428, -331110, -40, -18930, 18321010};
#define NUM_SSI_DATA            3
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//INTERRUPCIÓN DEL ADC0: interrumpe cada vez que se termina una conversión de todos los canales
void InterrADC0(void){
    while(!ADCIntStatus(ADC0_BASE, 1, false))         //Esperar a terminar la conversión de todos los canales
    {
    }

    ADC0flag = 1;
    ADCIntClear(ADC0_BASE, 1);     // Poner a 0 el flag de interrupción interno del ADC0

}

int main(void){
    uint32_t pui32ADC0Value[4];    //Cadena donde se almacenan todas las conversiones del ADC. Se declara de tamaño 4
                                   //porque leeremos 4 canales, a los que conectaremos los ejes X e Y de los dos joystick
                                   //(un eje a cada canal)

    RELOJ=SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    HAL_Init_SPI(2, RELOJ);  //Boosterpack a usar, Velocidad del MC
    Inicia_pantalla();       //Arranque de la pantalla

    //CONFIGURAR UART0 PARA COMUNICACIÓN SERIE CON EL ORDENADOR (DEPURACIÓN)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, RELOJ);  //Estamos configurando la UART0 con funciones de uartstdio para poder usar printf en PuTTY

    //CONFIGURAR UART5 PARA COMUNICACIÓN BLUETOOTH (MASTER)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    GPIOPinConfigure(GPIO_PC6_U5RX);
    GPIOPinConfigure(GPIO_PC7_U5TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    //UARTStdioConfig(5, 115200, reloj);  //La UART que use el bluetooth se tiene que configurar a mano para que no use la librería uartstdio.c
    UARTConfigSetExpClk(UART5_BASE, RELOJ, 115200,
                                (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_WLEN_8));
    UARTEnable(UART5_BASE);

    //CONFIGURACIÓN ADC0:
    //JOYSTICK SITUADO A LA IZDA:    VRx = PE3      VRy = PE2
    //JOYSTICK SITUADO A LA DCHA:    VRx = PE1      VRy = PE0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);                         //E2 => eje y dcha; E3 => eje x dcha
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);                         //E0 => eje y IZDA; E1 => eje x IZDA (NUEVO)
    //A partir de aquí, aparece en todas las funciones del ADC el número 1. Este número es el Sequence Number, y representa el
    //número de canales que se van a convertir. Según la documentación, un Sequence Number = 1 nos permite convertir hasta 4 canales.
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);                           //El canal 0 (pin AIN0) será para el eje x IZDA
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);                           //El canal 1 (pin AIN1) será para el eje y IZDA
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);                           //El canal 2 (pin AIN2) será para el eje x DCHA
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 | ADC_CTL_IE |ADC_CTL_END); //El canal 3 (pin AIN3) será para el eje y DCHA, y cerrará la conversión
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 1);                                                        // Poner a 0 el flag de interrupción interno del ADC0

    //CONFIGURACIÓN DE LA RUTINA DE INTERRUPCIÓN DEL ADC0
    ADCIntRegister(ADC0_BASE,1,InterrADC0);
    ADCIntEnable(ADC0_BASE, 1); //Habilitar la interrupción para salir de modo sleep
    IntMasterEnable();

    //CONFIGURACIÓN DEL MODO SLEEP: PERIFÉRICOS QUE SE QUEDARÁN DESPIERTOS
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralClockGating(true);

    //DIBUJAR PANTALLA DE BIENVENIDA
    Nueva_pantalla(16,16,16);
    ComColor(21,160,6);
    ComLineWidth(5);
    ComRect(10, 10, HSIZE-10, VSIZE-10, true);
    ComColor(65,202,42);
    ComRect(12, 12, HSIZE-12, VSIZE-12, true);
    ComColor(255,255,255);

    ComTXT(HSIZE/2,VSIZE/5, 28, OPT_CENTERX,"ROBOT RECOGEDOR");
    ComTXT(HSIZE/2,50+VSIZE/5, 28, OPT_CENTERX," SEPA 4 GIERM. 2022 ");
    ComTXT(HSIZE/2,100+VSIZE/5, 27, OPT_CENTERX,"Jose Antonio Heredia, Arturo Ubeda");

    ComRect(40,40, HSIZE-40, VSIZE-40, false);

    Dibuja();
    Espera_pant();
    int i;
#ifdef VM800B35
    for(i=0;i<6;i++)    Esc_Reg(REG_TOUCH_TRANSFORM_A+4*i, REG_CAL[i]);
#endif
#ifdef VM800B50
    for(i=0;i<6;i++)    Esc_Reg(REG_TOUCH_TRANSFORM_A+4*i, REG_CAL5[i]);
#endif

    //Mapeo: usaremos estas pendientes y ordenadas en el origen para mapear las lecturas de los joystick a posición en la pantalla
    //MaxEje = ValorCentralEje + R - r
    //MinEje = ValorCentralEje - R + r
    MaxEje = 0.25*HSIZE + 45 - 20;
    MinEje = 0.25*HSIZE - 45 + 20;
    mIzdaX=(MaxEje-MinEje)/4096;
    bIzdaX=MinEje;

    MaxEje = 0.7*VSIZE + 45 - 20;
    MinEje = 0.7*VSIZE - 45 + 20;
    mIzdaY=(MaxEje-MinEje)/4096;
    bIzdaY=MinEje;

    MaxEje = 0.75*HSIZE + 45 - 20;
    MinEje = 0.75*HSIZE - 45 + 20;
    mDchaX=(MaxEje-MinEje)/4096;
    bDchaX=MinEje;

    MaxEje = 0.7*VSIZE + 45 - 20;
    MinEje = 0.7*VSIZE - 45 + 20;
    mDchaY=(MaxEje-MinEje)/4096;
    bDchaY=MinEje;
    //////////////////////////////
    //BUCLE INFINITO
    while(1)
    {
        ADCProcessorTrigger(ADC0_BASE, 1);                   //Comenzar la conversión
        while(ADC0flag==0)SysCtlSleep();                     //Poner a dormir al micro hasta que no acabe la conversión de todos los canales
        ADC0flag = 0;
        ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);    // Almacenar las conversiones del ADC0 en la cadena de conversiones

        //Enviar caracter de control de comienzo de transferencia
        UARTCharPut(UART5_BASE,'A');

        //JOYSTICK IZDA
        //Enviar EjeX izda
        UARTprintf("EjeXIzda/16 : %d ",DatoXIzda);
        DatoXIzda = pui32ADC0Value[0];
        UARTCharPut(UART5_BASE,(DatoXIzda >> 8)& 0xff);       //Envio 8 bits más significativos EjeXIz
        UARTCharPut(UART5_BASE,DatoXIzda & 0xff);             //Envio 8 bits menos significativos EjeXIz
        //Enviar EjeY IZDA
        DatoYIzda = pui32ADC0Value[1];
        UARTprintf("\t\tEjeYIzda/16 Enviado: %d", DatoYIzda);
        UARTCharPut(UART5_BASE,(DatoYIzda >> 8)& 0xff);       //Envio 8 bits más significativos EjeYIz
        UARTCharPut(UART5_BASE,DatoYIzda & 0xff);             //Envio 8 bits menos significativos EjeYIz
        //JOYSTICK DCHA
        //Enviar EjeX DCHA
        DatoXDcha = pui32ADC0Value[2];
        UARTprintf("\t\tEjeXDcha/16 Enviado: %d ", DatoXDcha);
        UARTCharPut(UART5_BASE, (DatoXDcha >> 8)& 0xff);      //Envio 8 bits más significativos EjeXDer
        UARTCharPut(UART5_BASE, DatoXDcha & 0xff);            //Envio 8 bits menos significativos EjeXDer
        //Enviar EjeY DCHA
        DatoYDcha = pui32ADC0Value[3];
        UARTprintf("\t\tEjeYDcha/16 Enviado: %d",DatoYDcha);
        UARTCharPut(UART5_BASE, (DatoYDcha >> 8)& 0xff);      //Envio 8 bits más significativos EjeYDer
        UARTCharPut(UART5_BASE, DatoYDcha & 0xff);            //Envio 8 bits menos significativos EjeYDer
        //Enviar Motor Elegido
        UARTprintf("\t\tMotorElegido: %d\n", MotorElegido);
        UARTCharPut(UART5_BASE,MotorElegido);                 //Envio Motor Elegido


        //MANEJO DE LA PANTALLA
        Lee_pantalla();                                    //Obtener pulsación de la pantalla
        Nueva_pantalla(16,16,16);                          //Tras la pantalla de bienvenida, dibujar la pantalla de manejo del robot

        ComGradient(0,0,GRIS_CLARO,0,240,GRIS_OSCURO);
        ComColor(255,0,0);
        ComFgcolor(200, 200, 10);

        //DIBUJAR LÍNEA DIVISORIA
        ComColor(0,0,0); //Negro
        ComLine(0.5*HSIZE, 0, 0.5*HSIZE, VSIZE, 3);

        //DIBUJO DE LAS POSICIONES DE LOS JOYSTICK MAPEADAS EN LA PANTALLA
        //JOYSTICK IZQUIERDO
        ComColor(0,0,0);
        ComCirculo(0.25*HSIZE, 0.7*VSIZE, 45);

        ComColor(255,0,0);
        PixelX=mIzdaX*DatoXIzda+bIzdaX;
        PixelY=mIzdaY*DatoYIzda+bIzdaY;
        ComCirculo(PixelX, PixelY, 20);          //Dibujar joystick en la posición mapeada

        ComTXT(0.25*HSIZE, 0.45*VSIZE, 22, OPT_CENTERX,"Avanzar");
        ComTXT(0.25*HSIZE, 0.87*VSIZE, 22, OPT_CENTERX,"Retroceder");
        ComTXT(0.08*HSIZE, 0.67*VSIZE, 22, OPT_CENTERX,"Izquierda");
        ComTXT(0.42*HSIZE, 0.67*VSIZE, 22, OPT_CENTERX,"Derecha");

        //JOYSTICK DERECHO
        ComColor(0,0,0);
        ComCirculo(0.75*HSIZE, 0.7*VSIZE, 45);

        ComColor(255,0,0);
        PixelX=mDchaX*DatoXDcha+bDchaX;
        PixelY=mDchaY*DatoYDcha+bDchaY;
        ComCirculo(PixelX, PixelY, 20);          //Dibujar joystick en la posición mapeada

        ComTXT(0.58*HSIZE, 0.67*VSIZE, 22, OPT_CENTERX,"Sentido 1");
        ComTXT(0.92*HSIZE, 0.67*VSIZE, 22, OPT_CENTERX,"Sentido 2");

        //DIBUJAR BRAZO DEL ROBOT
        ComColor(0,0,0); //Negro
        ComLine(0.75*HSIZE-40, 0.25*VSIZE, 0.75*HSIZE-40, 0.05*VSIZE, 3);

        ComColor(255,233,0);
        ComRect(0.65*HSIZE-40,0.35*VSIZE,0.85*HSIZE-40,0.45*VSIZE,true);  //Rectángulo torreta (Motor 5)
        ComRect(0.75*HSIZE-20-40,0.25*VSIZE,0.75*HSIZE+20-40,0.35*VSIZE,true); //Rectángulo hombro (Motor 4)

        ComColor(0,0,0); //Negro
        ComLine(0.75*HSIZE-40, 0.1*VSIZE, HSIZE-40, 0.1*VSIZE, 3);

        ComColor(255,233,0);  //Amarillo
        ComRect(0.75*HSIZE-20-40,0.05*VSIZE,0.75*HSIZE+20-40,0.15*VSIZE,true);  //Rectángulo codo (Motor 3)
        ComRect(0.75*HSIZE-40+50,0.05*VSIZE,0.75*HSIZE-40+90,0.15*VSIZE,true);  //Rectángulo muñeca (Motor 2)

        ComRect(HSIZE-40,0.05*VSIZE,HSIZE-40+20,0.15*VSIZE,true); //Rectángulo garra (Motor 1)
        ComColor(0,0,0); //Negro
        ComLine(HSIZE-42, 0.05*VSIZE, HSIZE-42, 0.15*VSIZE, 3);
        ComLine(HSIZE-42, 0.05*VSIZE-3, HSIZE-18, 0.05*VSIZE-3, 3);
        ComLine(HSIZE-42, 0.15*VSIZE+3, HSIZE-18, 0.15*VSIZE+3, 3);

        //ELEGIR QUÉ ARTICULACIÓN SE DIBUJA DE ROJO
        ComColor(255,0,0);
        switch(MotorElegido){
        case 1:
            ComRect(HSIZE-40,0.05*VSIZE,HSIZE-40+20,0.15*VSIZE,true); //Rectángulo garra (Motor 1)
            break;
        case 2:
            ComRect(0.75*HSIZE-40+50,0.05*VSIZE,0.75*HSIZE-40+90,0.15*VSIZE,true);  //Rectángulo muñeca (Motor 2)
            break;
        case 3:
            ComRect(0.75*HSIZE-20-40,0.05*VSIZE,0.75*HSIZE+20-40,0.15*VSIZE,true);  //Rectángulo codo (Motor 3)
            break;
        case 4:
            ComRect(0.75*HSIZE-20-40,0.25*VSIZE,0.75*HSIZE+20-40,0.35*VSIZE-10,true); //Rectángulo hombro (Motor 4)
            break;
        case 5:
            ComRect(0.65*HSIZE-40,0.35*VSIZE,0.85*HSIZE-40,0.45*VSIZE,true);  //Rectángulo torreta (Motor 5)
            break;
        }

        //DIBUJO DE BORDES DE CADA RECTÁNGULO Y ACTUALIZACIÓN DE MotorElegido
        ComColor(0,0,0);
        ComRect(0.65*HSIZE-40-5,0.35*VSIZE-5,0.85*HSIZE-40+5,0.45*VSIZE+5,false);  //Rectángulo torreta (Motor 5)
        if(POSX>0.65*HSIZE-40-5 && POSX<0.85*HSIZE-40+5 && POSY>0.35*VSIZE-5 && POSY<0.45*VSIZE+5) //Pulsando en el rectangulo
        {
            MotorElegido = 5;
        }

        ComColor(0,0,0);
        ComRect(0.75*HSIZE-20-40-5,0.25*VSIZE-5,0.75*HSIZE+20-40+5,0.35*VSIZE-5,false);
        if(POSX>0.75*HSIZE-20-40-5 && POSX<0.75*HSIZE+20-40+5 && POSY>0.25*VSIZE-5 && POSY<0.35*VSIZE-5) //Pulsando en el rectangulo
        {
            MotorElegido = 4;
        }

        ComColor(0,0,0);
        ComRect(0.75*HSIZE-20-40-5,0.05*VSIZE-5,0.75*HSIZE+20-40+5,0.15*VSIZE+5,false);
        if(POSX>0.75*HSIZE-20-40-5 && POSX<0.75*HSIZE+20-40+5 && POSY>0.05*VSIZE-5 && POSY<0.15*VSIZE+5) //Pulsando en el rectangulo
        {
            MotorElegido = 3;
        }

        ComColor(0,0,0);
        ComRect(0.75*HSIZE-40+50-5,0.05*VSIZE-5,0.75*HSIZE-40+90+5,0.15*VSIZE+5,false);
        if(POSX>0.75*HSIZE-40+50 && POSX<0.75*HSIZE-40+90 && POSY>0.05*VSIZE && POSY<0.15*VSIZE) //Pulsando en el rectangulo
        {
            MotorElegido = 2;
        }

        ComColor(0,0,0);
        if(POSX>HSIZE-40 && POSX<HSIZE-40+20 && POSY>0.05*VSIZE && POSY<0.15*VSIZE) //Pulsando en el rectangulo
        {
            MotorElegido = 1;
        }

        //DIBUJAR COCHE DEL ROBOT
        ComColor(0,0,255);
        ComRect(0.25*HSIZE-40,0.05*VSIZE,0.25*HSIZE+40,0.40*VSIZE,true);
        ComColor(155,155,155);
        ComRect(0.20*HSIZE-10,0.1*VSIZE-8,0.20*HSIZE+10,0.1*VSIZE+30-3,true);
        ComRect(0.20*HSIZE-10,0.1*VSIZE-10+50,0.20*HSIZE+10,0.1*VSIZE+30-5+50,true);

        ComRect(0.30*HSIZE-10,0.1*VSIZE-8,0.30*HSIZE+10,0.1*VSIZE+30-3,true);
        ComRect(0.30*HSIZE-10,0.1*VSIZE-10+50,0.30*HSIZE+10,0.1*VSIZE+30-5+50,true);

        ComColor(0,0,0);
        ComRect(0.25*HSIZE-40-5,0.05*VSIZE-5,0.25*HSIZE+40+5,0.40*VSIZE+5,false);


        Dibuja();

    }
}
