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

void configuraPWM50Hz(uint32_t PeripheralPWM, uint32_t PeripheralGPIO, uint32_t BasePWM, uint32_t PinPWM_GPIO, uint32_t BasePuertoGPIO, uint32_t PinesGPIO, uint32_t GeneradorPWM, uint32_t OutPWM, uint32_t OutPWM_BIT){
    //Configuracion de todos los PWM
    SysCtlPeripheralEnable(PeripheralPWM);  //Habilitamos PWM
    SysCtlPeripheralEnable(PeripheralGPIO); //Habilitamos puerto pin PWM
    PWMClockSet(BasePWM,PWM_SYSCLK_DIV_64);   // al PWM le llega un reloj de 1.875MHz
    GPIOPinConfigure(PinPWM_GPIO);          //Configurar el pin a PWM
    GPIOPinTypePWM(BasePuertoGPIO, PinesGPIO);
    PWMGenConfigure(BasePWM, GeneradorPWM, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    int PeriodoPWM=37499; // 50Hz  a 1.875MHz

    PWMGenPeriodSet(BasePWM, GeneradorPWM, PeriodoPWM); //frec:50Hz
    PWMPulseWidthSet(BasePWM, OutPWM, 10);   //Inicialmente, velocidad de motor 0
    PWMGenEnable(BasePWM, GeneradorPWM);     //Habilita el generador especificado
    PWMOutputState(BasePWM, OutPWM_BIT , true);    //Habilita la salida especificada
}

void Avanzar(int velMotorMapeada){
    //Avance del coche: todos los motores (los de la derecha y los de la izquierda) van hacia delante
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0);              //IN1Dcha a 0     Debido a que se han conectado las alimentaciones de los motores
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3);     //IN2Dcha a 1     de los dos lados del coche de forma contraria, las señales quedan invertidas
    GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_3,GPIO_PIN_3);     //IN1Izda a 1
    GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_2,0);              //IN2Izda a 0

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, velMotorMapeada); //Dar velocidad a los motores de la DCHA
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, velMotorMapeada); //Dar velocidad a los motores de la IZDA
}

void Retroceder(int velMotorMapeada){
    //Retroceso del coche: todos los motores (los de la derecha y los de la izquierda) van hacia atrás
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_PIN_5);     //IN1Dcha a 1     Debido a que se han conectado las alimentaciones de los motores
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);              //IN2Dcha a 0     de los dos lados del coche de forma contraria, las señales quedan invertidas
    GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_3,0);              //IN1Izda a 0
    GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_2,GPIO_PIN_2);     //IN2Izda a 1

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, velMotorMapeada); //Dar velocidad a los motores de la DCHA
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, velMotorMapeada); //Dar velocidad a los motores de la IZDA
}

void GiroDerecha(int velMotorMapeada){
  //Giro a la derecha del coche: todos los motores (los de la derecha y los de la izquierda) van hacia delante,
  //pero los motores de la derecha van más rápido que los de la izquierda
       GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_PIN_5);  //IN1Dcha a 1     Debido a que se han conectado las alimentaciones de los motores
       GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);           //IN2Dcha a 0     de los dos lados del coche de forma contraria, las señales quedan invertidas
       GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_3,GPIO_PIN_3);  //IN1Izda a 1
       GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_2,0);           //IN2Izda a 0

       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, velMotorMapeada); //Dar velocidad a los motores de la DCHA
       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, velMotorMapeada); //Dar velocidad a los motores de la IZDA
}

void GiroIzquierda(int velMotorMapeada){
  //Giro a la izquierda del coche: todos los motores (los de la derecha y los de la izquierda) van hacia delante,
  //pero los motores de la izquierda van más rápido que los de la derecha
       GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0);          //IN1Dcha a 0     Debido a que se han conectado las alimentaciones de los motores
       GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3);  //IN2Dcha a 1     de los dos lados del coche de forma contraria, las señales quedan invertidas
       GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_3,0);           //IN1Izda a 0
       GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_2,GPIO_PIN_2);  //IN2Izda a 1

       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, velMotorMapeada); //Dar velocidad a los motores de la DCHA
       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, velMotorMapeada); //Dar velocidad a los motores de la IZDA
}

void Parar(){
    //Detener el coche: es indiferente los sentidos de giro de los lados del coche: la velocidad es cero para ambos
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0);           //IN1Dcha a 0     Debido a que se han conectado las alimentaciones de los motores
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3);  //IN2Dcha a 1     de los dos lados del coche de forma contraria, las señales quedan invertidas
    GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_3,0);           //IN1Izda a 0
    GPIOPinWrite(GPIO_PORTH_BASE,GPIO_PIN_2,GPIO_PIN_2);  //IN2Izda a 1

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 10); //Dar velocidad a los motores de la DCHA    CON 0 EL COCHE NO SE DETIENE, CON 10 SÍ
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 10); //Dar velocidad a los motores de la IZDA    CON 0 EL COCHE NO SE DETIENE, CON 10 SÍ
}
