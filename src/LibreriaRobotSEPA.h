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

extern void configuraPWM50Hz(uint32_t PeripheralPWM, uint32_t PeripheralGPIO, uint32_t BasePWM, uint32_t PinPWM_GPIO, uint32_t BasePuertoGPIO, uint32_t PinesGPIO, uint32_t GeneradorPWM, uint32_t OutPWM, uint32_t OutPWM_BIT);
extern void Avanzar(int velMotorMapeada);
extern void Retroceder(int velMotorMapeada);
extern void GiroDerecha(int velMotorMapeada);
extern void GiroIzquierda();
extern void Parar();

