//codigo finAL TIVA
// prueba1.c - TM4C1294XL + HC-SR04 + UART0 + LEDs + Buzzer PB5 (no bloqueante)
// - TRIG=PA6 (salida)
// - ECHO=PA7 (entrada, con divisor 5V->3V3)
// - UART0=115200 (PA0 RX, PA1 TX) via USB-ICDI
// - LEDs: PN1, PN0, PF4
// - Buzzer: PB5 (se activa 2s al recibir 'b' o 'B' por UART0)

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

// ---- Pines HC-SR04 ----
#define TRIG_PORT   GPIO_PORTA_BASE
#define TRIG_PIN    GPIO_PIN_6
#define ECHO_PORT   GPIO_PORTA_BASE
#define ECHO_PIN    GPIO_PIN_7

// ---- LEDs ----
#define LEDN_PORT   GPIO_PORTN_BASE
#define LEDN_PN1    GPIO_PIN_1
#define LEDN_PN0    GPIO_PIN_0
#define LEDF_PORT   GPIO_PORTF_BASE
#define LEDF_PF4    GPIO_PIN_4

// ---- Buzzer ----
#define BUZZ_PORT   GPIO_PORTB_BASE
#define BUZZ_PIN    GPIO_PIN_5
#define BUZZ_MS     2000      // duración del buzzer tras 'b', en ms
#define LOOP_MS     100       // periodo aproximado del loop principal
#define BUZZ_TICKS  (BUZZ_MS/LOOP_MS)

// ===== Utilidades =====
static inline void delay_us(uint32_t us){
  // 120 MHz: SysCtlDelay consume 3 ciclos -> 40 ≈ 1us
  while(us--) SysCtlDelay(40);
}

// ===== UART0 (USB-ICDI) =====
static void UART0_Init(uint32_t sysclk, uint32_t baud){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  UARTConfigSetExpClk(UART0_BASE, sysclk, baud,
    UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}
static void uart_putc(char c){ UARTCharPut(UART0_BASE, c); }
static void uart_puts(const char *s){ while(*s) UARTCharPut(UART0_BASE, *s++); }
static void uart_putu(uint32_t v){
  char b[11]; int i=10; b[i--]='\0'; if(!v){ uart_putc('0'); return; }
  while(v && i>=0){ b[i--]='0'+(v%10); v/=10; } uart_puts(&b[i+1]);
}

// ===== LEDs =====
static void LedsInit(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
  GPIOPinTypeGPIOOutput(LEDN_PORT, LEDN_PN1 | LEDN_PN0);
  GPIOPinTypeGPIOOutput(LEDF_PORT, LEDF_PF4);
  GPIOPinWrite(LEDN_PORT, LEDN_PN1 | LEDN_PN0, 0);
  GPIOPinWrite(LEDF_PORT, LEDF_PF4, 0);
}
static void ShowDistanceLEDs(int32_t cm){
  uint8_t n=0,f=0;
  if(cm<0){ static uint8_t t; t^=1; n=t?(LEDN_PN1|LEDN_PN0):0; f=t?LEDF_PF4:0; }
  else if(cm>10){ /* apagado */ }
  else if(cm>8){ n=LEDN_PN1; }
  else if(cm>6){ n=LEDN_PN1|LEDN_PN0; }
  else { n=LEDN_PN1|LEDN_PN0; f=LEDF_PF4; }
  GPIOPinWrite(LEDN_PORT, LEDN_PN1 | LEDN_PN0, n);
  GPIOPinWrite(LEDF_PORT, LEDF_PF4, f);
}

// ===== Buzzer PB5 =====
static void BuzzerInit(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
  GPIOPinTypeGPIOOutput(BUZZ_PORT, BUZZ_PIN);
  GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);
}
static void BuzzerOn(void){  GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, BUZZ_PIN); }
static void BuzzerOff(void){ GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0); }

// ===== HC-SR04 =====
static void HCSR04_Init(void){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
  GPIOPinTypeGPIOOutput(TRIG_PORT, TRIG_PIN);
  GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
  GPIOPinTypeGPIOInput(ECHO_PORT, ECHO_PIN);
  // pull-down para estado definido en ECHO
  GPIOPadConfigSet(ECHO_PORT, ECHO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}
static int32_t HCSR04_Read_cm(void){
  // pulso de 10us en TRIG
  GPIOPinWrite(TRIG_PORT, TRIG_PIN, TRIG_PIN); delay_us(10);
  GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);

  // esperar subida (timeout ~30ms)
  uint32_t wait = 30000;
  while((GPIOPinRead(ECHO_PORT,ECHO_PIN)==0) && wait--) delay_us(1);
  if(!wait) return -1;

  // medir ancho HIGH (timeout ~40ms)
  uint32_t width_us=0; wait=40000;
  while((GPIOPinRead(ECHO_PORT,ECHO_PIN)!=0) && wait--){ delay_us(1); width_us++; }
  if(!wait) return -1;

  return (int32_t)(width_us/58U); // ~58 us por cm
}

int main(void){
  // 120 MHz (PLL + XTAL 25 MHz)
  uint32_t sysclk = SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
    SYSCTL_USE_PLL    | SYSCTL_CFG_VCO_480, 120000000);

  UART0_Init(sysclk,115200);
  LedsInit();
  BuzzerInit();
  HCSR04_Init();

  uart_puts("\r\nTM4C1294XL HC-SR04 + UART0 + Buzzer\r\n");
  uart_puts("Envia 'b' por UART0 para buzzer 2s. Formato de salida: DIST,<cm>\r\n");

  int buzzer_ticks = 0;  // cuenta atrás en unidades de LOOP_MS

  while(1){
    // 1) Revisa UART (no bloqueante)
    int ch = UARTCharGetNonBlocking(UART0_BASE);
    if(ch != -1){
      if(ch=='b' || ch=='B'){
        buzzer_ticks = BUZZ_TICKS;   // arranca buzzer 2s sin bloquear
      }
    }

    // 2) Maneja buzzer (no bloqueante)
    if(buzzer_ticks > 0){ BuzzerOn();  buzzer_ticks--; }
    else                { BuzzerOff(); }

    // 3) Medición HC-SR04
    int32_t d = HCSR04_Read_cm();
    ShowDistanceLEDs(d);

    // 4) Reporte por UART
    uart_puts("DIST,");
    if(d<0) uart_puts("-1");
    else    uart_putu((uint32_t)d);
    uart_puts("\r\n");

    // 5) Periodo aproximado del loop
    // HCSR04_Read_cm() ya tarda decenas de ms; ajusta a ~100 ms totales
    // (usa un delay pequeño para estabilizar)
    delay_us(100000); // 100 ms
  }
}
