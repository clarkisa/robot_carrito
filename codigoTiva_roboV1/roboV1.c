// tiva_lab5_uart_usb.c — TM4C1294XL (UART0 por USB DEBUG)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "inc/hw_memmap.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#define SW_PORT   GPIO_PORTJ_BASE
#define SW1_PIN   0x01u   // PJ0
#define SW2_PIN   0x02u   // PJ1

#define BUZZ_PORT GPIO_PORTB_BASE
#define BUZZ_PIN  0x40u   // PB6

static void delay_ms(uint32_t sysclk, uint32_t ms){
    MAP_SysCtlDelay((sysclk/3000u)*ms);
}

static void uart0_init(uint32_t sysclk){
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, 0x01u | 0x02u); // PA0|PA1

    MAP_UARTConfigSetExpClk(UART0_BASE, sysclk, 115200,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}

static void uart0_puts(const char *s){
    while(*s) { MAP_UARTCharPut(UART0_BASE, (uint8_t)*s++); }
}

static int  uart0_getc_nb(void){
    int32_t v = MAP_UARTCharGetNonBlocking(UART0_BASE);
    return (v == -1) ? -1 : (int)(char)v;
}

static bool uart0_readline_nb(char *buf, uint32_t maxlen){
    static uint32_t idx=0;
    int ch;
    while((ch = uart0_getc_nb()) != -1){
        char c = (char)ch;
        if(c=='\r' || c=='\n'){
            if(idx>0){ buf[idx]='\0'; idx=0; return true; }
        }else{
            if(idx<maxlen-1) buf[idx++]=c; else idx=0;
        }
    }
    return false;
}

static void switches_init(void){
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    MAP_GPIOPinTypeGPIOInput(SW_PORT, SW1_PIN | SW2_PIN);
    MAP_GPIOPadConfigSet(SW_PORT, SW1_PIN|SW2_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

static void buzzer_init(void){
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    MAP_GPIOPinTypeGPIOOutput(BUZZ_PORT, BUZZ_PIN);
    MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);
}

int main(void){
    uint32_t sysclk = MAP_SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
        SYSCTL_USE_PLL    | SYSCTL_CFG_VCO_240, 120000000);

    switches_init();
    buzzer_init();
    uart0_init(sysclk);

    uint8_t sw1_prev=1, sw2_prev=1;
    char line[64];

    while(1){
        uint8_t sw = MAP_GPIOPinRead(SW_PORT, SW1_PIN|SW2_PIN);
        uint8_t sw1 = (sw & SW1_PIN)?1:0;
        uint8_t sw2 = (sw & SW2_PIN)?1:0;

        if(sw1_prev==1 && sw1==0){ delay_ms(sysclk,10); if((MAP_GPIOPinRead(SW_PORT,SW1_PIN)&SW1_PIN)==0){ uart0_puts("motor1\n"); while((MAP_GPIOPinRead(SW_PORT,SW1_PIN)&SW1_PIN)==0){} } }
        if(sw2_prev==1 && sw2==0){ delay_ms(sysclk,10); if((MAP_GPIOPinRead(SW_PORT,SW2_PIN)&SW2_PIN)==0){ uart0_puts("motor2\n"); while((MAP_GPIOPinRead(SW_PORT,SW2_PIN)&SW2_PIN)==0){} } }
        sw1_prev=sw1; sw2_prev=sw2;

        if(uart0_readline_nb(line, sizeof(line))){
            for(char *p=line; *p; ++p) *p = (char)tolower((unsigned char)*p);
            if(strcmp(line,"buzzer")==0){
                MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, BUZZ_PIN);
                delay_ms(sysclk,2000);
                MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);
                uart0_puts("OK BUZZER 2s\n");
            }
        }
    }
}
