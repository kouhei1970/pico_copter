
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 100000
#define DATA_BITS 8
#define STOP_BITS 2
#define PARITY UART_PARITY_EVEN
#define CH1MAX 1771
#define CH1MIN 278
#define CH2MAX 1756
#define CH2MIN 316
#define CH3MAX 1721
#define CH3MIN 312
#define CH4MAX 1745
#define CH4MIN 291
#define CH5MAX 1904
#define CH5MIN 144
#define CH6MAX 1904
#define CH6MIN 144


//０番と1番ピンに接続
#define UART_TX_PIN 0
#define UART_RX_PIN 1

//グローバル変数の宣言
extern uint16_t Chdata[18];

//グローバル関数の宣言
void radio_init(void);
