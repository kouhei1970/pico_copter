/*
 * radio.cpp
 * RC signal receive
*/
#include "radio.hpp"

//グローバル変数の宣言
uint16_t Chdata[18];

//ローカル変数の宣言
uint16_t chars_rxed = 0;
uint16_t data_num=0;
uint8_t sbus_data[25];
uint8_t ch;

//ローカル関数の宣言
void on_uart_rx(); 


void radio_init(void){

    /// シリアル通信の設定

    // UARTを基本の通信速度で設定
    uart_init(UART_ID, 2400);

    // 指定のGPIOをUARTのTX、RXピンに設定する
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    //指定のUARTを指定の通信速度に設定する
    int actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    //UART flow control CTS/RTSを使用しない設定
    uart_set_hw_flow(UART_ID, false, false);

    //通信フォーマットの設定
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

// RX interrupt handler
void on_uart_rx(void) {
    //short data;
    
    //gpio_put(25, 1);
    
    while (uart_is_readable(UART_ID)) {
        ch = uart_getc(UART_ID);
        if(ch==0x0f&&chars_rxed==0){
            sbus_data[chars_rxed]=ch;
            //printf("%02X ",ch);
            chars_rxed++;
        }
        else if(chars_rxed>0){
            sbus_data[chars_rxed]=ch;
            //printf("%02X ",ch);
            chars_rxed++;            
        }
        
        switch(chars_rxed){
            case 3:
                Chdata[0]=(sbus_data[1]|(sbus_data[2]<<8)&0x07ff);
                //printf("%04d ",Chdata[0]);
                break;
            case 4:
                Chdata[1]=(sbus_data[3]<<5|sbus_data[2]>>3)&0x07ff;
                //printf("%04d ",Chdata[1]);
                break;
            case 6:
                Chdata[2]=(sbus_data[3]>>6|sbus_data[4]<<2|sbus_data[5]<<10)&0x07ff;
                //printf("%04d ",Chdata[2]);
                break;
            case 7:
                Chdata[3]=(sbus_data[6]<<7|sbus_data[5]>>1)&0x07ff;
                //printf("%04d ",Chdata[3]);
                break;
            case 8:
                Chdata[4]=(sbus_data[7]<<4|sbus_data[6]>>4)&0x07ff;
                //printf("%04d ",Chdata[4]);
                break;
            case 10:
                Chdata[5]=(sbus_data[7]>>7|sbus_data[8]<<1|sbus_data[9]<<9)&0x07ff;
                //printf("%04d ",Chdata[5]);
                break;
            case 11:
                Chdata[6]  = ((sbus_data[9]>>2|sbus_data[10]<<6) & 0x07FF);
                //printf("%04d ",Chdata[6]);
                break;
            case 12:
                Chdata[7]  = ((sbus_data[10]>>5|sbus_data[11]<<3) & 0x07FF);
                //printf("%04d ",Chdata[7]);
                break;
            case 14:
                Chdata[8]  = ((sbus_data[12]|sbus_data[13]<< 8) & 0x07FF);
                //printf("%04d ",Chdata[8]);
                break;
            case 15:
                Chdata[9]  = ((sbus_data[13]>>3|sbus_data[14]<<5) & 0x07FF);
                //printf("%04d ",Chdata[9]);
                break;
            case 16:
                Chdata[10] = ((sbus_data[14]>>6|sbus_data[15]<<2|sbus_data[16]<<10) & 0x07FF);
                //printf("%04d ",Chdata[10]);
                break;
            case 17:
                Chdata[11] = ((sbus_data[16]>>1|sbus_data[17]<<7) & 0x07FF);
                //printf("%04d ",Chdata[11]);
                break;
            case 19:
                Chdata[12] = ((sbus_data[17]>>4|sbus_data[18]<<4) & 0x07FF);
                //printf("%04d ",Chdata[12]);
                break;
            case 21:
                Chdata[13] = ((sbus_data[18]>>7|sbus_data[19]<<1|sbus_data[20]<<9) & 0x07FF);
                //printf("%04d ",Chdata[13]);
                break;
            case 22:
                Chdata[14] = ((sbus_data[20]>>2|sbus_data[21]<<6) & 0x07FF);
                //printf("%04d ",Chdata[14]);
                break;
            case 23:
                Chdata[15] = ((sbus_data[21]>>5|sbus_data[22]<<3) & 0x07FF);
                //printf("%04d ",Chdata[15]);
                break;
            case 24:
                Chdata[16] = sbus_data[23];
                //printf("%04x ",Chdata[16]);
                break;
        }

        if(chars_rxed==25){
            Chdata[17]=sbus_data[24];
            //printf("%04d ",Chdata[17]);
            //printf("\n");
            chars_rxed=0;
        }
    }
}

