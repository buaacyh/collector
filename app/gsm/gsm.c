/*
 * gsm.c
 *
 *  Created on: 2017年9月13日
 *      Author: saber
 */


/* DMA Control Table */
#include "gsm.h"
#include "uart/uart.h"
#include "clocks/clocks.h"

#ifdef ewarm
#pragma data_alignment=1024
#else
#pragma DATA_ALIGN(controlTable, 1024)
#endif

#define MAX_WAIT_SECONDS 10

char *COMMON="[common]";
char *AT_IPR="AT+IPR=921600\r\n";
char *AT_CGSOCKCONT_IP_COM="AT+CGSOCKCONT=1,\"IP\",\"cmnet\"\r\n";
char *AT_CSOCKSETPN="AT+CSOCKSETPN=1\r\n";
char *AT_NETOPEN="AT+NETOPEN\r\n";
char *AT_NETCLOSE="AT+NETCLOSE\r\n";
char *AT_CIPCLOSE="AT+CIPCLOSE=0\r\n";
char *AT_CIPSEND="AT+CIPSEND=0,\r\n";
char *AT_CNMI="AT+CNMI=2,2\r\n";
char *AT_CGREG="AT+CGREG?\r\n";
char *AT_CIPHEAD="AT+CIPHEAD=0\r\n";
char *AT_CIPSRIP="AT+CIPSRIP=0\r\n";
char *AT_CSCLK="AT+CSCLK=1\r\n";
char *AT_CGSOCKCONT="AT+CGSOCKCONT=1,\"IP\",\"cmnet\"\r\n";
char *IP="120.77.34.29";
char *COM="8989";

char *AT_CIPOPEN_TCP="AT+CIPOPEN=0,\"TCP\",\"%s\",%s\r\n";



uint8_t controlTable[1024];

uint8_t config_buffer[MAX_RX_BUFFER];
uint8_t gsm_rec_buffer[MAX_RX_BUFFER];       //接收缓存器
uint8_t tx_buffer[MAX_TX_BUFFER];       //发送缓存器
uint16_t rec_index=0;
bool msg_reved = false;


const eUSCI_UART_Config gsm_uart_config=
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // 选择SMCLK时钟为源，6MHz
     3,                                      // bps=115200;
     4,                                       // UCxBRF = 0
     2,                                      // UCxBRS = 37
     EUSCI_A_UART_NO_PARITY,                  // No Parity
     EUSCI_A_UART_LSB_FIRST,                  // LSB First
     EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
     EUSCI_A_UART_MODE,                       // UART mode
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  //
};



void gsm_on()
{
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    delay_ms(500);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
    delay_ms(15000);                        //开机需要20s寻找网络
}

void gsm_off()
{
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    delay_ms(1000);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
    delay_ms(1000);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    delay_ms(1000);                        //关机需要20s
}

void gsm_dma_init()
{
    MAP_DMA_enableModule();
    MAP_DMA_setControlBase(controlTable);
    MAP_DMA_assignChannel(DMA_CH2_EUSCIA1TX);

    /* Disabling channel attributes */
    MAP_DMA_disableChannelAttribute(DMA_CH2_EUSCIA1TX, UDMA_ATTR_ALL);

    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH2_EUSCIA1TX,
           UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);

}

void gsm_conf_init()
{
    //msg_send(AT_IPR);
    _msg_send(AT_CGREG);
    _msg_send(AT_CSCLK);                //使能睡眠模式
    clear_buffer();
}


void set_gsm_net(char *ip,char *com)
{
    uint8_t ip_buffer[50];
    sprintf((char*)ip_buffer,AT_CIPOPEN_TCP,ip,com);
    _msg_send(ip_buffer);
}

void gsm_init()
{
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN6 | GPIO_PIN7 );    //out DTR|RST|POWKEY | GPIO_PIN6 | GPIO_PIN7
    MAP_GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN4 | GPIO_PIN5);     //in RI|STA|NET
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0 | GPIO_PIN6);

    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);      //GSM RI IRQ
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN1);

    MAP_Interrupt_enableInterrupt(INT_PORT2);
    MAP_Interrupt_enableMaster();

    //init uart interface
    uart_config _gsm_uart_conf = {.gpio_group = GPIO_PORT_P2,
         .gpio_pins= GPIO_PIN2 | GPIO_PIN3,
         .port = EUSCI_A1_BASE,
         .pconf = &gsm_uart_config};

    uart_init(&_gsm_uart_conf);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);

    //gsm_on
    delay_ms(20000);
    gsm_dma_init();
    gsm_conf_init();

    //sleep_gsm();
}

void gsm_close(void){
    uart_config _gsm_uart_conf = {.gpio_group = GPIO_PORT_P2,
             .gpio_pins= GPIO_PIN2 | GPIO_PIN3,
             .port = EUSCI_A1_BASE,
             .pconf = &gsm_uart_config};

    uart_close(&_gsm_uart_conf);
}

void _msg_send(char *message)
{
    uint16_t length;
    //isFinished=false;
    memset(tx_buffer, '\0', MAX_TX_BUFFER);
    length=strlen(message);
    strcpy(tx_buffer,message);

    MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH2_EUSCIA1TX,
                           UDMA_MODE_BASIC, tx_buffer,
            (void*) UART_getTransmitBufferAddressForDMA(EUSCI_A1_BASE), length);

    MAP_DMA_enableChannel(DMA_CH2_EUSCIA1TX);
    delay_ms(1000);
}

void msg_send(char *message){

    start_msg_send();
    _msg_send(message);
    stop_msg_send();

}

void start_msg_send(){
    _msg_send(AT_CIPSEND);
}

void stop_msg_send(){
    UART_transmitData(EUSCI_A1_BASE,0x1A);
}

void gsm_connect()
{
    open_net();
    _msg_send(AT_CIPHEAD);
    _msg_send(AT_CIPSRIP);

    set_gsm_net((char*)IP,(char*)COM);                 //配置IP端口
    clear_buffer();
}

//清空接收缓存器，并将接收指针移至接收缓存器头
void clear_buffer()
{
    memset(gsm_rec_buffer, '\0', MAX_RX_BUFFER);
    rec_index=0;
}

char * msg_read()
{
    return gsm_rec_buffer;
}



//GSM RI IRQ
void PORT2_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    if(status & GPIO_PIN1)
    {

    }

}

void EUSCIA1_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t tem;
        tem=MAP_UART_receiveData(EUSCI_A1_BASE);
        gsm_rec_buffer[rec_index++] = tem ;
        if(tem == 0x03){
            msg_reved = true;
        }
    }

}


bool wait_msg_reved(){
    uint8_t i = 0;
    for(i=0;i<MAX_WAIT_SECONDS;++i){
        if(msg_reved){
            msg_reved = false;
            return true;
        }
        else{
            delay_ms(1000);
        }
    }
    return false;
}

