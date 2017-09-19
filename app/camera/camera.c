/*
 * camera.c
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */


#include "camera.h"
#include "network/wiznet.h"
#include "network/Ethernet/wizchip_conf.h"
#include "network/Ethernet/socket.h"
#include "tfcard/tfcard.h"
#include "fatfs/ff.h"

#define CAMERA_SOCK 0
#define ANY_PORT 50000
#define DATA_BUF_SIZE 1024

const uint8_t c_des_ip[4] = {192,168,1,10};
const uint16_t c_des_port = 4444;

typedef enum{
    START = 0,
    IMAGE_CAPUTED,
    IMAGE_SIZE_REVING,
    IMAGE_RECIEVING,
    IMAGE_RECIEVED
}camera_process_type;

camera_process_type camera_stat = START;

uint8_t camera_init(void){

    wizchip_spi_configuration();
    wiznet_init();

    tcp_client_init();
}
int32_t tcpc_run(char *buffer){
    int32_t ret; // return value for SOCK_ERRORs
    uint16_t size = 0;
    uint32_t write_size = 0;
    int32_t image_size = 0;
    int32_t rev_image_size = 0;
    FIL image_fil;
    switch(getSn_SR(CAMERA_SOCK)){
        case SOCK_ESTABLISHED:
            if(getSn_IR(CAMERA_SOCK) & Sn_IR_CON)   // Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
            {
                setSn_IR(CAMERA_SOCK, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
            }


            ret = send(CAMERA_SOCK, "image_capture", strlen("image_capture")+1);
            if( ret <0){
                close(CAMERA_SOCK); // socket close
                return ret;
            }
            while(1){
                if((size = getSn_RX_RSR(CAMERA_SOCK)) > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
                {
                    if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE; // DATA_BUF_SIZE means user defined buffer size (array)
                    ret = recv(CAMERA_SOCK, buffer, size); // Data Receive process (H/W Rx socket buffer -> User's buffer)
                    if(ret <= 0) return ret;

                    switch(camera_stat){
                        case START:
                            if(strcmp(buffer, "image_captured") == 0)
                            {
                                ret = send(CAMERA_SOCK, "image_capture", strlen("image_capture")+1);
                                if( ret <0){
                                    close(CAMERA_SOCK); // socket close
                                    return ret;
                                }
                                if(ret == strlen("image_capture")+1){
                                    camera_stat = IMAGE_SIZE_REVING;
                                    rev_image_size = 0;
                                    //write file to tf card
                                }
                            }
                            break;
                        case IMAGE_SIZE_REVING:
                            image_size = atoi(buffer);
                            ret = send(CAMERA_SOCK, "image_recieving", strlen("image_recieving")+1);
                            if( ret <0 || image_size <0){
                                close(CAMERA_SOCK); // socket close
                                return ret;
                            }
                            else{
                                camera_stat = IMAGE_RECIEVING;
                                if((ret = f_open(&image_fil,"image.jpg", FA_WRITE|FA_CREATE_ALWAYS)) != FR_OK){
                                    close(CAMERA_SOCK);
                                    return ret;
                                }

                            }
                            break;
                        case IMAGE_RECIEVING:
                            if(rev_image_size<image_size){
                                f_write(&image_fil, buffer, size, &write_size);
                                rev_image_size+=write_size;
                                if(rev_image_size == image_size){
                                    f_close(&image_fil);

                                    camera_stat = IMAGE_RECIEVED;
                                    ret = send(CAMERA_SOCK, "image_recieved", strlen("image_recieved")+1);
                                }
                                else{
                                    ret = send(CAMERA_SOCK, "image_recieving", strlen("image_recieving")+1);
                                    if(ret == strlen("image_recieving")+1 && image_size>0){
                                        camera_stat = IMAGE_RECIEVING;
                                    }
                                }

                            }
                            break;
                        case IMAGE_RECIEVED:
                            printf("image recieved!\n");
                            close(CAMERA_SOCK);
                            return ret;
                            break;
                        default:
                            break;

                    }
                }
            }

            break;
        case SOCK_CLOSE_WAIT:
            if((ret=disconnect(CAMERA_SOCK)) != SOCK_OK) return ret;
            break;
        case SOCK_INIT:
            if( (ret = connect(CAMERA_SOCK, c_des_ip, c_des_port)) != SOCK_OK) return ret;  //  Try to TCP connect to the TCP server (destination)
            break;
        case SOCK_CLOSED:
            close(CAMERA_SOCK);
            if((ret=socket(CAMERA_SOCK, Sn_MR_TCP, ANY_PORT, 0x00)) != CAMERA_SOCK) return ret; // TCP socket open with 'any_port' port number
            break;
        default:
            break;

    }
}
uint8_t capture_image(void){
    uint16_t any_port =     50000;

    int32_t ret; // return value for SOCK_ERRORs
}


