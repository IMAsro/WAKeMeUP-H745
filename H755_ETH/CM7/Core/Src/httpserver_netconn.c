/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/httpser-netconn.c 
  * @author  MCD Application Team
  * @brief   Basic http server implementation using LwIP netconn API  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "string.h"
#include "httpserver_netconn.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdio.h"
#include "math.h"

#include <stdio.h>
#include <stdlib.h>

#include <stm32h7xx_hal_uart.h>
#include <stm32h7xx_hal_gpio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO (osPriorityAboveNormal)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart3;

struct m4_to_m7 {
	float temperature[60];
	float pressure[60];
	float potlevel;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;

	uint8_t led_r;
	uint8_t led_g;
	uint8_t led_b;
	uint8_t led_w;
};

struct m7_to_m4{
	uint32_t rgb_r;
	uint32_t rgb_g;
	uint32_t rgb_b;

	uint8_t w_led_status;
};



struct visualization_data_s {
	float temperature[60];
	float humidity[60];
	float potlevel;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	uint8_t led_r;
	uint8_t led_g;
	uint8_t led_b;
	uint8_t led_w;
} visualization_data;

struct led_data_s {
	uint8_t led_r;
	uint8_t led_g;
	uint8_t led_b;
	uint8_t led_w;
} led_data;

/* Private function prototypes -----------------------------------------------*/
void serve_404(struct netconn *conn);
/* Private functions ---------------------------------------------------------*/

void fillData(struct visualization_data_s *data)
{

	volatile struct m4_to_m7 * const m4_to_m7_ptr = (struct m4_to_m7 *)(0x30040A00);


	for (int i = 0; i < 59; i++){
			data->temperature[i] = data->temperature[i+1];
			data->humidity[i] = data->humidity[i+1];
		}

		data->temperature[59] = m4_to_m7_ptr->temperature[59];

		data->humidity[59] = m4_to_m7_ptr->pressure[59];

		data->potlevel = m4_to_m7_ptr->potlevel;

		data->led_r = led_data.led_r;
		data->led_g = led_data.led_g;
		data->led_b = led_data.led_b;
		data->led_w = led_data.led_w;


		data->accel_x = m4_to_m7_ptr->accel_x;
		data->accel_y = m4_to_m7_ptr->accel_y;
		data->accel_z = m4_to_m7_ptr->accel_z;

		data->gyro_x = m4_to_m7_ptr->gyro_x;
		data->gyro_y = m4_to_m7_ptr->gyro_y;
		data->gyro_z = m4_to_m7_ptr->gyro_z;

		data->mag_x = m4_to_m7_ptr->mag_x;
		data->mag_y = m4_to_m7_ptr->mag_y;
		data->mag_z = m4_to_m7_ptr->mag_z;


		data->led_r = m4_to_m7_ptr->led_r;
		data->led_g = m4_to_m7_ptr->led_g;
		data->led_b = m4_to_m7_ptr->led_b;
		data->led_w = m4_to_m7_ptr->led_w;

	}

void create_json_payload(struct visualization_data_s *data, char * payload)
{
	char temp_buf[64];

	memset(temp_buf, 0, 64);
	/*create json "file"*/
	strcat(payload, "{");


	strcat(payload, "\"temperature\":[");
	sprintf(temp_buf, "%.2f", data->temperature[0]);
	strcat(payload, temp_buf);
	for (int i = 0; i < 60; i++){
		sprintf(temp_buf, ",%.2f", data->temperature[i]);
		strcat(payload, temp_buf);
	}
	strcat(payload, "],");

	strcat(payload, "\"humidity\":[");
	sprintf(temp_buf, "%.2f", data->humidity[0]);
	strcat(payload, temp_buf);

	for (int i = 0; i < 60; i++){
		sprintf(temp_buf, ",%.2f", data->humidity[i]);
		strcat(payload, temp_buf);
	}
	strcat(payload, "],");

	sprintf(temp_buf, "\"potlevel\":%.2f,", data->potlevel);
	strcat(payload, temp_buf);

	sprintf(temp_buf, "\"accel_x\":%.2f,", data->accel_x);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"accel_y\":%.2f,", data->accel_y);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"accel_z\":%.2f,", data->accel_z);
	strcat(payload, temp_buf);

	sprintf(temp_buf, "\"gyro_x\":%.2f,", data->gyro_x);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"gyro_y\":%.2f,", data->gyro_y);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"gyro_z\":%.2f,", data->gyro_z);
	strcat(payload, temp_buf);

	sprintf(temp_buf, "\"mag_x\":%.2f,", data->mag_x);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"mag_y\":%.2f,", data->mag_y);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"mag_z\":%.2f,", data->mag_z);
	strcat(payload, temp_buf);

	sprintf(temp_buf, "\"led_r\":%d,", data->led_r);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"led_g\":%d,", data->led_g);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"led_b\":%d,", data->led_b);
	strcat(payload, temp_buf);
	sprintf(temp_buf, "\"led_w\":%d", data->led_w);
	strcat(payload, temp_buf);

	strcat(payload, "}");
}

void serve_get_data_endpoint(struct netconn *conn){
	/*Create buffer for JSON payload data*/
	char data_resp_buf[2000];
	memset(data_resp_buf, 0, 2000);
	/*Get data from shared struct*/
	fillData(&visualization_data);
	/*HEADER*/
	strcat(data_resp_buf, "HTTP/1.1 200 OK\r\n");
	strcat(data_resp_buf, "Content-Type: application/json\r\n");
	strcat(data_resp_buf, "Access-Control-Allow-Origin: *\r\n");
    strcat(data_resp_buf, "\r\n");
    /*PAYLOAD*/
    create_json_payload(&visualization_data, data_resp_buf);

    /*Write json "file" to conn structure*/
	netconn_write(conn, data_resp_buf, strlen(data_resp_buf), NETCONN_COPY);

}


void serve_post_data_endpoint(struct netconn *conn, char *reqbuf, uint16_t buflen){
	char data_resp_buf[256];
	char temp_buff[64];
	char type[8];

	volatile struct m7_to_m4 * const m7_to_m4_ptr = (struct m7_to_m4 *)(0x30040D00);

	memset(data_resp_buf, 0, 256);
	memset(temp_buff, 0, 64);
	memset(type, 0, 8);

	uint8_t err = 1;
	uint32_t value;

	/*Set values of RGB led and white LED*/
	sscanf(reqbuf, "POST /data/led/%1s?value=%d ", type, (int *)&value);

	switch(type[0]){
		case 'r':
			err = 0;
			led_data.led_r = (uint8_t) (value % 256);
			m7_to_m4_ptr->rgb_r = led_data.led_r;
			break;
		case 'g':
			err = 0;
			led_data.led_g = (uint8_t) (value % 256);
			m7_to_m4_ptr->rgb_g = led_data.led_g;
			break;
		case 'b':
			err = 0;
			led_data.led_b = (uint8_t) (value % 256);
			m7_to_m4_ptr->rgb_b = led_data.led_b;
			break;
		case 'w':
			err = 0;
			led_data.led_w = (uint8_t) (value % 256);
			m7_to_m4_ptr->w_led_status = led_data.led_w;
			break;
		default:
			err = 1;
			break;
	}

	if(err == 0){
	/*If values for LEDs were set right*/
		strcat(data_resp_buf, "HTTP/1.1 200 OK\r\n");
		strcat(data_resp_buf, "Content-Type: text/plain\r\n");
		strcat(data_resp_buf, "Access-Control-Allow-Origin: *\r\n");
		strcat(data_resp_buf, "\r\n");

		sprintf(temp_buff, "{\"led_%c\":%d}", type[0], (int)value);
		strcat(data_resp_buf, temp_buff);

		netconn_write(conn, data_resp_buf , strlen(data_resp_buf), NETCONN_COPY);

	} else {
	/*If values for LEDs were set wrong*/
		serve_404(conn);
	}

}

void serve_404(struct netconn *conn){
	/*Error page*/
	char data_resp_buf[512];
	memset(data_resp_buf, 0, 512);


	strcat(data_resp_buf, "HTTP/1.1 404 Not Found\r\n");
	strcat(data_resp_buf, "Server: LwIP Nucleo H755\r\n");
	strcat(data_resp_buf, "Connection: Closed\r\n");
	strcat(data_resp_buf, "Access-Control-Allow-Origin: *\r\n");
	strcat(data_resp_buf, "Content-Type: text/html; charset=iso-8859-1\r\n\r\n\r\n");
	strcat(data_resp_buf, "<center><big>404 oh noes</h1></center>");
	netconn_write(conn, data_resp_buf, strlen(data_resp_buf), NETCONN_COPY);

}

/**
  * @brief serve tcp connection  
  * @param conn: pointer on connection structure 
  * @retval None
  */
static void http_server_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    err_t recv_err;
    char *buf;
    u16_t buflen;
    struct fs_file file;

    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

    /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
    recv_err = netconn_recv(conn, &inbuf);

    if (recv_err == ERR_OK)
    {
        if (netconn_err(conn) == ERR_OK)
        {
            netbuf_data(inbuf, (void **)&buf, &buflen);

            /*Is this an HTTP GET command? (only check the first 5 chars, since
            there are other formats for GET, and we're keeping it very simple )*/
            if ((buflen >= 5) && (strncmp(buf, "GET /", 5) == 0))
            {

            	/*Check if request to get ST.gif*/
            	if (strncmp((char const *)buf, "GET /head02.png", 14) == 0)
            	{
            		fs_open(&file,"/head02.png");
            		netconn_write(conn, (const unsigned char *)(file.data), (size_t)file.len, NETCONN_NOCOPY);
            		fs_close(&file);
            	}
            	else if((strncmp(buf, "GET /index.html", 15) == 0) || (strncmp(buf, "GET / ", 6) == 0))
            	{
					/*Load STM32H7xx page*/
					fs_open(&file, "/index.html");
					netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
					fs_close(&file);
				 }
            	else if(strncmp((char const *)buf, "GET /data HTTP/1.1", 18) == 0)
            	{
            		/*Handle data endpoint*/
            		serve_get_data_endpoint(conn);
            	}
            	else
            	{
					serve_404(conn);
				}
			}
            else if(strncmp(buf, "POST /data/led/", 15) == 0)
            {
           	  serve_post_data_endpoint(conn, buf, buflen);
            }
             else
             {
       		serve_404(conn);
             }

        }
      }



    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);

    /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void http_server_netconn_thread(void *arg)
{
    struct netconn *conn, *newconn;
    err_t err, accept_err;

    /* Create a new TCP connection handle */
    conn = netconn_new(NETCONN_TCP);

    if (conn != NULL)
    {
        /* Bind to port 80 (HTTP) with default IP address */
        err = netconn_bind(conn, NULL, 80);

        if (err == ERR_OK)
        {
            /* Put the connection into LISTEN state */
            netconn_listen(conn);

            while (1)
            {
                /* accept any icoming connection */
                accept_err = netconn_accept(conn, &newconn);
                if (accept_err == ERR_OK)
                {
                    /* serve connection */
                    http_server_serve(newconn);

                    /* delete connection */
                    netconn_delete(newconn);
                }
            }
        }
    }
}

/**
  * @brief  Initialize the HTTP server (start its thread) 
  * @param  none
  * @retval None
  */
void http_server_netconn_init()
{
	visualization_data = (const struct visualization_data_s){ 0 };
	visualization_data.humidity[59] = 1025.0;
	visualization_data.temperature[59] = 15.0;
    sys_thread_new("HTTP", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
