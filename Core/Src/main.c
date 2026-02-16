/******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  * By: Ding Dao En And Liew Kang Zhen
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/Components/lsm6dsl/lsm6dsl.h"
#include "stdio.h"
#include "string.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include "stm32l4xx_hal_rcc.h"

//For wifi module
#include "main.h"
#include "wifi.h"
#include <stdlib.h>	// for rand(). Can be removed if valid sensor data is sent instead

#define MAX_LENGTH 400	// adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 1000
#define WIFI_WRITE_TIMEOUT 1000
//#define USING_IOT_SERVER  //Please uncomment this to enable IOT on Thingsboard feature, but due to the Thingsboard delays as mentioned in Canvas, we have commented this out for more stable

const char* WiFi_SSID = "KZ Laptop";				// Replacce mySSID with WiFi SSID for your router / Hotspot
const char* WiFi_password = "43z}N904";	// Replace myPassword with WiFi password for your router / Hotspot
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;	// WiFi security your router / Hotspot. No need to change it unless you use something other than WPA2 PSK
const uint16_t SOURCE_PORT = 1234;	// source port, which can be almost any 16 bit number

const char* SERVER_NAME = "demo.thingsboard.io";
const uint16_t IOT_DEST_PORT = 80;
const uint16_t PACKET_DEST_PORT = 2028;
uint8_t ipaddr[4] = {192, 168, 137, 1}; // IP address of your laptop wireless lan adapter, which is the one you successfully used to test Packet Sender above.
									// If using IoT platform, this will be overwritten by DNS lookup, so the values of x and y doesn't matter
											//(it should still be filled in with numbers 0-255 to avoid compilation errors)
SPI_HandleTypeDef hspi3;

static void UART1_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void SystemClock_Config(void);
static void Buzzer_Init(void);
static void Buzz_Start();
static void Buzz_Stop();
static void GYRO_GPIO_INT_Init(void);
void SENSOR_IO_Init(void);
static void Gyro_Tilt(void);
static void Gyro_Tilt_Disable(void);
static void Gyro_Tap(void);
static void Gyro_Tap_Disable(void);


UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
int gif = 0;
int flag = 0;

volatile int switch_game = 0;
volatile int press_counter = 0;
volatile uint32_t start_t0;
int mag_th = 0;
uint32_t start_cap;
volatile int cont_f = 0;
volatile int captured;
int player_nearby_flag;
volatile int flag_mode1;
int iot_connect_flag = 1;
uint32_t buzz_timer;
int buzz_flag;
uint32_t TB_timer;

uint8_t req[MAX_LENGTH];					// request packet
uint8_t resp[MAX_LENGTH];					// response packet
uint16_t Datalen;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_1: 								// Wifi Interrupt
			SPI_WIFI_ISR();
			break;

		case LSM6DSL_INT1_EXTI11_Pin:					// Gyro Tilt and Tap Interrupt
			flag_mode1 = 1;
			break;

		case BUTTON_EXTI13_Pin:							// Button Interrupt
			if (switch_game == 1) {
				if ((mag_th == 1) && ((HAL_GetTick() - start_cap) <= 3000))
				{
					captured = 1;
					cont_f = 0;
					return;
				} else {
					cont_f = 0;
					captured = 0;
				}
			}

			if(press_counter == 0)
			{
				press_counter = 1;
				start_t0 = HAL_GetTick();
				return;
			}

			if (press_counter == 1)
			{
				if(HAL_GetTick() - start_t0 <= 1000)
				{
					if (switch_game == 0)
					{
						switch_game = 1;
					} else if (switch_game == 1)
					{
						switch_game = 0;
					} else
					{
						switch_game = 1;
					}
					press_counter = 0;
					return;
				}

				start_t0 = HAL_GetTick();

			}
			return;
	}
}

int main(void)
{
//	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 	HAL_Init();
	SystemClock_Config();
	UART1_Init();
	MX_GPIO_Init();
	MX_I2C1_Init();

	char message_print[MAX_LENGTH];

	int light_colour = 0; //0 for Green light, 1 for Red light
	int player_count = 0;
	int bribe_count = 0;
	int change_mod = -1;

//	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_GYRO_Init();
	BSP_ACCELERO_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_MAGNETO_Init();
	ssd1306_Init();
	Buzzer_Init();
	SENSOR_IO_Init();
	GYRO_GPIO_INT_Init();

	float gyro_data[3];
	int16_t gyro_data_i16[3] = {0};
	float acc_data[3];
	int16_t acc_data_i16[3] = {0};
	float hdata;
	float pdata;
	float tdata;
	float mag_data[3];
	int16_t mag_data_i16[3] = {0};
	float past_data[2] = {0};
	float prev_tdata = 0;
	float prev_pdata = 0;
	float prev_hdata = 0;

	uint32_t start_t1; // initialize start time for clock
	uint32_t start_t2;
	uint32_t start_t3;
	uint32_t start_t4;
	start_t1 = HAL_GetTick();
	start_t2 = HAL_GetTick();
	start_t3 = HAL_GetTick();
	start_t4 = HAL_GetTick();


	char json[1000];

	WIFI_Status_t WiFi_Stat; 																						// WiFi status. Should remain WIFI_STATUS_OK if everything goes well
	WiFi_Stat = WIFI_Init();																						// if it gets stuck here, you likely did not include EXTI1_IRQHandler() in stm32l4xx_it.c as mentioned above
	WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security); 											// joining a WiFi network takes several seconds. Don't be too quick to judge that your program has 'hung' :)
	if(WiFi_Stat!=WIFI_STATUS_OK) while(1); 																		// halt computations if a WiFi connection could not be established.


	WiFi_Stat = WIFI_OpenClientConnection(2, WIFI_TCP_PROTOCOL, "conn", ipaddr, PACKET_DEST_PORT, SOURCE_PORT);
	if(WiFi_Stat!=WIFI_STATUS_OK) while(1);

#ifdef USING_IOT_SERVER
	WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr);															// DNS lookup to find the ip address, if using a connection to an IoT server
	WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, IOT_DEST_PORT, SOURCE_PORT);
	if(WiFi_Stat!=WIFI_STATUS_OK) while(1);
#endif

	while (1)
	{
		if (switch_game == 0)
		{
			Gyro_Tap_Disable();
			Gyro_Tilt();

			change_mod = -1;
			flag_mode1 = 0;

			sprintf(message_print, "Entering Red Light, Green Light as Enforcer\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			sprintf((char*)req, "Enforcer Please Choose Your Detection Mode\r\n1: Default Mode\r\n2: Tilt Mode\r\n");
			WiFi_Stat = WIFI_SendData(2, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

			while(1)
			{
				WiFi_Stat = WIFI_ReceiveData(2, resp, MAX_LENGTH, &Datalen, 100);

				if (WiFi_Stat == WIFI_STATUS_OK && Datalen > 0)
				{
					// Make sure the received data is null-terminated
					if (Datalen < MAX_LENGTH) {
						resp[Datalen] = '\0';  // safe null-termination
					} else {
						resp[MAX_LENGTH - 1] = '\0'; // prevent overflow
					}

					if (!strcmp((char *)resp, "1\r")) {
						change_mod = 0;
						break;
					} else if (!strcmp((char *)resp, "2\r")) {
						change_mod = 1;
						break;
					}
				}
			}


			draw_doll_back();
			sprintf(message_print, "Green Light\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			hdata = BSP_HSENSOR_ReadHumidity();

			sprintf(message_print, "Humidity: %f\r\n", hdata);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			tdata = BSP_TSENSOR_ReadTemp();
			sprintf(message_print, "Temperature: %f\r\n", tdata);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			pdata = BSP_PSENSOR_ReadPressure();
			sprintf(message_print, "Pressure: %f\r\n\n", pdata);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			start_t1 = HAL_GetTick();
			start_t2 = HAL_GetTick();
			light_colour = 0;

			while (switch_game == 0)
			{
				if (light_colour == 0) // for green light
				{
					//Code for green light
					BSP_LED_On(LED2);
#ifdef USING_IOT_SERVER
					if (iot_connect_flag == 0)
					{
						WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, IOT_DEST_PORT, SOURCE_PORT);
						iot_connect_flag = 1;
					}
#endif

					if (HAL_GetTick() - start_t1 >= (10000))
					{
						light_colour = !light_colour;
						sprintf(message_print, "Red Light\r\n\n");
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						past_data[0] = 0;
						past_data[1] = 0;

						BSP_GYRO_GetXYZ(gyro_data_i16);
						gyro_data[0] = (float)gyro_data_i16[0] / (1000.0f);
						gyro_data[1] = ((float)gyro_data_i16[1] / (1000.0f)) - (17.0f);
						gyro_data[2] = ((float)gyro_data_i16[2] / (1000.0f)) + (16.0f);

						BSP_ACCELERO_AccGetXYZ(acc_data_i16);
						acc_data[0] = (float)acc_data_i16[0] * (9.8/1000.0f);
						acc_data[1] = (float)acc_data_i16[1] * (9.8/1000.0f);
						acc_data[2] = (float)acc_data_i16[2] * (9.8/1000.0f);

						sprintf(message_print, "Gyro X: %f\r\n", gyro_data[0]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Gyro Y: %f\r\n", gyro_data[1]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Gyro Z: %f\r\n\n", gyro_data[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Acc X: %f\r\n", acc_data[0]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Acc Y: %f\r\n", acc_data[1]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Acc Z: %f\r\n\n", acc_data[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						start_t1 = HAL_GetTick();
						start_t2 = HAL_GetTick();
						start_t3 = HAL_GetTick();
					}

					if (HAL_GetTick() - start_t2 >= (2000))
					{
						hdata = BSP_HSENSOR_ReadHumidity();
						sprintf(message_print, "Humidity: %f\r\n", hdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						tdata = BSP_TSENSOR_ReadTemp();
						sprintf(message_print, "Temperature: %f\r\n", tdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						pdata = BSP_PSENSOR_ReadPressure();
						sprintf(message_print, "Pressure: %f\r\n\n", pdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

#ifdef USING_IOT_SERVER
						sprintf(json,
							"{\"Temp\":%f, \"Humidity\":%f, \"Pressure\":%f}",
							tdata, hdata, pdata);
						sprintf((char*)req, "POST /api/v1/DEnd_1/telemetry HTTP/1.1\r\nHost: demo.thingsboard.io\r\nContent-Length: %d\r\n\r\n%s",strlen(json),json);
						WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
#endif
						start_t2 = HAL_GetTick();
					}
				} else
				{
					// code for red light
					draw_doll_face();

					if (HAL_GetTick() - start_t1 >= (10000))
					{
						draw_doll_back();
						light_colour = !light_colour;
						sprintf(message_print, "Green Light\r\n\n");
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						hdata = BSP_HSENSOR_ReadHumidity();
						sprintf(message_print, "Humidity: %f\r\n", hdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						tdata = BSP_TSENSOR_ReadTemp();
						sprintf(message_print, "Temperature: %f\r\n", tdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						pdata = BSP_PSENSOR_ReadPressure();
						sprintf(message_print, "Pressure: %f\r\n\n", pdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

						start_t1 = HAL_GetTick();
						start_t2 = HAL_GetTick();
#ifdef USING_IOT_SERVER
						WIFI_CloseClientConnection(1);
						iot_connect_flag = 0;
#endif

					}

					if (HAL_GetTick() - start_t2 >= (500))
					{
						BSP_LED_Toggle(LED2);

						BSP_GYRO_GetXYZ(gyro_data_i16);
						gyro_data[0] = (float)gyro_data_i16[0] / (1000.0f);
						gyro_data[1] = ((float)gyro_data_i16[1] / (1000.0f)) - (17.0f);
						gyro_data[2] = ((float)gyro_data_i16[2] / (1000.0f)) + (16.0f);

						BSP_ACCELERO_AccGetXYZ(acc_data_i16);
						acc_data[0] = (float)acc_data_i16[0] * (9.8/1000.0f);
						acc_data[1] = (float)acc_data_i16[1] * (9.8/1000.0f);
						acc_data[2] = (float)acc_data_i16[2] * (9.8/1000.0f);

						if (change_mod == 0)
						{
							if (gyro_data[0] <= -35 || gyro_data[0] >= 35 || gyro_data[1] <= -40 || gyro_data[1] >= 40
									|| gyro_data[2] <= -35 || gyro_data[2] >= 35 ||
									acc_data[0] <= past_data[0] - 1 || acc_data[0] >= past_data[0] + 1 ||
									acc_data[1] <= past_data[1] - 1 || acc_data[1] >= past_data[1] + 1)
							{
								past_data[0] = acc_data[0];
								past_data[1] = acc_data[1];
								sprintf(message_print, "Player Out!\r\n\n");
								HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
							}
						} else if (change_mod == 1)
						{
							if (flag_mode1 == 1)
							{
								sprintf(message_print, "Player Out!\r\n\n");
								HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

								flag_mode1 = 0;
							}
						}

						start_t2 = HAL_GetTick();
					}

					if (HAL_GetTick() - start_t3 >= (2000))
					{
						BSP_GYRO_GetXYZ(gyro_data_i16);
						gyro_data[0] = (float)gyro_data_i16[0] / (1000.0f);
						gyro_data[1] = ((float)gyro_data_i16[1] / (1000.0f)) - (17.0f);
						gyro_data[2] = ((float)gyro_data_i16[2] / (1000.0f)) + (16.0f);

						BSP_ACCELERO_AccGetXYZ(acc_data_i16);
						acc_data[0] = (float)acc_data_i16[0] * (9.8/1000.0f);
						acc_data[1] = (float)acc_data_i16[1] * (9.8/1000.0f);
						acc_data[2] = (float)acc_data_i16[2] * (9.8/1000.0f);

						sprintf(message_print, "Gyro X: %f\r\n", gyro_data[0]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Gyro Y: %f\r\n", gyro_data[1]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Gyro Z: %f\r\n\n", gyro_data[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Acc X: %f\r\n", acc_data[0]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Acc Y: %f\r\n", acc_data[1]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						sprintf(message_print, "Acc Z: %f\r\n\n", acc_data[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
#ifdef USING_IOT_SERVER
						sprintf(json,
							"{\"Gyro X\":%f, \"Gyro Y\":%f, \"Gyro Z\":%f, "
							"\"Acc X\":%f, \"Acc Y\":%f, \"Acc Z\":%f}",
							gyro_data[0], gyro_data[1], gyro_data[2],
							acc_data[0], acc_data[1], acc_data[2]);
						sprintf((char*)req, "POST /api/v1/DEnd_1/telemetry HTTP/1.1\r\nHost: demo.thingsboard.io\r\nContent-Length: %d\r\n\r\n%s",strlen(json),json);
						WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
#endif
						start_t3 = HAL_GetTick();
					}
				}
			}
		}

		if (switch_game == 1)
		{
			Gyro_Tilt_Disable();
			Gyro_Tap();
			prev_hdata = BSP_HSENSOR_ReadHumidity();
			prev_tdata = BSP_TSENSOR_ReadTemp();
			prev_pdata = BSP_PSENSOR_ReadPressure();

#ifdef USING_IOT_SERVER
			if (iot_connect_flag == 0)
			{
				WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, IOT_DEST_PORT, SOURCE_PORT);
				iot_connect_flag = 1;
			}
#endif

			sprintf(message_print, "Entering Catch And Run as Enforcer\r\n\n");
			bribe_count = 0;
			player_count = 0;
			draw_catch_count(player_count);
			draw_catch_count(0);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			start_t1 = HAL_GetTick();
			start_t2 = HAL_GetTick();
			start_t3 = HAL_GetTick();
			start_t4 = HAL_GetTick();
			TB_timer = HAL_GetTick();
			start_cap = HAL_GetTick();
			flag_mode1 = 0;

			while (switch_game == 1)
			{

#ifdef USING_IOT_SERVER
				if (HAL_GetTick() - start_t4 >= (10000))
				{
					WIFI_CloseClientConnection(1);
					iot_connect_flag = 0;
					WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, IOT_DEST_PORT, SOURCE_PORT);
					iot_connect_flag = 1;
					start_t4 = HAL_GetTick();
				}
#endif

				if (buzz_flag == 1 && HAL_GetTick() - buzz_timer >= 100)
				{
					Buzz_Stop();

				}

				if (HAL_GetTick() - start_t1 >= 1000)
				{
					hdata = BSP_HSENSOR_ReadHumidity();
					tdata = BSP_TSENSOR_ReadTemp();
					pdata = BSP_PSENSOR_ReadPressure();

					if (tdata >= prev_tdata + 0.2)
					{
						sprintf(message_print, "Temperature spike detected! T:%fC. Dangerous environment!\r\n\n", tdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					}

					if (pdata >= prev_pdata + 0.1)
					{
						sprintf(message_print, "Pressure spike detected! P:%fPa. Dangerous environment!\r\n\n", pdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					}

					if (hdata >= prev_hdata + 1)
					{
						sprintf(message_print, "Humidity spike detected! H:%fg/cm3. Dangerous environment!\r\n\n", hdata);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					}

					prev_tdata = tdata;
					prev_pdata = pdata;
					prev_hdata = hdata;

					start_t1 = HAL_GetTick();
				}

				if (HAL_GetTick() - start_t2 >= 1000)
				{
					BSP_MAGNETO_GetXYZ(mag_data_i16);
					mag_data[0] = (float)mag_data_i16[0];
					mag_data[1] = (float)mag_data_i16[1];
					mag_data[2] = (float)mag_data_i16[2];
					sprintf(message_print, "Magneto X: %f\r\n", mag_data[0]);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					sprintf(message_print, "Magneto Y: %f\r\n", mag_data[1]);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					sprintf(message_print, "Magneto Z: %f\r\n\n", mag_data[2]);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

					start_t2 = HAL_GetTick();
				}

#ifdef USING_IOT_SERVER
				if (HAL_GetTick() - TB_timer >= (2000))
				{
					sprintf(json,
						"{\"Mag X\":%f, \"Mag Y\":%f, \"Mag Z\":%f}",
						mag_data[0], mag_data[1], mag_data[2]);
					sprintf((char*)req, "POST /api/v1/DEnd_1/telemetry HTTP/1.1\r\nHost: demo.thingsboard.io\r\nContent-Length: %d\r\n\r\n%s",strlen(json),json);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

					sprintf(json,
						"{\"Temp2\":%f, \"Humidity2\":%f, \"Pressure2\":%f}",
						tdata, hdata, pdata);
					sprintf((char*)req, "POST /api/v1/DEnd_1/telemetry HTTP/1.1\r\nHost: demo.thingsboard.io\r\nContent-Length: %d\r\n\r\n%s",strlen(json),json);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

					TB_timer = HAL_GetTick();
				}
#endif

				if (captured == 1) {
					sprintf(message_print, "Player captured, good job!\r\n\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					Buzz_Start();
					mag_th = 0;
					cont_f = 0;
					captured = 0;
					start_cap = HAL_GetTick();
					player_count ++;
					draw_catch_count(player_count);
					if (player_count == 5)
					{
						switch_game = 2;
						sprintf(message_print, "PENTAKILL!!\r\n\nDouble Press to restart this game.\r\n\n");
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					}
				}

				if ((mag_th == 1) && ((HAL_GetTick() - start_cap) >= 3000))
				{
					sprintf(message_print, "Player escaped! Keep trying.\r\n\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					mag_th = 0;
					cont_f = 0;
					captured = 0;
					player_nearby_flag = 0;
					start_cap = HAL_GetTick();
				}

				if (flag_mode1 == 1)
				{
					flag_mode1 = 0;

					if (mag_th == 0 && bribe_count < 5)
					{
						if (player_count > 0)
						{
							player_count --;
							draw_catch_count(player_count);
							bribe_count ++;
							sprintf((char*)req, "Shhh... Bribed %d player!\r\n\n", bribe_count);
							WiFi_Stat = WIFI_SendData(2, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
						}

						if (bribe_count == 5)
						{
							sprintf((char*)req, "Bribe quota reached\r\n\n");
							WiFi_Stat = WIFI_SendData(2, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
						}
					}
				}

				if (mag_data[1] <= -2500)
				{
					if (player_nearby_flag == 0)
					{
						sprintf(message_print, "Player is nearby! Move faster.\r\n\n");
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						player_nearby_flag = 1;
					}

					mag_th = 1;

					if (cont_f == 0)
					{
						cont_f = 1;
						start_cap = HAL_GetTick();
					}

					if ((HAL_GetTick() - start_t3) >= 50)
					{
						BSP_LED_Toggle(LED2);
						start_t3 = HAL_GetTick();
					}

				} else if ((mag_data[1] <= -2000) && (mag_data[1] > -2500))
				{
					mag_th = 0;
					cont_f = 0;
					player_nearby_flag = 0;
					if ((HAL_GetTick() - start_t3) >= 250)
					{
						BSP_LED_Toggle(LED2);
						start_t3 = HAL_GetTick();
					}

				} else if ((mag_data[1] <= -1500) && (mag_data[1] > -2000))
				{
					mag_th = 0;
					cont_f = 0;
					player_nearby_flag = 0;
					if ((HAL_GetTick() - start_t3) >= 1000)
					{
						BSP_LED_Toggle(LED2);
						start_t3 = HAL_GetTick();
					}

				} else {
					mag_th = 0;
					cont_f = 0;
					player_nearby_flag = 0;
				}
			}
		}

		if (switch_game == 2)
		{

		}
	}
}

/*User Press Button intialization function*/
static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();  // Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*Gyroscope GPIO interrupt intialization function*/
static void GYRO_GPIO_INT_Init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();  // Enable AHB2 Bus for GPIOD

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Configuration of LSM6DSL_INT1_EXTI11_Pin (GPIO-D Pin-11) as AF,
	GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	// Enable NVIC EXTI line 11
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*Gyroscope Wrist Tilt internal registers enable function*/
static void Gyro_Tilt(void)
{
	uint8_t ctrl = 0x00;
	uint8_t tmp;

	/* Read CTRL1_XL */
	tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);

	/* Write value to ACC MEMS CTRL1_XL register: FS and Data Rate */
	ctrl = 0x30; //0b00110000 --> 52Hz, 2g
	tmp &= ~(0xFC); //tmp ---> 0b000000xx
	tmp |= ctrl; //tmp ---> 0b00110000xx
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, tmp);

	/*Write value to ACC MEMS CTRL10_C register: Enable Tilt and Enable Embedded Functionalities*/
	tmp = 0x0C;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, tmp);

	/*Write value to ACC MEMS MD1_CFG register: Tilt detector interrupt driven to INT1 pin*/
	tmp = 0x02;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, tmp);

	/* Read CTRL3_C */
	tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

	/* Write value to ACC MEMS CTRL3_C register: BDU and Auto-increment */
	ctrl = 0x44; // ctrl ---> 0b01000100
	tmp &= ~(0x44); // tmp ---> 0bx1xxx1xx
	tmp |= ctrl; // tmp ---> 0bx1xxx1xx
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);
}

static void Gyro_Tilt_Disable(void)
{
	uint8_t ctrl = 0x00;
	uint8_t tmp;

	/*Write value to ACC MEMS CTRL10_C register: Disable Tilt and Enable Embedded Functionalities*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, tmp);

	/*Write value to ACC MEMS MD1_CFG register: Disable Tilt detector interrupt driven to INT1 pin*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, tmp);
}

static void Gyro_Tap(void)
{
	uint8_t ctrl = 0x00;
	uint8_t tmp;

	/* Read CTRL1_XL */
	tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);

	/* Write value to ACC MEMS CTRL1_XL register: FS and Data Rate */
	ctrl = 0x60; //0b01100000 --> 416Hz, 2g
	tmp &= ~(0xFC); //tmp ---> 0b000000xx
	tmp |= ctrl; //tmp ---> 0b011000xx
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, tmp);

	/*Write value to ACC MEMS TAP_CFG register: Enable interrupts and tap detection on X,Y,Z axis*/
	tmp = 0x8E;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, tmp);

	/*Write value to ACC MEMS TAP_THS_6D register: Set tap threshold*/
	tmp = 0x89;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_THS_6D, tmp);

	/*Write value to ACC MEMS INT_DUR2 register: Set quiet and shock time windows*/
	tmp = 0x06;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT_DUR2, tmp);

	/*Write value to ACC MEMS WAKE_UP_THS register: Only single-tap enabled (SINGLE_DOUBLE_TAP = 0)*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_THS, tmp);

	/* Write value to ACC MEMS MD1_CFG register: Single-tap interrupt driven into INT1 pin*/
	tmp = 0x40;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, tmp);
}

static void Gyro_Tap_Disable(void)
{
	uint8_t ctrl = 0x00;
	uint8_t tmp;

	/*Write value to ACC MEMS TAP_CFG register: Disable interrupts and tap detection on X,Y,Z axis*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, tmp);

	/*Write value to ACC MEMS TAP_THS_6D register: Clear tap threshold*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_THS_6D, tmp);

	/*Write value to ACC MEMS INT_DUR2 register: Clear quiet and shock time windows*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT_DUR2, tmp);

	/*Write value to ACC MEMS WAKE_UP_THS register: Only single-tap disabled (SINGLE_DOUBLE_TAP = 0)*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_THS, tmp);

	/* Write value to ACC MEMS MD1_CFG register: Disable Single-tap interrupt driven into INT1 pin*/
	tmp = 0x00;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, tmp);
}

/*Function to initialize GPIOA Pin 4 for Buzeer*/
static void Buzzer_Init(void)
{

	__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable AHB2 Bus for GPIOA

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//Configuration of Buzzer (GPIO-A Pin-4) for Arduino D7
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Output Push Pull Mode
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Write output low to initialize buzzer.
}

/*Function to let buzzer buzz for 100ms*/
static void Buzz_Start()
{
	buzz_flag = 1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	buzz_timer = HAL_GetTick();
}

static void Buzz_Stop()
{
	buzz_flag = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/*UART initialization function*/
static void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */ /* similar to configuring I2C registers after setting a GPIO to I2C */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	while(1);
	}
}
//
static void MX_I2C1_MspInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;


	__HAL_RCC_GPIOB_CLK_ENABLE();

	/**I2C1 GPIO Configuration
	PB8  ------> I2C1_SCL
	PB9  ------> I2C1_SDA
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*** Configure the I2C peripheral ***/
	/* Enable I2C clock */
	__HAL_RCC_I2C1_CLK_ENABLE();

	/* Force the I2C peripheral clock reset */
	__HAL_RCC_I2C1_FORCE_RESET();

	/* Release the I2C peripheral clock reset */
	__HAL_RCC_I2C1_RELEASE_RESET();

	/* Enable and set I2Cx Interrupt to a lower priority */
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

	/* Enable and set I2Cx Interrupt to a lower priority */
	HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00C0216C;   // For 400kHz if PCLK1 = 80MHz
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    MX_I2C1_MspInit();
    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure main internal regulator output voltage **/
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators: use HSI16 with PLL **/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;      // divide HSI16 by 1
    RCC_OscInitStruct.PLL.PLLN = 10;     // multiply by 10 → 16 MHz × 10 = 160 MHz
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // divide by 2 → 80 MHz SYSCLK
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB, and APB busses clocks **/
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // use PLL as system clock
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 80 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;   // 80 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // 80 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    while (1)
    {
    }
}

void SPI3_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&hspi3);
}




