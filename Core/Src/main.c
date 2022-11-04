/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2s.h"
#include "iwdg.h"
#include "pdm2pcm.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "arm_math.h"
#include <string.h>
#include "FLASH_SECTOR_F4.h"
#include "jsmn.h"
#include "tusb.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const char* upload_image[2]=
{
  "Hello world from TinyUSB DFU! - Partition 0",
  "Hello world from TinyUSB DFU! - Partition 1"
};

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 5000,
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 	Input buffer (data) is a uint8 variable with a length equal to (Output frequency / 1000 *
	decimation factor * Input Microphone Channels / 16 < - (data format) ) at least.
	Output buffer (dataOut) is a uint16 variable with a length equal to (Output frequency /
	1000 * Output Microphone Channels) at least.
 */
#define FFT_SAMPLES 1024
#define PCM_SAMPLES FFT_SAMPLES
#define PDM_SAMPLES (PCM_SAMPLES*4)
#define REAL_VALUE_FREQ_SAMPLING 44014

#define FREQS_SAMPLES FFT_SAMPLES/2

#define MAX_AMPLITUDE 60
#define MAX_PWM 100

#define PWM_GAIN 40

#define SIZE_RX_BUF_USB 512


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t pdmRxBuf[PDM_SAMPLES];
uint16_t PCMOutBuffer[PCM_SAMPLES];
uint8_t fftcomplete;
float32_t FFTInBuffer[FFT_SAMPLES];
float32_t FFTOutBuffer[FFT_SAMPLES];
volatile bool samplesReady;
uint8_t OutFreqArray[10];
arm_rfft_fast_instance_f32 FFTHandler;

char RxBufferFromUSB[SIZE_RX_BUF_USB];
uint16_t Head = 0;
char WriteArrayFlash[SIZE_RX_BUF_USB];
char ReadArrayFlash[SIZE_RX_BUF_USB];
char string[SIZE_RX_BUF_USB];
char rewriteFLASH[SIZE_RX_BUF_USB];
uint16_t rewritecnt = 0;
char rewriteRxUSB[SIZE_RX_BUF_USB];
uint16_t rewritecnt_1 = 0;


int Freqs[FREQS_SAMPLES];
int Noises_floor_offset = 40; //35

// Pomoce naukowe:
// https://github.com/zserge/jsmn
// Uzycie strtol() i tego do stringow : https://git.man.poznan.pl/stash/projects/PRACELAB/repos/pam/browse/common/token.c

int i;
int r;
jsmn_parser p;
jsmntok_t t[128]; /* We expect no more than 128 tokens */

int Freq_Range[12];
float32_t Amplitude_Section[6];
int Freq_Section[6];
int16_t Gain[6];

uint8_t MotorPWM[8];

// TIMERS
uint32_t ConfigTimer;
uint32_t PWMTimer;
uint32_t ButtonTimer;
uint32_t TimerB;

// FLAGS
uint8_t RxFlagUSB = 0;

uint32_t CheckDataFlash = 0;

uint8_t FlashFlag = 0;

uint32_t Timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
float32_t GetMaxAmplitude(uint16_t lowFreq, uint16_t highFreq);
void InitParams();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void cdc_task(void)
{
     // connected and there are data available
     if ( tud_cdc_available() )
     {
       // read datas
       char buf[64];
       uint32_t count = tud_cdc_read(buf, sizeof(buf));
       (void) count;

 	  memcpy(&RxBufferFromUSB[Head], buf, count);
 	  Head += 64;

 	  if(strstr(RxBufferFromUSB, "connection=0") != NULL){

 		  RxFlagUSB = 1;
 	  }

 	  if(strstr(RxBufferFromUSB, "}") != NULL){

 		  RxFlagUSB = 1;
 	  }

       //tud_cdc_write(buf, count);
       //tud_cdc_write_flush();
     }
}

float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /*
   *  ZEBY DFU DZIALALO NALEZY
   *  PIN: BOOT0 PODLACZYC DO VDD
   *  PIN: BOOT1 PODLACZYC DO GND (PIN BOOT1 sprawdzamy w dokumentacji danego stm32 ktory to pin, w naszym przypadku jest to PIN: PB2)
   *  PO PODLACZENIU ZRESETOWAC STM32
   */


  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
	 // SendUart(&huart2, "!!!!!IWDG RESET!!!!!\n\r");
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  __HAL_RCC_CLEAR_RESET_FLAGS();
  }


  HAL_I2S_Receive_DMA(&hi2s2, &pdmRxBuf[0],PDM_SAMPLES);
  arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

  tusb_init();
  tud_init(BOARD_TUD_RHPORT);

  InitParams();

  jsmn_init(&p);

  Flash_Read_Data(0x08060000, &CheckDataFlash, 1);
  if(CheckDataFlash != 0xFFFFFFFF){
	  Flash_Read_Data(0x08060000 , (uint32_t *)ReadArrayFlash, 128);
	  Convert_To_Str((uint32_t *)ReadArrayFlash, string);
	  FlashFlag = 1;

  }

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
/*
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 50);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 50);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 50);
*/
  PWMTimer = HAL_GetTick();
  ConfigTimer = HAL_GetTick();
  ButtonTimer = HAL_GetTick();
  TimerB = HAL_GetTick();

  Timer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	  {
		  // PRESCALER NA 32 TO PO 4 SEK BEZ ODSWIEZANIA STM32 NASTAPI RESET!
		  /* Refresh Error */
		  Error_Handler();
	  }



	  if((HAL_GetTick() - Timer) > 500){
		  Timer = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }


	  tud_task();
	  cdc_task();
	  led_blinking_task();

	  if((HAL_GetTick() - ButtonTimer) > 100){
		  ButtonTimer = HAL_GetTick();

	  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET){

		  HAL_GPIO_WritePin(CONV_STD_GPIO_Port, CONV_STD_Pin, GPIO_PIN_SET);
		  if(HAL_GetTick() - TimerB > 4000){
			  TimerB = HAL_GetTick();

		  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET){
			  HAL_GPIO_WritePin(CONV_STD_GPIO_Port, CONV_STD_Pin, GPIO_PIN_RESET);

			  while(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET);
		  }
		  }
	  }

	  }

	  if (RxFlagUSB == 1) {
	  			RxFlagUSB = 0;
	  			if (strstr(RxBufferFromUSB, "connection=0") != NULL) {
	  				char connected[] = "connection=1";
	  				tud_cdc_write(connected, sizeof(connected));
	  				tud_cdc_write_flush();
	  				Head = 0;
	  				memset(&RxBufferFromUSB[0], 0, sizeof(RxBufferFromUSB));
	  			} else {

	  				jsmn_init(&p);
	  				char *ptr;

	  				r = jsmn_parse(&p, RxBufferFromUSB, strlen(RxBufferFromUSB), t,
	  						sizeof(t) / sizeof(t[0]));
	  				if (r < 0) {
	  					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  					printf("Failed to parse JSON: %d\n", r);
	  					return 1;
	  				}
	  				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  				/* Assume the top-level element is an object */
	  				if (r < 1 || t[0].type != JSMN_OBJECT) {
	  					printf("Object expected\n");
	  					return 1;
	  				}

	  				/* Loop over all keys of the root object */
	  				for (i = 1; i < r; i++) {
	  					if (jsoneq(RxBufferFromUSB, &t[i], "F1_min") == 0) {
	  						Freq_Range[0] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F2_min") == 0) {
	  						Freq_Range[2] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F3_min") == 0) {
	  						Freq_Range[4] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F4_min") == 0) {

	  						Freq_Range[6] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F5_min") == 0) {

	  						Freq_Range[8] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F6_min") == 0) {

	  						Freq_Range[10] = strtol(
	  								RxBufferFromUSB + t[i + 1].start, &ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F1_max") == 0) {

	  						Freq_Range[1] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F2_max") == 0) {

	  						Freq_Range[3] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F3_max") == 0) {

	  						Freq_Range[5] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F4_max") == 0) {

	  						Freq_Range[7] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F5_max") == 0) {

	  						Freq_Range[9] = strtol(RxBufferFromUSB + t[i + 1].start,
	  								&ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "F6_max") == 0) {

	  						Freq_Range[11] = strtol(
	  								RxBufferFromUSB + t[i + 1].start, &ptr, 0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "G1") == 0) {

	  						Gain[0] = strtol(RxBufferFromUSB + t[i + 1].start, &ptr,
	  								0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "G2") == 0) {

	  						Gain[1] = strtol(RxBufferFromUSB + t[i + 1].start, &ptr,
	  								0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "G3") == 0) {

	  						Gain[2] = strtol(RxBufferFromUSB + t[i + 1].start, &ptr,
	  								0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "G4") == 0) {

	  						Gain[3] = strtol(RxBufferFromUSB + t[i + 1].start, &ptr,
	  								0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "G5") == 0) {

	  						Gain[4] = strtol(RxBufferFromUSB + t[i + 1].start, &ptr,
	  								0);

	  						i++;
	  					} else if (jsoneq(RxBufferFromUSB, &t[i], "G6") == 0) {

	  						Gain[5] = strtol(RxBufferFromUSB + t[i + 1].start, &ptr,
	  								0);

	  						i++;
	  					}

	  					else {
	  						//printf("Unexpected key: %.*s\n", t[i].end - t[i].start,
	  						// RxBufferFromUSB + t[i].start);
	  					}
	  				}

	  				Head = 0;
	  				// \/ Wysyla co 64 bajty takze zeby wyslac wiecej musisz zrobic petle
	  				//tud_cdc_write(RxBufferFromUSB, sizeof(RxBufferFromUSB));
	  				//tud_cdc_write_flush();
	  				int numofwords = (strlen(RxBufferFromUSB) / 4)
	  						+ ((strlen(RxBufferFromUSB) % 4) != 0);
	  				Flash_Write_Data(0x08060000, (uint32_t*) RxBufferFromUSB,
	  						numofwords);
	  				memset(&RxBufferFromUSB[0], 0, sizeof(RxBufferFromUSB));

	  			}

	  		}

	  		if (FlashFlag == 1) {


	  			for (char *p = strchr(string, '}'); p != NULL;
	  					p = strchr(p + 1, '}')) {
	  				rewritecnt = p - string;
	  			}

	  			memcpy(rewriteFLASH, string, (rewritecnt + 1));

	  			jsmn_init(&p);
	  			char *ptr;

	  			r = jsmn_parse(&p, rewriteFLASH, strlen(rewriteFLASH), t, //Ta funkcja wyrzuca HardFault
	  					sizeof(t) / sizeof(t[0]));// Wynika to z przepisywania danych z FLASH poniewaz wiecej przepisuje co daje to ze
	  			// Przepisuje pozniej 0xFF a jsmn parser tego nie widzi jako terminatora \0, Solution: Usun 0xFF aby zostaly same 0
	  			if (r < 0) {
	  				printf("Failed to parse JSON: %d\n", r);
	  				return 1;
	  			}

	  			/* Assume the top-level element is an object */
	  			if (r < 1 || t[0].type != JSMN_OBJECT) {
	  				printf("Object expected\n");
	  				return 1;
	  			}

	  			/* Loop over all keys of the root object */
	  			for (i = 1; i < r; i++) {
	  				if (jsoneq(rewriteFLASH, &t[i], "F1_min") == 0) {
	  					Freq_Range[0] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F2_min") == 0) {
	  					Freq_Range[2] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F3_min") == 0) {
	  					Freq_Range[4] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F4_min") == 0) {

	  					Freq_Range[6] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F5_min") == 0) {

	  					Freq_Range[8] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F6_min") == 0) {

	  					Freq_Range[10] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F1_max") == 0) {

	  					Freq_Range[1] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F2_max") == 0) {

	  					Freq_Range[3] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F3_max") == 0) {

	  					Freq_Range[5] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F4_max") == 0) {

	  					Freq_Range[7] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F5_max") == 0) {

	  					Freq_Range[9] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "F6_max") == 0) {

	  					Freq_Range[11] = strtol(rewriteFLASH + t[i + 1].start, &ptr,
	  							0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "G1") == 0) {

	  					Gain[0] = strtol(rewriteFLASH + t[i + 1].start, &ptr, 0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "G2") == 0) {

	  					Gain[1] = strtol(rewriteFLASH + t[i + 1].start, &ptr, 0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "G3") == 0) {

	  					Gain[2] = strtol(rewriteFLASH + t[i + 1].start, &ptr, 0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "G4") == 0) {

	  					Gain[3] = strtol(rewriteFLASH + t[i + 1].start, &ptr, 0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "G5") == 0) {

	  					Gain[4] = strtol(rewriteFLASH + t[i + 1].start, &ptr, 0);

	  					i++;
	  				} else if (jsoneq(rewriteFLASH, &t[i], "G6") == 0) {

	  					Gain[5] = strtol(rewriteFLASH + t[i + 1].start, &ptr, 0);

	  					i++;
	  				}

	  				else {
	  					//printf("Unexpected key: %.*s\n", t[i].end - t[i].start,
	  					// rewriteFLASH + t[i].start);
	  				}
	  			}

	  			FlashFlag = 0;
	  		}


	  if (samplesReady) {
			PDM_Filter(&pdmRxBuf[0], &PCMOutBuffer[0], &PDM1_filter_handler);

			for (uint16_t cnt = 0; cnt < FFT_SAMPLES; cnt++) {
				//FFTInBuffer[cnt] = ((float32_t)PCMOutBuffer[cnt]);
				//FFTInBuffer[cnt] = ((float32_t)PCMOutBuffer[cnt])/65556.0; //Range : 0 - 100%
				FFTInBuffer[cnt] = ((float32_t) PCMOutBuffer[cnt]) / 32778.0; //Range : -1 - 1
			}

			fftcomplete = 1;
			samplesReady = false;
		}

		if (fftcomplete) {
			//CALCULATE FFT
			arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0);

			int FreqPoint = 0;

			// calculate abs values and linear-to-dB
			for (int i = 0; i < FFT_SAMPLES; i = i + 2) {
				Freqs[FreqPoint] = (int) (20
						* log10f(
								complexABS(FFTOutBuffer[i],
										FFTOutBuffer[i + 1])))
						- Noises_floor_offset;

				if (Freqs[FreqPoint] < 0) {
					Freqs[FreqPoint] = 0;
				}
				FreqPoint++;
			}

			// Equation which give number of element in array equal frequancy
			// f_sample = 44kHz (real value check in CubeIDE, 44 014Hz)
			// FFT_SAMPLES = 1024
			// Example 500 Hz -> 500 * (1024/(44014/2)) = 23.26 ~ 23 element -> Freqs[23] = 500 Hz
			OutFreqArray[0] = (uint8_t) Freqs[1]; // 22 Hz
			OutFreqArray[1] = (uint8_t) Freqs[3]; // 63 Hz
			OutFreqArray[2] = (uint8_t) Freqs[6]; // 125 Hz
			OutFreqArray[3] = (uint8_t) Freqs[12]; // 250 Hz
			OutFreqArray[4] = (uint8_t) Freqs[23]; // 500 Hz
			OutFreqArray[5] = (uint8_t) Freqs[47]; // 1000 Hz
			OutFreqArray[6] = (uint8_t) Freqs[102]; // 2200 Hz
			OutFreqArray[7] = (uint8_t) Freqs[209]; // 4500 Hz
			OutFreqArray[8] = (uint8_t) Freqs[372]; // 8000 Hz
			//OutFreqArray[9] = (uint8_t) Freqs[698]; // 15000 Hz

			fftcomplete = 0;
		}


	  if((HAL_GetTick() - PWMTimer) > 100){
	  	  	  	  	  	PWMTimer = HAL_GetTick();

			Amplitude_Section[0] = GetMaxAmplitude(Freq_Range[0],
					Freq_Range[1]);
			Amplitude_Section[1] = GetMaxAmplitude(Freq_Range[2],
					Freq_Range[3]);
			Amplitude_Section[2] = GetMaxAmplitude(Freq_Range[4],
					Freq_Range[5]);
			Amplitude_Section[3] = GetMaxAmplitude(Freq_Range[6],
					Freq_Range[7]);
			Amplitude_Section[4] = GetMaxAmplitude(Freq_Range[8],
					Freq_Range[9]);
			Amplitude_Section[5] = GetMaxAmplitude(Freq_Range[10],
					Freq_Range[11]);

			if (Amplitude_Section[0] > 0) {
				if (Amplitude_Section[0] >= MAX_AMPLITUDE) {
					MotorPWM[0] = MAX_PWM;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MotorPWM[0]);

				} else {
					MotorPWM[0] = (Amplitude_Section[0] / MAX_AMPLITUDE) * 100;

					if((MotorPWM[0] + Gain[0]) > MAX_PWM){
						MotorPWM[0] = MAX_PWM;
					} else if((MotorPWM[0] + Gain[0]) <= 0){
						MotorPWM[0] = 0;
					} else {
						MotorPWM[0] = MotorPWM[0] + Gain[0];
					}

					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MotorPWM[0]);
				}
			} else if (__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1) != 0) {
				MotorPWM[0] = 0;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			}

			if (Amplitude_Section[1] > 0) {
				if (Amplitude_Section[1] >= MAX_AMPLITUDE) {
					MotorPWM[1] = MAX_PWM;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MotorPWM[1]);

				} else {
					MotorPWM[1] = (Amplitude_Section[1] / MAX_AMPLITUDE) * 100;

					if((MotorPWM[1] + Gain[1]) > MAX_PWM){
						MotorPWM[1] = MAX_PWM;
					} else if((MotorPWM[1] + Gain[1]) <= 0){
						MotorPWM[1] = 0;
					} else {
						MotorPWM[1] = MotorPWM[1] + Gain[1];
					}

					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MotorPWM[1]);
				}
			} else if (__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2) != 0) {
				MotorPWM[1] = 0;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			}

			if (Amplitude_Section[2] > 0) {
				if (Amplitude_Section[2] >= MAX_AMPLITUDE) {
					MotorPWM[2] = MAX_PWM;
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MotorPWM[2]);

				} else {
					MotorPWM[2] = (Amplitude_Section[2] / MAX_AMPLITUDE) * 100;

					if((MotorPWM[2] + Gain[2]) > MAX_PWM){
						MotorPWM[2] = MAX_PWM;
					} else if((MotorPWM[2] + Gain[2]) <= 0){
						MotorPWM[2] = 0;
					} else {
						MotorPWM[2] = MotorPWM[2] + Gain[2];
					}

					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MotorPWM[2]);
				}
			} else if (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) != 0) {
				MotorPWM[2] = 0;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			}

			if (Amplitude_Section[3] > 0) {
				if (Amplitude_Section[3] >= MAX_AMPLITUDE) {
					MotorPWM[3] = MAX_PWM;
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MotorPWM[3]);

				} else {
					MotorPWM[3] = (Amplitude_Section[3] / MAX_AMPLITUDE) * 100;

					if((MotorPWM[3] + Gain[3]) > MAX_PWM){
						MotorPWM[3] = MAX_PWM;
					} else if((MotorPWM[3] + Gain[3]) <= 0){
						MotorPWM[3] = 0;
					} else {
						MotorPWM[3] = MotorPWM[3] + Gain[3];
					}

					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MotorPWM[3]);
				}
			} else if (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2) != 0) {
				MotorPWM[3] = 0;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			}

			if (Amplitude_Section[4] > 0) {
				if (Amplitude_Section[4] >= MAX_AMPLITUDE) {
					MotorPWM[4] = MAX_PWM;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MotorPWM[4]);

				} else {
					MotorPWM[4] = (Amplitude_Section[4] / MAX_AMPLITUDE) * 100;

					if((MotorPWM[4] + Gain[4]) > MAX_PWM){
						MotorPWM[4] = MAX_PWM;
					} else if((MotorPWM[4] + Gain[4]) <= 0){
						MotorPWM[4] = 0;
					} else {
						MotorPWM[4] = MotorPWM[4] + Gain[4];
					}

					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MotorPWM[4]);
				}
			} else if (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1) != 0) {
				MotorPWM[4] = 0;
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			}

			if (Amplitude_Section[5] > 0) {
				if (Amplitude_Section[5] >= MAX_AMPLITUDE) {
					MotorPWM[5] = MAX_PWM;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MotorPWM[5]);

				} else {
					MotorPWM[5] = (Amplitude_Section[5] / MAX_AMPLITUDE) * 100;

					if((MotorPWM[5] + Gain[5]) > MAX_PWM){
						MotorPWM[5] = MAX_PWM;
					} else if((MotorPWM[5] + Gain[5]) <= 0){
						MotorPWM[5] = 0;
					} else {
						MotorPWM[5] = MotorPWM[5] + Gain[5];
					}

					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MotorPWM[5]);
				}
			} else if (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2) != 0) {
				MotorPWM[5] = 0;
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
			}

		}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Jezeli taktowanie glownego zegara jest ustawione na 48MHz ustaw FLASH_LATENCY na 1 czyli FLASH_LATENCY_1

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// DFU callbacks
// Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
//--------------------------------------------------------------------+

// Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
// Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
// During this period, USB host won't try to communicate with us.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
  if ( state == DFU_DNBUSY )
  {
    // For this example
    // - Atl0 Flash is fast : 1   ms
    // - Alt1 EEPROM is slow: 100 ms
    return (alt == 0) ? 1 : 100;
  }
  else if (state == DFU_MANIFEST)
  {
    // since we don't buffer entire image and do any flashing in manifest stage
    return 0;
  }

  return 0;
}

// Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
// This callback could be returned before flashing op is complete (async).
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length)
{
  (void) alt;
  (void) block_num;

  //printf("\r\nReceived Alt %u BlockNum %u of length %u\r\n", alt, wBlockNum, length);

  for(uint16_t i=0; i<length; i++)
  {
    printf("%c", data[i]);
  }

  // flashing op for download complete without error
  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked when download process is complete, received DFU_DNLOAD (wLength=0) following by DFU_GETSTATUS (state=Manifest)
// Application can do checksum, or actual flashing if buffered entire image previously.
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_manifest_cb(uint8_t alt)
{
  (void) alt;
  printf("Download completed, enter manifestation\r\n");

  // flashing op for manifest is complete without error
  // Application can perform checksum, should it fail, use appropriate status such as errVERIFY.
  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked when received DFU_UPLOAD request
// Application must populate data with up to length bytes and
// Return the number of written bytes
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t* data, uint16_t length)
{
  (void) block_num;
  (void) length;

  uint16_t const xfer_len = (uint16_t) strlen(upload_image[alt]);
  memcpy(data, upload_image[alt], xfer_len);

  return xfer_len;
}

// Invoked when the Host has terminated a download or upload transfer
void tud_dfu_abort_cb(uint8_t alt)
{
  (void) alt;
  printf("Host aborted transfer\r\n");
}

// Invoked when a DFU_DETACH request is received
void tud_dfu_detach_cb(void)
{
  printf("Host detach, we should probably reboot\r\n");
}

//--------------------------------------------------------------------+
// BLINKING TASK + Indicator pulse
//--------------------------------------------------------------------+

void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( HAL_GetTick() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led_state);
  //board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	samplesReady = true;
}

float32_t GetMaxAmplitude(uint16_t lowFreq, uint16_t highFreq){

	// index1 + 12 = Example 500 Hz -> 500 * (1024/(44014/2)) = 23.26 ~ 23 element -> Freqs[23] = 500 Hz
	uint16_t indexarrayforlowFreq = lowFreq * (float)((float)FFT_SAMPLES/((float)REAL_VALUE_FREQ_SAMPLING/2.f));
	uint16_t indexarrayforhighFreq = highFreq * (float)((float)FFT_SAMPLES/((float)REAL_VALUE_FREQ_SAMPLING/2.f));
	//uint32_t blocksize = indexarrayforhighFreq - indexarrayforlowFreq;
	if(indexarrayforlowFreq <= 0){
		indexarrayforlowFreq = 0;
	} else {
		indexarrayforlowFreq -= 5;
	}

	if(indexarrayforlowFreq > 505){
		indexarrayforlowFreq = 512;
	} else {
		indexarrayforlowFreq += 5;
	}

	//float32_t amplitude = 0;
	//uint32_t currentIndex = 0;

		/**
		 * @brief Maximum value of a floating-point vector.
		 * @param[in]  pSrc       points to the input buffer
		 * @param[in]  blockSize  length of the input vector
		 * @param[out] pResult    maximum value returned here
		 * @param[out] pIndex     index of maximum value returned here
		 */
			// RANGE of Array \/
	//arm_max_f32((float32_t *)&Freqs[indexarrayforlowFreq], blocksize, &amplitude, &currentIndex);

	//int TempArray[512] = {0};
	//memset(TempArray, 0, 512);
	//memcpy(TempArray, Freqs, 512);

	  for (int i = indexarrayforlowFreq; i < indexarrayforhighFreq; ++i) {
	    if (Freqs[indexarrayforlowFreq] < Freqs[i]) {
	    	Freqs[indexarrayforlowFreq] = Freqs[i];
	    }
	  }

return Freqs[indexarrayforlowFreq];

}

void InitParams(){

	Freq_Range[0] = 400;
	Freq_Range[1] = 600;

	Freq_Range[2] = 700;
	Freq_Range[3] = 900;

	Freq_Range[4] = 1400;
	Freq_Range[5] = 1600;

	Freq_Range[6] = 1900;
	Freq_Range[7] = 2100;

	Freq_Range[8] = 2400;
	Freq_Range[9] = 2600;

	Freq_Range[10] = 900;
	Freq_Range[11] = 1100;


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
