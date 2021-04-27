/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USB_HID_SCAN_NEXT 0x01
#define USB_HID_SCAN_PREV 0x02
#define USB_HID_STOP      0x04
#define USB_HID_EJECT     0x08
#define USB_HID_PAUSE     0x10
#define USB_HID_MUTE      0x20
#define USB_HID_VOL_UP    0x40
#define USB_HID_VOL_DEC   0x80

// USB keyboard codes
#define USB_HID_MODIFIER_LEFT_CTRL   0x01
#define USB_HID_MODIFIER_LEFT_SHIFT  0x02
#define USB_HID_MODIFIER_LEFT_ALT    0x04
#define USB_HID_MODIFIER_LEFT_GUI    0x08 // (Win/Apple/Meta)
#define USB_HID_MODIFIER_RIGHT_CTRL  0x10
#define USB_HID_MODIFIER_RIGHT_SHIFT 0x20
#define USB_HID_MODIFIER_RIGHT_ALT   0x40
#define USB_HID_MODIFIER_RIGHT_GUI   0x80
#define USB_HID_KEY_L     0x0F

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

//void setCount(int state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile long pause    = 50;
volatile long lastTurn = 0;
volatile int state = 0; 
volatile int count = 0; 
int actualcount    = 0; 


volatile int rotation_side = 0;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	  // HID Mouse
  struct mouseHID_t {
      uint8_t buttons;
      int8_t x;
      int8_t y;
      int8_t wheel;
  };
  struct mouseHID_t mouseHID;
  mouseHID.buttons = 0;
  mouseHID.x = 0;
  mouseHID.y = 0;
  mouseHID.wheel = 0;
	int temp = 1;
	
	
	struct mediaHID_t {
    uint8_t id;
    uint8_t keys;
  };
	
	struct mediaHID_t mediaHID;
  mediaHID.id = 2;
  mediaHID.keys = 0;
	
	int count;
	int mute_state = 1, next_state = 1, play_state = 1, rev_state = 1;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	//HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			if(rotation_side > 0){
				mediaHID.keys = USB_HID_VOL_UP;
				//for(int i = 0; i < rotation_side; i++){
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
					HAL_Delay(10);
				//}
				rotation_side = 0;
			}
			else if (rotation_side < 0){
				mediaHID.keys = USB_HID_VOL_DEC;
				//for(int i = 0; i > rotation_side; i--){
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
					HAL_Delay(10);
				//}
				rotation_side = 0;
			} 
		
		
		if(HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)){
			if (mute_state){
				mediaHID.keys = USB_HID_MUTE;
				mute_state = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);
				mediaHID.keys = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);

			}
		}
		else{
			mute_state = 1;
		}
		
		if(HAL_GPIO_ReadPin(BTN_NEXT_GPIO_Port, BTN_NEXT_Pin)){
			if (next_state){
				mediaHID.keys = USB_HID_SCAN_NEXT;
				next_state = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);
				mediaHID.keys = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);

			}
		}
		else{
			next_state = 1;
		}
		
		
		if(HAL_GPIO_ReadPin(BTN_PLAY_GPIO_Port, BTN_PLAY_Pin)){
			if (play_state){
				mediaHID.keys = USB_HID_PAUSE;
				play_state = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);
				mediaHID.keys = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);

			}
		}
		else{
			play_state = 1;
		}
		
		
		if(HAL_GPIO_ReadPin(BTN_REV_GPIO_Port, BTN_REV_Pin)){
			if (rev_state){
				mediaHID.keys = USB_HID_SCAN_PREV;
				rev_state = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);
				mediaHID.keys = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
				HAL_Delay(30);

			}
		}
		else{
			rev_state = 1;
		}
		
		
			
				// Send HID report
	//		mediaHID.keys = USB_HID_VOL_DEC;
		//	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
	//		HAL_Delay(30);
	//		mediaHID.keys = 0;
	//		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
	//		HAL_Delay(30);

		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_BTN_Pin BTN_PLAY_Pin BTN_NEXT_Pin BTN_REV_Pin */
  GPIO_InitStruct.Pin = ENC_BTN_Pin|BTN_PLAY_Pin|BTN_NEXT_Pin|BTN_REV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_A_Pin ENC_B_Pin */
  GPIO_InitStruct.Pin = ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = ENC_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GetTick() - lastTurn < pause) return;
	__disable_irq();
	GPIO_PinState pinAValue,pinBValue ; 
	pinAValue = HAL_GPIO_ReadPin(ENC_STR_GPIO_Port, ENC_STR_Pin);
	pinBValue = HAL_GPIO_ReadPin(ENC_REV_GPIO_Port, ENC_REV_Pin);
	if(GPIO_Pin==ENC_STR_Pin){
		if (state == 0  && !pinAValue &&  pinBValue || state == 2  && pinAValue && !pinBValue) {
    state += 1; // ???? ??????????? ???????, ?????????? ?????????? state
    lastTurn = HAL_GetTick();
  }
  if (state == -1 && !pinAValue && !pinBValue || state == -3 && pinAValue &&  pinBValue) {
    state -= 1; // ???? ??????????? ???????, ?????????? ? ????? ?????????? state
    lastTurn = HAL_GetTick();
  }
  setCount(state); // ????????? ?? ???? ?? ??????? ???? ?? 4 ????????? ???????? (2 ?????????)
  if (pinAValue && pinBValue && state != 0) state = 0; // ???? ???-?? ????? ?? ???, ?????????? ?????? ? ???????? ?????????
		
	}
	if(GPIO_Pin==ENC_REV_Pin){
		if (state == 1 && !pinAValue && !pinBValue || state == 3 && pinAValue && pinBValue) {
    state += 1; // ???? ??????????? ???????, ?????????? ?????????? state
    lastTurn = HAL_GetTick();
  }
  if (state == 0 && pinAValue && !pinBValue || state == -2 && !pinAValue && pinBValue) {
    state -= 1; // ???? ??????????? ???????, ?????????? ? ????? ?????????? state
    lastTurn = HAL_GetTick();
  }
  setCount(state); // ????????? ?? ???? ?? ??????? ???? ?? 4 ????????? ???????? (2 ?????????)
  if (pinAValue && pinBValue && state != 0) state = 0; // ???? ???-?? ????? ?? ???, ?????????? ?????? ? ???????? ?????????
	}
	 __enable_irq();
}

void setCount(int state) {          // ????????????? ???????? ????????
  if (state == 4 || state == -4) {  // ???? ?????????? state ??????? ???????? ???????? ??????????
    count += (int)(state / 4);      // ???????????/????????? ???????
    lastTurn = HAL_GetTick();            // ?????????? ????????? ?????????
  }
}
*/
//AB
//00 state 0
//01 state 1	10 state 4
//11 state 2	11 state 5
//10 state 3	01 state 6
//00 state 4	00 state 9




void b_fall(void){
	if (state == 2 && !HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
		rotation_side += -1;
		state = 0;
	}
	else if (state == 4 && !HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
		rotation_side += 1;
		state = 0;
	}
	else {
		state = 0;
	}
}



void b_rise(void){
	if (state == 0 && !HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
		state = 3;
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = ENC_A_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else if (state == 0 && HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
		state = 1;
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = ENC_A_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else {
		state = 0;
	}
}

void a_fall(void){
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = ENC_A_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	if (state == 1 && HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
		state = 2;
	}
	else{
		state = 0;
	}

}

void a_rise(void){
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = ENC_A_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	if (state == 3 && HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
		state = 4;
	}
	else{
		state = 0;
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==ENC_A_Pin){
		for(int i = 0; i < 100; i++){};
		if (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
			a_rise();
		}
		else{
			a_fall();
		}
	}else{
		for(int i = 0; i < 100; i++){};
		if (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
			b_rise();
		}
		else{
			b_fall();
		}
	}

}




//void b_fall(void){
//	if (state == 2 && HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
//		state = 3;
//	}
//	else if (state == 6 && !HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
//		rotation_side += -1;
//		state = 0;
//	}
//	else {
//		state = 0;
//	}
//}



//void b_rise(void){
//	if (state == 4 && HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
//		state = 5;
//	}
//	else if (state == 0 && !HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
//		state = 1;
//	}
//	else {
//		state = 0;
//	}
//}



//void a_fall(void){
//	if (state == 3 && !HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
//		rotation_side += 1;
//		state = 0;
//	}
//	else if (state == 5 && HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
//		state = 6;
//	}
//	else {
//		state = 0;
//	}
//}

//void a_rise(void){
//	if (state == 0 && !HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
//		state = 4;
//	}
//	else if (state == 1 && HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
//		state = 2;
//	}
//	else {
//		state = 0;
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
