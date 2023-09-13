/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************

 PROJETO CARRINHO COM SENSOR ULTRASSONICO
 HENRIQUE RAUPP E DAVI FAGUNDES
 4422



 MOTOR_LADO_DIREITO 1 = PC0 = IN4
 MOTOR_LADO_DIREITO 2 = PC1 = IN3

 MOTOR_LADO_ESQUERDO 1 = PC2 = IN2
 MOTOR_LADO_ESQUERDO 2 = PC3 = IN1


 PA10 = PWM motor direito = TIM1 Ch 3
 PA11 = PWM motor esquerdo = TIM1 Ch 4



 MOTOR	IN1 	IN2

 hora	H		L
 ant.h	L		H
 fre	L		L



 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define tout 100
#define tam_msg 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t tempo_ativo;

uint32_t periodo;

float tempo_alto;

uint8_t msg[100];

uint32_t distancia;

uint8_t cont;
uint8_t cont2;


enum {frente = 0, parado, redirecionando};

uint8_t estado_atual = frente;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//============================= Funcoes comandos do carrinho =======================//
void Pra_Frente() {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //IN1 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); //IN2 low

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); //IN3 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); //IN4 low

}

void Parado() {

	//while (func_stateP = 1) {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //IN1 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); //IN2 low

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //IN3 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); //IN4 high

	//}
}

void Pra_Esquerda() {

	//while (func_state = 1) {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //IN1 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); //IN2 low

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //IN3 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); //IN4 high
	//}

}




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

//============================= Calculando largura de pulso do sensor ==========================//

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {

		tempo_alto = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
		distancia = tempo_alto / 58;
	}

}

//============================= Enviando pela serial =============================//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		sprintf(msg, "\rDistancia: %i cm", distancia);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), tout);
		msg[0] = '/0';
	}


//============================= Comando caso haja obstaculo =======================//

	if (htim->Instance == TIM11) {

		cont++;
		cont2++;
	}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim10); // Serial
	HAL_TIM_Base_Start_IT(&htim11);// Base de tempo

///////////////////SONAR////////////////////////////////////
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //leitura sonar
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);// Combinado com o canal 1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //geracao trig sonnar

/////////////////////Vel.Motor//////////////////////////
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // velocidade motor direito   25kHz
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // velocidade motor esquerdo  25kHz

	//strncpy(msg, "Raupp - Tro \n Medicao de DC, Periodo e Tempo ativo\n\r", tam_msg);
	//HAL_UART_Transmit(&huart2, msg , strlen(msg), tout); // transmitindo msg
	// strncpy(msg, " ", tam_msg); // limpando a variavel mensagem para evitar escrita por cima

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//estado_atual = frente;

	while (1) {

		switch (estado_atual) {

		case frente:

			Pra_Frente();

			if (distancia <= 20){
				cont = 0;
				estado_atual = parado;}

			break;

		case parado:
		//cont = 0;

			Parado();

			if (cont >= 10) {
				cont = 0;
				cont2 = 0;
				estado_atual = redirecionando;}

			break;


		case redirecionando:
			//cont2 = 0;

			if (cont2 >= 15) {
				cont2 = 0;
				estado_atual = frente;
			}
			else
				Pra_Esquerda();

			break;

		default:

			Parado();

			break;

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
