/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @brief Controle de temperatura com STM32 + FreeRTOS
 *
 * Funcionalidades:
 * - Leitura de temperatura via ADC (sensor LM35)
 * - Filtro de média móvel (N = 100 amostras)
 * - Controle PID em tempo real
 * - Atuação via PWM (TIM3)
 *
 * Projeto acadêmico – Engenharia de Controle e Automação
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

/* USER CODE BEGIN Includes */
#include <string.h>
#include "queue.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Defines */
#define N 100   // Número de amostras utilizadas no filtro de média móvel
/* USER CODE END Defines */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN Variaveis */
QueueHandle_t adcQueue;        // Fila para valores lidos do ADC (temperatura)
QueueHandle_t filteredQueue;   // Fila para valores filtrados (média móvel)

uint16_t adc_values[N];        // Buffer circular da média móvel
float filtered_value;          // Valor filtrado (média)
/* USER CODE END Variaveis */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototipos */
void vReadADC(void *pvParameters);
void vMovingAverage(void *pvParameters);
void vPID(void *pvParameters);
/* USER CODE END Prototipos */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* ============================================================= */
/* ======================== MAIN =============================== */
/* ============================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    /* Inicializa PWM */
    TIM3->CCR2 = 40;                            // Duty cycle inicial
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Inicia PWM

    /* Criação das filas */
    adcQueue = xQueueCreate(10, sizeof(uint16_t));
    filteredQueue = xQueueCreate(10, sizeof(uint16_t));

    if (adcQueue == NULL || filteredQueue == NULL)
    {
        Error_Handler(); // Falha na criação das filas
    }

    /* Criação das tasks */
    xTaskCreate(vReadADC, "Leitura ADC", 128, NULL, 1, NULL);
    xTaskCreate(vMovingAverage, "Media Movel", 128, NULL, 1, NULL);
    xTaskCreate(vPID, "Controle PID", 128, NULL, 1, NULL);

    /* Inicia o escalonador */
    vTaskStartScheduler();

    /* Nunca deve chegar aqui */
    while (1) {}
}

/* ============================================================= */
/* ======================== TASKS ============================== */
/* ============================================================= */

/**
 * @brief Task responsável pela leitura do ADC
 * Converte a leitura do LM35 para temperatura em °C
 */
void vReadADC(void *pvParameters)
{
    uint16_t raw_adc;
    uint16_t temperatura_celsius;

    while (1)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        raw_adc = HAL_ADC_GetValue(&hadc1);

        /* Conversão LM35: 10 mV / °C */
        temperatura_celsius = (raw_adc * 3300 / 4095) / 10;

        xQueueSend(adcQueue, &temperatura_celsius, portMAX_DELAY);
    }
}

/**
 * @brief Task responsável pelo filtro de média móvel
 * Recebe temperatura bruta e envia temperatura filtrada
 */
void vMovingAverage(void *pvParameters)
{
    uint16_t novo_valor;
    float acumulador;
    char msg[10];

    static TickType_t ultimo_envio = 0;
    const TickType_t intervalo_envio = pdMS_TO_TICKS(1000); // 1 segundo

    while (1)
    {
        if (xQueueReceive(adcQueue, &novo_valor, portMAX_DELAY) == pdPASS)
        {
            /* Desloca os valores antigos */
            for (int i = N - 1; i > 0; i--)
            {
                adc_values[i] = adc_values[i - 1];
            }

            adc_values[0] = novo_valor;

            /* Calcula média */
            acumulador = 0.0f;
            for (int i = 0; i < N; i++)
            {
                acumulador += adc_values[i];
            }

            filtered_value = acumulador / N;
            uint16_t filtered_int = (uint16_t) round(filtered_value);

            xQueueSend(filteredQueue, &filtered_int, portMAX_DELAY);

            /* Envio serial a cada 1 segundo */
            if ((xTaskGetTickCount() - ultimo_envio) >= intervalo_envio)
            {
                sprintf(msg, "%hu\r\n", filtered_int);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
                ultimo_envio = xTaskGetTickCount();
            }

            vTaskDelay(1);
        }
    }
}

/**
 * @brief Task de controle PID
 * Calcula o controle e aplica no PWM
 */
void vPID(void *pvParameters)
{
    TickType_t tempo_anterior, tempo_atual;
    uint16_t temperatura;

    int erro, erro_anterior = 0;
    float dt;

    float integral = 0;
    int saida, saida_ideal;

    const uint16_t setpoint = 45; // Temperatura desejada (°C)

    /* Ganhos PID */
    const float kp = 100.0f;
    const float ki = 2.0f;
    const float kd = 0.5f;

    tempo_anterior = xTaskGetTickCount();

    while (1)
    {
        if (xQueueReceive(filteredQueue, &temperatura, portMAX_DELAY) == pdPASS)
        {
            tempo_atual = xTaskGetTickCount();
            dt = (tempo_atual - tempo_anterior) * portTICK_PERIOD_MS / 1000.0f;

            if (dt < 0.01f)
                dt = 0.01f;

            tempo_anterior = tempo_atual;

            erro = setpoint - temperatura;

            integral += erro * dt;
            float derivativo = (erro - erro_anterior) / dt;
            erro_anterior = erro;

            saida_ideal = (kp * erro) + (ki * integral) + (kd * derivativo);

            /* Saturação + anti-windup */
            if (saida_ideal > 100)
            {
                saida = 100;
                integral -= (saida_ideal - 100) / ki;
            }
            else if (saida_ideal < 0)
            {
                saida = 0;
                integral -= (saida_ideal) / ki;
            }
            else
            {
                saida = saida_ideal;
            }

            TIM3->CCR2 = saida;
        }
    }
}

/* ============================================================= */
/* ===================== CALLBACKS ============================== */
/* ============================================================= */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
}

/* ============================================================= */
/* ===================== ERROR HANDLER ========================== */
/* ============================================================= */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1600 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 99;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 57600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
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
