/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <adxl345.h>
#include <arm_math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t fft_full_flag = 0; // flag when data for FFT is ready
arm_rfft_fast_instance_f32 fftHandler;

float fft_out[FFT_BUFFER_SIZE];

typedef struct {
    uint16_t header;      	// 2 bytes
    uint16_t msg_id;      	// 2 byte
    uint16_t length;  		// 2 bytes
} __attribute__((packed)) packet_t;  // packed to avoid padding

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Function for handling external callbacks
  *
  */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if(pin == ADXL_INT1_Pin){
		if (fifo_collecting) return; // avoid overlap
		CollectFifo();
	}
}

int __io_putchar(int ch)
{
    if (ch == '\n') {
        __io_putchar('\r');
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

/**
  * @brief  Calculate fft and send it with UART
  * We also apply Hann window and normalize.
  *
  */
void send_fft(void) {
    const uint32_t N = FFT_BUFFER_SIZE;
    float *bu = use_one ? z_array_1_fft : z_array_2_fft;

    // Optional: subtract mean (DC remove) and apply Hann window
    static float windowed[FFT_BUFFER_SIZE];
    float mean = 0.0f;
    for (uint32_t i = 0; i < N; ++i) mean += bu[i];
    mean /= (float)N;
    for (uint32_t i = 0; i < N; ++i) {
        // Hann window: w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
        float w = 0.5f * (1.0f - cosf(2.0f * 3.14159265358979323846f * i / (N - 1)));
        windowed[i] = (bu[i] - mean) * w;
    }

    // Run real FFT in-place (input -> fft_out)
    arm_rfft_fast_f32(&fftHandler, windowed, fft_out, 0);

    // calculate number of unique bins
    const uint32_t nbins = (N / 2) + 1;
    static float fft_mag[(FFT_BUFFER_SIZE/2) + 1];

    fft_mag[0] = fabsf(fft_out[0]);

    // k = N/2 (Nyquist) : real = out[1]
    fft_mag[nbins - 1] = fabsf(fft_out[1]);

    // k = 1 .. N/2 - 1 : packed as (real, imag) starting at index 2
    for (uint32_t k = 1; k < (N / 2); ++k) {
        float re = fft_out[2 * k];
        float im = fft_out[2 * k + 1];
        fft_mag[k] = sqrtf(re * re + im * im);
    }

    // Normalize to get amplitude (single-sided):
    for (uint32_t k = 0; k < nbins; ++k) {
        fft_mag[k] /= (float)N; // scale
        if (k != 0 && k != (nbins - 1)) fft_mag[k] *= 2.0f;
    }

    // Prepare header: length is number of floats (nbins)
    packet_t hdr;
    hdr.header = 0x6969;
    hdr.msg_id = 1; // FFT
    hdr.length = (uint16_t)nbins; // 257 for 512-point FFT

    // Send header then data
    HAL_UART_Transmit(&huart2, (uint8_t*)&hdr, sizeof(hdr), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)fft_mag, hdr.length * sizeof(float), HAL_MAX_DELAY);
}

/**
  * @brief  Send data in time domain with UART
  *
  */
void send_time_domain(void){
    float *data = use_one ? z_array_1_timeD: z_array_2_timeD;  // max size, adjust as needed
    // Prepare header
    packet_t hdr;
    hdr.header = 0x6969;
    hdr.msg_id = 2;
    hdr.length = FFT_BUFFER_SIZE;

    // Send header
    HAL_UART_Transmit(&huart2, (uint8_t*)&hdr, sizeof(hdr), HAL_MAX_DELAY);

    // Send data
    HAL_UART_Transmit(&huart2, (uint8_t*)data, hdr.length * 4, HAL_MAX_DELAY);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  for (uint8_t i = 0; i < 4; i++) {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    HAL_Delay(100);
  }

  printf("Searching...\n");
  if (adxl345_init() == HAL_OK) {
	  printf("OK: ADXL345\n");
  } else {
	  printf("Error: ADXL345 not found\n");
  }

  arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (fft_full_flag == 1){
		  HAL_IWDG_Refresh(&hiwdg);
		  send_fft();
		  send_time_domain();
		  fft_full_flag = 0;
		  use_one ^= 1; // Change to point to other z_arrays
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
    __disable_irq();
    while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
    /* Optional: report the file and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
