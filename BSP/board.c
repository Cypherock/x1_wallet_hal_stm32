
#include <stdio.h>
#include "board.h"
#include "stm32l4xx_hal.h"
#include "sdk_config.h"
#include "libusb.h"

UART_HandleTypeDef hlpuart1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
RNG_HandleTypeDef hrng;

uint32_t joystick_pressed = 0xff;
uint32_t buzzer_counter=0, bsp_tick=0;
atecc_interface_type atecc_mode=0; 
#define LPUART_TIMEOUT	5

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	  SCB->VTOR = FIRMWARE_START_ADDRESS;
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
	                              |RCC_PERIPHCLK_USB;
	  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
	  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure the main internal regulator output voltage
	  */
	  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_PWR_EnableBkUpAccess();
}

void BSP_USB_Clock_Init(){
    HAL_PWR_EnableBkUpAccess();

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable USB power on Pwrctrl CR2 register */
    HAL_PWREx_EnableVddUSB();
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void BSP_LPUART1_UART_Init(uint32_t baudrate)
{

  /* USER CODE BEGIN USART3_Init 0 */
  HAL_UART_DeInit(&hlpuart1);
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = baudrate;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OverSampling = UART_OVERSAMPLING_16;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_EnableReceiverTimeout(&hlpuart1);
	HAL_UART_ReceiverTimeout_Config(&hlpuart1, 24);
  /* USER CODE END USART3_Init 2 */

}

void BSP_LPUART1_UART_DeInit(){
	HAL_UART_DeInit(&hlpuart1);
}

uint8_t BSP_LPUART1_UART_Write(uint8_t* data, uint16_t size){
	bsp_tick = LPUART_TIMEOUT;

    while(!(hlpuart1.Instance->ISR & UART_FLAG_TXE)) {
        // wait for tx empty
        if(bsp_tick == 0)
			return 1;
    }
    hlpuart1.Instance->TDR = *data;
	return 0;
}

uint8_t BSP_LPUART1_UART_ClearReadBuff(){
	//set timeout value
    bsp_tick = LPUART_TIMEOUT;

    while(!(hlpuart1.Instance->ISR & UART_FLAG_TC)) {
        //wait for transmission complete
		if(bsp_tick == 0){
			break;
		}
    }

    // clear rx buffer
    hlpuart1.Instance->RQR = USART_RQR_RXFRQ;

    // clear overrun error
    // clear rx timeout flag
    // clear framing error
    hlpuart1.Instance->ICR = USART_ICR_ORECF | USART_ICR_RTOCF | USART_ICR_FECF;
}


uint8_t BSP_LPUART1_UART_Read(uint8_t* data, uint16_t size){
	//set timeout value
    bsp_tick = LPUART_TIMEOUT;

    while(bsp_tick != 0) {
		if(hlpuart1.Instance->ISR & UART_FLAG_RXNE) {
			*data = hlpuart1.Instance->RDR & 0x7f;
			return 0;
		}
		if(hlpuart1.Instance->ISR & UART_FLAG_RTOF) {
			return hlpuart1.Instance->ICR = USART_ICR_RTOCF;
		}
    }

    return 1;
}



/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void BSP_I2C1_Init(void)
{
	  hi2c1.Instance = BSP_PN532_OLED_I2C;
	  hi2c1.Init.Timing = BSP_PN532_OLED_I2C_SPEED;
	  hi2c1.Init.OwnAddress1 = 0x3F;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0xFF;
	  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }



	  /* USER CODE BEGIN I2C2_Init 0 */

	  /* USER CODE END I2C2_Init 0 */

	  /* USER CODE BEGIN I2C2_Init 1 */

	  /* USER CODE BEGIN I2C2_Init 2 */

	  /* USER CODE END I2C2_Init 2 */

}


uint8_t BSP_I2C1_IO_Write(uint16_t devAddress, uint8_t *pData , uint8_t size)
{
	/* TODO: Add error handling for i2c transactions */
	return HAL_I2C_Master_Transmit(&hi2c1, devAddress, pData, size, 1000);
}

uint8_t BSP_I2C1_IO_Read(uint16_t devAddress, uint8_t *pData , uint8_t size)
{
	/* TODO: Add error handling for i2c transactions */
	return HAL_I2C_Master_Receive(&hi2c1, devAddress, pData, size, 100);
}

void BSP_I2C1_IO_Read_MEM(uint16_t devAddress, uint16_t memAddress, uint8_t *pData , uint8_t size)
{
	/* TODO: Add error handling for i2c transactions */
	return HAL_I2C_Mem_Read(&hi2c1, devAddress, memAddress, I2C_MEMADD_SIZE_8BIT, pData, size, 1000);
}

void BSP_I2C1_AddressScan(void)
{
	HAL_StatusTypeDef result;
	uint8_t i;
	for (i=1; i<128; i++)
	{
	  /*
	   * the HAL wants a left aligned i2c address
	   * &hi2c1 is the handle
	   * (uint16_t)(i<<1) is the i2c address left aligned
	   * retries 2
	   * timeout 2
	   */
	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
	  if (result == HAL_OK)
	  {
		  if(i==0x24 || i==0x3c){
			  BSP_TIM2_PWM_Start(200);
			  BSP_DelayMs(1000);
		  }
	  }
	}
}

/**
  * @brief I2C2 Initialization Function
  * @param timing   for selecting I2C speed standard or fast
  *                 standard:-  BSP_ATECC_I2C_MODE_STANDARD
  *                 fast:-      BSP_ATECC_I2C_MODE_FAST
  * @retval None
  */
void BSP_I2C2_Init(uint32_t timing)
{
	  /* USER CODE BEGIN I2C2_Init 0 */
	  if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE END I2C3_Init 0 */

	  /* USER CODE BEGIN I2C3_Init 1 */

	  /* USER CODE END I2C3_Init 1 */
	  hi2c2.Instance = BSP_ATECC_I2C;
	  hi2c2.Init.Timing = timing;
	  hi2c2.Init.OwnAddress1 = 0x3F;
	  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c2.Init.OwnAddress2 = 0xFF;
	  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN I2C3_Init 2 */

	  /* USER CODE END I2C3_Init 2 */

}

void BSP_I2C2_DeInit(){
	if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
}

uint8_t BSP_I2C2_IO_Write(uint16_t devAddress, uint8_t *pData , uint8_t size)
{
	/* TODO: Add error handling for i2c transactions */
	return HAL_I2C_Master_Transmit(&hi2c2, devAddress, pData, size, 1000);
}

uint8_t BSP_I2C2_IO_Read(uint16_t devAddress, uint8_t *pData , uint8_t size)
{
	/* TODO: Add error handling for i2c transactions */
	return HAL_I2C_Master_Receive(&hi2c2, devAddress, pData, size, 100);
}

void BSP_I2C2_AddressScan(void)
{
	HAL_StatusTypeDef result;
	uint8_t i;
	for (i=1; i<128; i++)
	{
	  /*
	   * the HAL wants a left aligned i2c address
	   * &hi2c2 is the handle
	   * (uint16_t)(i<<1) is the i2c address left aligned
	   * retries 2
	   * timeout 2
	   */
	  result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 2, 2);
	  if (result == HAL_OK)
	  {
		  if(i==0x60){
			  BSP_TIM2_PWM_Start(100);
			  BSP_DelayMs(500);
		  }
	  }
	}
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void BSP_TIM2_Init(void)
{

	  /* USER CODE BEGIN TIM2_Init 0 */

	  /* USER CODE END TIM2_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};

	  /* USER CODE BEGIN TIM2_Init 1 */

	  /* USER CODE END TIM2_Init 1 */
	  htim2.Instance = BSP_BUZZER_TIMER;
	  htim2.Init.Prescaler = BSP_BUZZER_TIMER_PRESCALAR;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = BSP_BUZZER_TIMER_PERIOD;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = BSP_BUZZER_TIMER_COUNTER;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, BSP_BUZZER_PWM_CHANNEL) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM2_Init 2 */

	  /* USER CODE END TIM2_Init 2 */
	  HAL_TIM_MspPostInit(&htim2);

}

void BSP_TIM3_Base_Start_IT()
{
    HAL_TIM_Base_Start_IT(&htim3);
}


/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void BSP_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

void BSP_TIM6_Init(void)
{

	  /* USER CODE BEGIN TIM6_Init 0 */

	  /* USER CODE END TIM6_Init 0 */

	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM6_Init 1 */

	  /* USER CODE END TIM6_Init 1 */
	  htim6.Instance = TIM6;
	  htim6.Init.Prescaler = 0;
	  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim6.Init.Period = 79;
	  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM6_Init 2 */
	  HAL_TIM_Base_Stop(&htim6);
	  /* USER CODE END TIM6_Init 2 */

}


uint32_t delay_us_counter=0;
void BSP_DelayCounterDec()
{
	if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
			if (delay_us_counter)
			{
			    delay_us_counter--;
			}
		}
	}
}

void BSP_DelayUs(uint32_t delay)
{
	delay_us_counter=delay;
	HAL_TIM_Base_Start(&htim6);
	do{
		if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
			delay_us_counter--;
	}while(delay_us_counter != 0);
	HAL_TIM_Base_Stop(&htim6);

}

void BSP_TIM2_PWM_Start(uint32_t buzzer_on_time)
{
	buzzer_counter=buzzer_on_time;
    HAL_TIM_PWM_Start(&htim2, BSP_BUZZER_PWM_CHANNEL);
}

void BSP_TIM2_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, BSP_BUZZER_PWM_CHANNEL);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void BSP_GPIO_Init(uint32_t hardware_version)
{

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  if(hardware_version == 0){
		  hardware_version = DEVICE_HARDWARE_STM32_3;
	  }

#ifdef NDEBUG
    /**
     * Set state of all unused GPIOs as Input pulled up in release builds to achieve best
     * EMI/EMC results.
     */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
      HAL_PWREx_EnableVddIO2();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /*Configure GPIO pins : Unused_Pin UnusedE1_Pin UnusedE4_Pin UnusedE2_Pin
            UnusedE5_Pin UnusedE0_Pin UnusedE6_Pin UnusedE8_Pin
            UnusedE10_Pin UnusedE12_Pin UnusedE7_Pin UnusedE9_Pin
            UnusedE11_Pin UnusedE13_Pin UnusedE14_Pin UnusedE15_Pin */
    GPIO_InitStruct.Pin = Unused_Pin|UnusedE1_Pin|UnusedE4_Pin|UnusedE2_Pin
                |UnusedE5_Pin|UnusedE0_Pin|UnusedE6_Pin|UnusedE8_Pin
                |UnusedE10_Pin|UnusedE12_Pin|UnusedE7_Pin|UnusedE9_Pin
                |UnusedE11_Pin|UnusedE13_Pin|UnusedE14_Pin|UnusedE15_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : UnusedB8_Pin UnusedB3_Pin UnusedB9_Pin
                UnusedB5_Pin UnusedB15_Pin UnusedB14_Pin UnusedB13_Pin
                UnusedB2_Pin UnusedB10_Pin UnusedB12_Pin UnusedB0_Pin
                UnusedB1_Pin */
    GPIO_InitStruct.Pin = UnusedB8_Pin|UnusedB3_Pin|UnusedB9_Pin
                |UnusedB5_Pin|UnusedB15_Pin|UnusedB14_Pin|UnusedB13_Pin
                |UnusedB2_Pin|UnusedB10_Pin|UnusedB12_Pin|UnusedB0_Pin
                |UnusedB1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : UnusedD7_Pin UnusedD5_Pin UnusedD6_Pin UnusedD4_Pin
                UnusedD3_Pin UnusedD1_Pin UnusedD2_Pin UnusedD0_Pin
                UnusedD15_Pin UnusedD14_Pin UnusedD13_Pin UnusedD12_Pin
                UnusedD11_Pin UnusedD10_Pin UnusedD9_Pin UnusedD8_Pin */
    GPIO_InitStruct.Pin = UnusedD7_Pin|UnusedD5_Pin|UnusedD6_Pin|UnusedD4_Pin
                |UnusedD3_Pin|UnusedD1_Pin|UnusedD2_Pin|UnusedD0_Pin
                |UnusedD15_Pin|UnusedD14_Pin|UnusedD13_Pin|UnusedD12_Pin
                |UnusedD11_Pin|UnusedD10_Pin|UnusedD9_Pin|UnusedD8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : UnusedA15_Pin UnusedA14_Pin UnusedA13_Pin UnusedA10_Pin
                UnusedA9_Pin UnusedA8_Pin UnusedA4_Pin UnusedA7_Pin
                UnusedA2_Pin UnusedA5_Pin UnusedA3_Pin UnusedA6_Pin
                UnusedA1_Pin */
    GPIO_InitStruct.Pin = UnusedA15_Pin|UnusedA14_Pin|UnusedA13_Pin|UnusedA10_Pin
                |UnusedA9_Pin|UnusedA8_Pin|UnusedA4_Pin|UnusedA7_Pin
                |UnusedA2_Pin|UnusedA5_Pin|UnusedA3_Pin|UnusedA6_Pin
                |UnusedA1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : UnusedC12_Pin UnusedC10_Pin UnusedC13_Pin UnusedC11_Pin
                UnusedC14_Pin UnusedC9_Pin UnusedC15_Pin UnusedC0_Pin
                UnusedC1_Pin UnusedC2_Pin UnusedC3_Pin */
    GPIO_InitStruct.Pin = UnusedC12_Pin|UnusedC10_Pin|UnusedC13_Pin|UnusedC11_Pin
                |UnusedC14_Pin|UnusedC9_Pin|UnusedC15_Pin|UnusedC0_Pin
                |UnusedC1_Pin|UnusedC2_Pin|UnusedC3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : UnusedG14_Pin UnusedG13_Pin UnusedG12_Pin UnusedG10_Pin
                UnusedG9_Pin UnusedG5_Pin UnusedG3_Pin UnusedG4_Pin
                UnusedG11_Pin UnusedG6_Pin UnusedG1_Pin UnusedG2_Pin
                UnusedG7_Pin UnusedG0_Pin UnusedG8_Pin UnusedG15_Pin */
    GPIO_InitStruct.Pin = UnusedG14_Pin|UnusedG13_Pin|UnusedG12_Pin|UnusedG10_Pin
                |UnusedG9_Pin|UnusedG5_Pin|UnusedG3_Pin|UnusedG4_Pin
                |UnusedG11_Pin|UnusedG6_Pin|UnusedG1_Pin|UnusedG2_Pin
                |UnusedG7_Pin|UnusedG0_Pin|UnusedG8_Pin|UnusedG15_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pins : UnusedF2_Pin UnusedF1_Pin UnusedF0_Pin UnusedF3_Pin
                UnusedF4_Pin UnusedF5_Pin UnusedF12_Pin UnusedF14_Pin
                UnusedF15_Pin UnusedF11_Pin UnusedF13_Pin */
    GPIO_InitStruct.Pin = UnusedF2_Pin|UnusedF1_Pin|UnusedF0_Pin|UnusedF3_Pin
                |UnusedF4_Pin|UnusedF5_Pin|UnusedF12_Pin|UnusedF14_Pin
                |UnusedF15_Pin|UnusedF11_Pin|UnusedF13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

      /*Configure GPIO pins : UnusedH0_Pin UnusedH1_Pin */
    GPIO_InitStruct.Pin = UnusedH0_Pin|UnusedH1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
#endif

	  /*Configure GPIO pins : PC4 PC5 PC6 PC7
	                           PC8 */
	  GPIO_InitStruct.Pin = BSP_JOYSTICK_UP_PIN|BSP_JOYSTICK_DOWN_PIN|BSP_JOYSTICK_RIGHT_PIN|BSP_JOYSTICK_LEFT_PIN
	                          |BSP_JOYSTICK_ENTER_PIN;
	  GPIO_InitStruct.Mode = BSP_JOYSTICK_GPIO_MODE(hardware_version);
	  GPIO_InitStruct.Pull = BSP_JOYSTICK_GPIO_PULL(hardware_version);
	  HAL_GPIO_Init(BSP_JOYSTICK_GPIO_PORT, &GPIO_InitStruct);
	  joystick_pressed = BSP_JOYSTICK_READ_STATE(hardware_version);

	  /*Configure GPIO pins : PB4 PB5 */
	  GPIO_InitStruct.Pin = BSP_PN532_RST_PIN|BSP_PN532_INT_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(BSP_PN532_RST_PORT, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


    // USB
    /* Configure DM DP Pins */
    GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure VBUS Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    /* Enable USB FS Clock */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    // Enable VBUS sense (B device) via pin PA9
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
	}

uint32_t Key_Pressed = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	BSP_TIM2_PWM_Start(100);
	if (GPIO_Pin == BSP_JOYSTICK_UP_PIN)
	{
		Key_Pressed = 1;//up
	}
	else if (GPIO_Pin == BSP_JOYSTICK_DOWN_PIN)
	{
		Key_Pressed = 2;//down
	}
	else if (GPIO_Pin == BSP_JOYSTICK_RIGHT_PIN)
	{
		Key_Pressed = 4;//right
	}
	else if (GPIO_Pin == BSP_JOYSTICK_LEFT_PIN)
	{
		Key_Pressed = 3;//left
	}
	else if (GPIO_Pin == BSP_JOYSTICK_ENTER_PIN)
	{
		Key_Pressed = 5;//ok
	}
}

uint32_t BSP_GetKeyPressed(void)
{
  return Key_Pressed;
}

void BSP_ClearKeyPressed(void)
{
  Key_Pressed = 0;;
}

uint32_t disp_error_check(uint32_t ret){
  if(ret != HAL_OK){
    while (1)
    {
      BSP_TIM2_PWM_Start(50);
      BSP_DelayMs(100);
      BSP_TIM2_PWM_Start(50);
      BSP_DelayMs(2800);
    }
  }
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
      uint32_t bank = 0;

      if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
      {
        /* No Bank swap */
        if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
        {
          bank = FLASH_BANK_1;
        }
        else
        {
          bank = FLASH_BANK_2;
        }
      }
      else
      {
        /* Bank swap */
        if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
        {
          bank = FLASH_BANK_2;
        }
        else
        {
          bank = FLASH_BANK_1;
        }
      }

      return bank;
}
/**
  * Read word from Non-Volatile memory
  */
void BSP_NonVolatileRead(uint32_t addr, uint32_t *dstAddr, uint32_t numOfbytes)
{
    uint32_t* sourceAddr = (uint32_t *)addr;
    uint8_t offset = numOfbytes%4;
    uint32_t numOfWords = numOfbytes/4;

    for (int i=0; i < numOfWords; i++) {
        *dstAddr = ((uint32_t)(*sourceAddr));
        dstAddr++;
        sourceAddr++;
    }

    if(offset){
        uint32_t word = ((uint32_t)(*sourceAddr));
        *dstAddr = *dstAddr & (0xFFFFFFFF<<((offset)*8));
        word = word & (0xFFFFFFFF>>((4-offset)*8));
        *dstAddr = *dstAddr | word;
    }
}

BSP_Status_t BSP_FlashSectorErase(uint32_t page_address, uint32_t noOfpages)
{
    FLASH_EraseInitTypeDef ptEraseTypeInfo;
    uint32_t sectorError;
    BSP_Status_t status = BSP_OK;
    uint32_t noOfpages_erased=0;
    ptEraseTypeInfo.Banks = GetBank(page_address);
    ptEraseTypeInfo.TypeErase = FLASH_TYPEERASE_PAGES;
    ptEraseTypeInfo.Page = GetPage(page_address);

    if((ptEraseTypeInfo.Page + noOfpages) > FLASH_BANK_SIZE/FLASH_PAGE_SIZE){
        noOfpages_erased = ptEraseTypeInfo.NbPages = (FLASH_BANK_SIZE/FLASH_PAGE_SIZE)-ptEraseTypeInfo.Page;
    }
    else{
        noOfpages_erased = ptEraseTypeInfo.NbPages = noOfpages;
    }
    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&ptEraseTypeInfo, &sectorError) != HAL_OK)
        status = BSP_FLASH_ERR;

    if((noOfpages_erased<noOfpages)&&(ptEraseTypeInfo.Banks==1)){
        ptEraseTypeInfo.Banks = 2;
        ptEraseTypeInfo.TypeErase = FLASH_TYPEERASE_PAGES;
        ptEraseTypeInfo.Page = FLASH_BASE+FLASH_BANK_SIZE;//GetPage(page_address);
        ptEraseTypeInfo.NbPages = noOfpages-noOfpages_erased;
        if (HAL_FLASHEx_Erase(&ptEraseTypeInfo, &sectorError) != HAL_OK)
            status = BSP_FLASH_ERR;
    }
    HAL_FLASH_Lock();

    return status;
}

BSP_Status_t BSP_FlashSectorWrite(__IO uint32_t *dstAddr, const uint32_t* srcAddr, uint32_t noOfbytes)
{
	int i;
    BSP_Status_t status = BSP_OK;
    uint64_t *p64Data = (uint64_t*)srcAddr;
	uint32_t noOfWords = (noOfbytes>>3)<<1;

    if((*dstAddr < FLASH_BASE) || ((*dstAddr + noOfbytes) > FLASH_END))
        return status;

    HAL_FLASH_Unlock();
    for (i = 0; i < noOfWords; i+=2) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)(dstAddr), *p64Data) == HAL_OK) {
            if (*p64Data != *(uint64_t*)dstAddr) {
                status = BSP_FLASH_ERR;
                break;
            }
            dstAddr+=2;
            p64Data++;
        }
        else
            status = BSP_FLASH_ERR;
    }

	if(noOfbytes%8 != 0){
		uint8_t	padding = 8-(noOfbytes%8);
		uint64_t u64Data=0;

		if(padding < 4){
			u64Data = srcAddr[1] | (0xFFFFFFFF<<((4-padding)*8));
			u64Data = u64Data<<32;
			u64Data |= srcAddr[0];
		}
		else{
			u64Data = (uint64_t)0xFFFFFFFF << 32;
			u64Data |= srcAddr[0] | (0xFFFFFFFF<<((8-padding)*8));
		}
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)(dstAddr), u64Data) == HAL_OK) {
			if (u64Data != *(uint64_t*)dstAddr) {
				status = 1;
			}
		}
	}

    HAL_FLASH_Lock();
    return status;
}

void BSP_reset(void)
{
	HAL_NVIC_SystemReset();
}

void BSP_DelayMs(uint32_t delayValue)
{
	HAL_Delay(delayValue);
}

void BSP_Buzzer_Timer(){
	if(buzzer_counter!=0){
		buzzer_counter--;
		if(buzzer_counter==0){
			BSP_TIM2_PWM_Stop();
		}
	}
}

void BSP_RNG_Init(void)
{
/********************  Bits definition for RNG_CR Clock Error detection register  *******************/
#define RNG_CR_CED_Pos    (5U)
#define RNG_CR_CED_Msk    (0x1UL << RNG_CR_RNGEN_Pos)                        /*!< 0x00000004 */
#define RNG_CR_CED        RNG_CR_RNGEN_Msk

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */

  BSP_RNG_End();    //disable RNG if already started

  __HAL_RCC_RNG_CLK_ENABLE();                   //Enable RNG clock
  CLEAR_BIT(RNG->SR, RNG_IT_CEI|RNG_IT_SEI);    //Clear RNG Interrupt status bits
  SET_BIT(RNG->CR, RNG_CR_CED);                 //Enable Clock Error detection
  SET_BIT(RNG->CR, RNG_CR_RNGEN);               //Enable RNG

  //Check if any error has occured in RNG Clock or random generation
  if(HAL_IS_BIT_SET(RNG->SR, RNG_IT_SEI) || HAL_IS_BIT_SET(RNG->SR, RNG_IT_CEI)){
    CLEAR_BIT(RNG->SR, RNG_IT_CEI|RNG_IT_SEI);
    Error_Handler();
  }

  uint32_t tickstart = HAL_GetTick();
  while (HAL_IS_BIT_CLR(RNG->SR, RNG_SR_DRDY)     //Is RNG data ready?
        || HAL_IS_BIT_SET(RNG->SR, RNG_SR_SECS)   //Is Seed error detected?
        || HAL_IS_BIT_SET(RNG->SR, RNG_SR_CECS))  //Is Clock error ongoing?
  {
    if ((HAL_GetTick() - tickstart) > 2)          //Taking too long?
    {
      Error_Handler();
    }
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

BSP_Status_t BSP_RNG_Generate(uint32_t *random32bit) {
  //If RNG interrupt had occured, restart RNG
  if(HAL_IS_BIT_SET(RNG->SR, RNG_IT_SEI) || HAL_IS_BIT_SET(RNG->SR, RNG_IT_CEI)){
    BSP_RNG_Init();
  }

  uint32_t tickstart = HAL_GetTick();
  //Wait till RNG is ready and all issues are resolved
  while (HAL_IS_BIT_CLR(RNG->SR, RNG_SR_DRDY) || HAL_IS_BIT_SET(RNG->SR, RNG_SR_SECS) || HAL_IS_BIT_SET(RNG->SR, RNG_SR_CECS))
  {
    if ((HAL_GetTick() - tickstart) > 2)
    {
      return BSP_RNG_HW_ERROR;
    }
  }

  //Check if random data is zero
  if(RNG->DR != 0) {
    *random32bit = RNG->DR;
    return BSP_OK;
  }
  return BSP_RNG_ZERO_ERROR;
}

void BSP_RNG_End(void){
  __HAL_RCC_RNG_CLK_DISABLE();      //disable RNG clock
  CLEAR_BIT(RNG->CR, RNG_CR_RNGEN); //disable RNG
}


/* user callback function for systick Handler */
void HAL_SYSTICK_Callback(void)
{
	BSP_App_Timer_Run();
	BSP_Buzzer_Timer();
	if(bsp_tick != 0){
		bsp_tick--;
	}
}

BSP_App_Timer_Callback_t appTimerList[BSP_APP_TIMER_MAX];
static uint8_t appTimerRegisterCount = 0;
static uint32_t systickCounter = 0;

void BSP_App_Timer_Init()
{
  for (uint8_t count = 0; count < BSP_APP_TIMER_MAX; count++)
  {
	  appTimerList[count].appTimerCb   = NULL;
	  appTimerList[count].timeOutValue = 0;
  }
}

uint32_t BSP_App_Timer_Create(uint8_t TimerId, void appTimerHandler(void))
{
	if ((BSP_APP_TIMER_MAX > appTimerRegisterCount) && (NULL != appTimerHandler))
	{
		appTimerList[TimerId].appTimerCb   = appTimerHandler;
		appTimerList[TimerId].timeOutValue = 0;
		appTimerRegisterCount++;
		return 0;
	}
	else
	{
		return 1;
	}

}

void BSP_App_Timer_Start(uint8_t TimerId, uint32_t timeOutValue)
{
	if ((appTimerRegisterCount > TimerId) && (NULL != appTimerList[TimerId].appTimerCb))
	{
		appTimerList[TimerId].timeOutValue = timeOutValue;
	}
}

void BSP_App_Timer_Stop(uint8_t TimerId)
{
	if ((appTimerRegisterCount > TimerId) && (NULL != appTimerList[TimerId].appTimerCb))
	{
		appTimerList[TimerId].timeOutValue = 0;
		appTimerList[TimerId].appTimerCb   = NULL;
	}
}

void BSP_App_Timer_Run(void)
{
	systickCounter++;

	for (uint8_t count = 0; count < appTimerRegisterCount; count++)
	{
//       if (appTimerList[count].timeOutValue >= systickCounter)
		if(appTimerList[count].timeOutValue==0){
			continue;
		}
		if((systickCounter+1)%(appTimerList[count].timeOutValue)==0)
       {
    	   if(NULL != appTimerList[count].appTimerCb)
    	   {
    	     appTimerList[count].appTimerCb();
    	   }
       }
	}
}

void BSP_sysClkDisable(void)
{
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

uint32_t read_hw_gpio_config()
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BSP_JOYSTICK_LEFT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BSP_JOYSTICK_GPIO_PORT, &GPIO_InitStruct);

    if (HAL_GPIO_ReadPin(BSP_JOYSTICK_GPIO_PORT, BSP_JOYSTICK_LEFT_PIN) == GPIO_PIN_SET) {
        return DEVICE_HARDWARE_STM32_2;
    } else {
        return DEVICE_HARDWARE_STM32_3;
    }
}
