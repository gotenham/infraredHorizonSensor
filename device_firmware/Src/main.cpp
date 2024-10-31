/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "retarget.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "mtxCV.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _DEBUG_MAIN //DEBUG FLAG, uncomment to enable module debugging
//#define _DEBUG_SENSOR //DEBUG FLAG, uncomment to enable module debugging related to sensor init

#define MLX_EEPROM_SIZE 832
#define MLX_FRAME_DATA_SIZE 834
#define MLX_THERMAL_ARR_SIZE 768

#define IMAGE_WIDTH 32
#define IMAGE_HEIGHT 24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
const uint8_t MLX90640_address = 0x33; // 0x33 default 7-bit unshifted address of the MLX90640

const uint8_t gaussian_N = 5; // SET Gaussian NxN kernel size, must be odd
const RowVector<float, (int)gaussian_N> gaussianRowVector = (Eigen::RowVector<float, (int)gaussian_N>() <<
		0.070766f, 0.244460f, 0.369546f, 0.244460f, 0.070766f).finished();
const Vector<float, (int)gaussian_N> gaussianColVector = (Eigen::Vector<float, (int)gaussian_N>() <<
		0.070766f, 0.244460f, 0.369546f, 0.244460f, 0.070766f).finished(); // TBD::wasting memory given transpose of row vect

// add your relevent intrinsicK_inv params below from sensor calibration
const Matrix3f intrinsicK_inv = (Eigen::Matrix3f() <<
		1.f,	0, 		-16.f,
		0,		-1.f, 	12.f,
		0,		0, 		1.f).finished();

const float cannyGradientThreshHigh = 0.7f; // ratio of largest gradient in suppressed image; edge must have gradient >= this to qualify as strong edge
const float cannyGradientThreshLow = 0.3f; // ratio of {cannyGradientThreshHigh OR largest gradient in suppressed image}, edge must have gradient >= this to qualify a weak edge
const size_t minSequenceLength = 10; //minimum sequence of edge points required to qualify being added to edgeSequenceListReturn
const uint8_t strongEdgeValue = 255; // candidate edges are assigned this value in the canny returned edgeMask when >= calculated dynThreshHigh
const uint8_t weakEdgeValue = 160; // candidate edges are assigned this value in the canny returned edgeMask when >= calculated dynThreshLow

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

// FUNCTION SIGNATURES IN MAIN
bool initialise_MLX90640(const uint8_t& I2C_address, paramsMLX90640& _paramsTarget); // combines initialisation steps and avoids storing eeData longer than required
int getFrame_MLX90640(const uint8_t& I2C_address, float *_imgArr, float& _ta, paramsMLX90640& _params);
void sendGraphCoords(const vector<EdgePoint*>& edgeContour); // Helper function to send edge sequence to serial, to display as graph on host machine
void sendSerialData(const uint8_t* data, uint16_t size);
template<typename MatrixType>
void sendMatrixSerial(const MatrixType& matrix, const char* uniqueTitle);
unsigned int getSysTickSeed(void); //generates "random" number as ms since system was rebooted (used for RANSAC input seed)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_LPUART1_UART_Init();
	MX_SPI1_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	//Retarget various serial print commands, gives printf functionality to COM port serial for debugging
	RetargetInit(&hlpuart1);

	MLX90640_I2CGeneralReset(); //send general reset command to reset connected I2C devices
	HAL_Delay(200); // allow startup settling

	// Configure MLX90640_I2C_Driver.h with reference to IC2 interface it should use; ie hi2c1
	MLX90640_I2CInitRef(&hi2c1);

	#ifdef _DEBUG_MAIN
		printf("\n\r\n\r-------------------------------- RESET --------------------------------\n\r");
		printf(">> Serial Debug Started @%lubaud <<\n\r", hlpuart1.Init.BaudRate); // default: 115200
	#endif

	/* Key sensor related data storage */
	paramsMLX90640 mlx90640Params; //struct stores sensor configuration and mapping parameters recieved from device
	float mlx90640ImageArr[MLX_THERMAL_ARR_SIZE]; // MAIN storage for returned 1-D thermal array from sensor

	/* Perform initialisation for thermal sensor */
	if(!initialise_MLX90640(MLX90640_address, mlx90640Params)){
		#ifdef _DEBUG_MAIN
			printf("\n\rFAILED TO INIT MLX90640, freezing..");
		#endif
		//Set yellow LED ON to indicate startup error
		HAL_GPIO_WritePin(GPIOB, GPIO_LED2_YEL_OUTPUT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_LED1_GRN_OUTPUT_Pin, GPIO_PIN_RESET);
		while (1); //freeze program execution // TBD for debugging
	} else {
		#ifdef _DEBUG_MAIN
			printf("OK: MLX90640 Sucessfully Initialised!\n\r");
		#endif
		//Set green LED ON to indicate startup OK
		HAL_GPIO_WritePin(GPIOB, GPIO_LED2_YEL_OUTPUT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_LED1_GRN_OUTPUT_Pin, GPIO_PIN_SET);
	}

	/* Setup main loop local variables */
	Matrix<float, IMAGE_HEIGHT, IMAGE_WIDTH> mtxThermal; // MAIN THERMAL IMAGE FRAME
	vector<EdgePoint> edgeCandidates; // init var to store coordinates of identified weak and strong edges in cannyEdgeDetection returned image edge mask
	//Matrix<int, IMAGE_HEIGHT, IMAGE_WIDTH> edgeMask; // MAIN EDGE MASK FRAME
	//MatrixXf mtxThermalNorm(IMAGE_HEIGHT, IMAGE_WIDTH); // MAIN NORMAL OF THERMAL IMAGE FRAME
	//Contours allContours; // init storage for Contours (eg a vector of edge sequences)
	vector<EdgePoint*> edgeContour;
	Matrix<float, 6, 1> best_conic; // Run RANSAC to fit a conic eq parameters into best_conic[A,B,C,D,E,F]
	float ta = -999.0; //used for temp offsets on MLX sensor (not used as currently reading direct pixel values instead of temperature calc)


	#ifdef _DEBUG_MAIN
		//printParamsMLX90640(&mlx90640Params);
	#endif

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//Get new thermal image from sensor as 1D float array in mlx90640ImageArr
		getFrame_MLX90640(MLX90640_address, mlx90640ImageArr, ta, mlx90640Params);

		//Reset thermal image matrix
		mtxThermal.setZero();
		//Convert thermal image 1D array to 2D image matrix
		MtxCV::getByteMatrixFromFloatArray(mlx90640ImageArr, MLX_THERMAL_ARR_SIZE, mtxThermal); //, mtxThermalNorm

		// Send virgin thermal image to serial
		//sendMatrixSerial(mtxThermal, "mtxThermalVirgin");

		//perform convolution with seperable row anc col vectors, ie gaussian blur
		MtxCV::separableConvolution(mtxThermal, gaussianRowVector, gaussianColVector, true);

		// Perform canny edge detection with non-max suppression
		//MtxCV::cannyEdgeDetection(mtxThermal, edgeCandidates, cannyGradientThreshLow, cannyGradientThreshHigh, &edgeMask, weakEdgeValue, strongEdgeValue);
		//MtxCV::edgeTrackingByHysteresis(edgeMask, edgeCandidates, allContours, minSequenceLength, weakEdgeValue, strongEdgeValue);

		// BELOW IS TEMP helper fuction to reduce memory usage in main, it simply combinees the functionality of MtxCV::cannyEdgeDetection() and MtxCV::edgeTrackingByHysteresis()
		MtxCV::cannyTrackingHelperFn(mtxThermal, edgeCandidates, edgeContour, cannyGradientThreshLow, cannyGradientThreshHigh, weakEdgeValue, strongEdgeValue, minSequenceLength);

		//Generate subpixel coordinate adjustements for current edge sequence
		MtxCV::getZernikeSubpixelAdjustment(mtxThermal, edgeContour); // >> edgeContour.adjustment is the new coordinates after correction

		//Map sub-pixel coords to plane in world object space
		MtxCV::mapEdgeSequence2WorldFrame(edgeContour, intrinsicK_inv, 10.f); // >> edgeContour.adjustment and edgeContour.coord are the new coordinates after transform to object space

		//FLIP IMAGE HORIZONTAL + VERTICAL, inverts image to account for optical flip
		//mtxThermal.colwise().reverseInPlace();
		//mtxThermal.rowwise().reverseInPlace();

		//send thermal image via serial as renderable image
		sendMatrixSerial(mtxThermal, "mtxThermal"); // FYI: Flip image horizontally ( mtxThermal.colwise().reverse() )
		//sendMatrixSerial(edgeMask, "edgeMask");

		//Send the pixel edge and zernike sub-pixel adjusted coordinates to serial
		sendGraphCoords(edgeContour);

		//reset best conic params
		best_conic.setZero();
		unsigned int tickSeed = getSysTickSeed();
		// ransacConicFitting KEY::::{inputCoords, outputConicParams, eccentricity_minThreshold, numTestSamples, inlier_threshold(conic eq remainder must be less than this))}
		FnState fnFitConicModel = MtxCV::ransacConicFitting(edgeContour, best_conic, 1.f, 3.f, 100, 0.1f, tickSeed); //

		if(fnFitConicModel == FnState::OK){
			#ifdef _DEBUG_MAIN
				//printf("FoundConicModel\n\r");
			#endif
			MtxCV::classifyConic(best_conic);
		}

		HAL_GPIO_TogglePin(GPIOB, GPIO_LED1_GRN_OUTPUT_Pin); // TOGGLE onboard LED each loop

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

	/** Configure LSE Drive Capability
	*/
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00100618;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
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

	/** I2C Fast mode Plus enable
	*/
	HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00909BEB;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
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
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_LED2_YEL_OUTPUT_Pin|GPIO_LED1_GRN_OUTPUT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_EXT_PWM_INPUT_PB12_Pin */
	GPIO_InitStruct.Pin = GPIO_EXT_PWM_INPUT_PB12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_EXT_PWM_INPUT_PB12_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_EXT_INTERRUPT_PB13_Pin */
	GPIO_InitStruct.Pin = GPIO_EXT_INTERRUPT_PB13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIO_EXT_INTERRUPT_PB13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_SW2_INPUT_Pin */
	GPIO_InitStruct.Pin = GPIO_SW2_INPUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIO_SW2_INPUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIO_LED2_YEL_OUTPUT_Pin GPIO_LED1_GRN_OUTPUT_Pin */
	GPIO_InitStruct.Pin = GPIO_LED2_YEL_OUTPUT_Pin|GPIO_LED1_GRN_OUTPUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* COMMENTED AS FUNCTION IS RELATED TO TESTING

*/

bool initialise_MLX90640(const uint8_t& I2C_address, paramsMLX90640& _paramsTarget) {
/*	//Typedef different sensor operation modes
	typedef enum MLX90640_Mode {
		MLX90640_INTERLEAVED, //image read in interleaved lines (odd rows, even rows etc)
		MLX90640_CHESS // [default] image read in chess pattern (odd index, even index etc)
	};

	//Typedef different sensor resolutions for pixel ADC reading
	typedef enum MLX90640_Resolution {
		MLX90640_ADC_16BIT,
		MLX90640_ADC_17BIT,
		MLX90640_ADC_18BIT, // [default]
		MLX90640_ADC_19BIT
	};

	//Typedef different sensor supported refresh rates
	typedef enum MLX90640_RefreshRate {
		MLX90640_0_5HZ,
		MLX90640_1HZ,
		MLX90640_2HZ, // [default]
		MLX90640_4HZ,
		MLX90640_8HZ,
		MLX90640_16HZ,
		MLX90640_32HZ,
		MLX90640_64HZ
	};
*/

	//Check if MLX90640 is available at I2C address
	if (!MLX90640_isConnected(I2C_address)) {
		#ifdef _DEBUG_SENSOR
			printf("[Err] MLX90640 not detected at 0x%x I2C address.\n\r", I2C_address);
		#endif
		return false;
	} else {
		#ifdef _DEBUG_SENSOR
			printf("OK: MLX90640 I2C online!\n\r");
		#endif
	}

	// Dump parameters from MLX90640 EEPROM
	uint16_t eeMLX90640[MLX_EEPROM_SIZE]; // EEPROM dump from thermal camera parameters, used in thermal array API calcs
	int eeDumpState = MLX90640_DumpEE(I2C_address, eeMLX90640);
	if (eeDumpState != 0) {
		#ifdef _DEBUG_SENSOR
			// -1 indicates NACK occurred during the communication and eeMLX90640 is invalid EEPROM data
			printf("[Err] Failed to dump EEPROM. Return code [%d].\n\r", eeDumpState);
		#endif
		return false;
	} else {
		#ifdef _DEBUG_SENSOR
			printf("OK: MLX90640 EEROM dumped successfully.\n\r");
		#endif
	}

	int getParamsState = MLX90640_ExtractParameters(eeMLX90640, &_paramsTarget);
	if (getParamsState != 0) {
		//eeMLX90640 is not valid, the parameters extraction is aborted
		// -3: MLX90640_BROKEN_PIXELS_NUM_ERROR, -4: MLX90640_OUTLIER_PIXELS_NUM_ERROR, -5: MLX90640_BAD_PIXELS_NUM_ERROR, -6: CheckAdjacentPixels failed check
		#ifdef _DEBUG_SENSOR
			printf("[Err] Failed to extract parameters. Return code [%d].\n\r", getParamsState);
		#endif
		return false;
	} else {
		#ifdef _DEBUG_SENSOR
			printf("OK: MLX90640 parameters extracted!\n\r");
		#endif
	}

	//Check sensor current mode and re-configure if not as expected
	int deviceSetting = MLX90640_GetCurMode(I2C_address); //return 0 is interleaved mode, 1 is chess pattern mode
	if(deviceSetting != (int)MLX90640_Mode::MLX90640_CHESS) {
		int returnCode = MLX90640_SetChessMode(I2C_address);
		if(returnCode != 0) {
			#ifdef _DEBUG_SENSOR
				printf("[Err] Failed to set mode to [%u]. Return code [%d].\n\r", (int)MLX90640_Mode::MLX90640_CHESS, returnCode);
			#endif
//			return false;
		} else {
			#ifdef _DEBUG_SENSOR
				printf("OK: Re-configured device to reading mode: MLX90640_CHESS\n\r");
			#endif
		}
	} else {
		#ifdef _DEBUG_SENSOR
			printf("OK: Device is already in reading mode: MLX90640_CHESS\n\r");
		#endif
	}


	//Check sensor current pixel ADC resolution and re-configure if not as expected
	deviceSetting = MLX90640_GetCurResolution(I2C_address); // see MLX90640_Resolution typedef
	if(deviceSetting != (int)MLX90640_Resolution::MLX90640_ADC_18BIT) {
		int returnCode = MLX90640_SetResolution(I2C_address, (uint8_t)MLX90640_Resolution::MLX90640_ADC_18BIT);
		if(returnCode != 0) {
			#ifdef _DEBUG_SENSOR
				printf("[Err] Failed to set ADC resolution to [%u]. Return code [%d].\n\r", (int)MLX90640_Resolution::MLX90640_ADC_18BIT, returnCode);
			#endif
//			return false;
		} else {
			#ifdef _DEBUG_SENSOR
				printf("OK: Re-configured device to ADC resolution: MLX90640_ADC_18BIT\n\r");
			#endif
		}
	} else {
		#ifdef _DEBUG_SENSOR
			printf("OK: Device is already set to ADC resolution: MLX90640_ADC_18BIT\n\r");
		#endif
	}

	//Check sensor current pixel ADC resolution and re-configure if not as expected
	deviceSetting = MLX90640_GetRefreshRate(I2C_address); // see MLX90640_Resolution typedef
	if(deviceSetting != (int)MLX90640_RefreshRate::MLX90640_8HZ) {
		int returnCode = MLX90640_SetRefreshRate(I2C_address, (uint8_t)MLX90640_RefreshRate::MLX90640_8HZ);
		if(returnCode != 0) {
			#ifdef _DEBUG_SENSOR
				printf("[Err] Failed to set refresh rate to [%u]. Return code [%d].\n\r", (int)MLX90640_RefreshRate::MLX90640_8HZ, returnCode);
			#endif
//			return false;
		} else {
			#ifdef _DEBUG_SENSOR
				printf("OK: Re-configured device to refresh rate: MLX90640_8HZ\n\r");
			#endif
		}
	} else {
		#ifdef _DEBUG_SENSOR
			printf("OK: Device is already set to refresh rate: MLX90640_8HZ\n\r");
		#endif
	}


	return true; //(eeDumpState == 0 && getParamsState == 0);

}

/*Helper function to load thermal image array from subpage 1 and 2.*/
int getFrame_MLX90640(const uint8_t& I2C_address, float *_imgArr, float& _ta, paramsMLX90640& _params) { //, MLXFrameProgress& _getImgState
	// New Subpage is avaiable every 1/FPS, eg MLX90640_8HZ is new subPage every 125ms, meaning a full page is ready every 250ms
	// as per amanufacturer: time necessary before the device can reach the specified accuracy ~4 min.
	int status = -1; //track results of sensor related function executions
	uint16_t mlx90640Frame[MLX_FRAME_DATA_SIZE]; //temp storage for 1D image frame from sensor

	// for both subframes of complete image
	for (uint8_t page = 0; page < 2; page++) {
		//extract subframe (#1, then #2) to mlx90640Frame
		status = MLX90640_GetFrameData(I2C_address, mlx90640Frame);
		//Verify if extraction ok
		if (status < 0) {
			#ifdef _DEBUG_SENSOR
				printf("[Err]@getFrame_MLX90640 return code: %d\n\r", status);
			#endif
			return status;
		}
		//Construct image from both sub frame #1 and #2
		MLX90640_GetImage(mlx90640Frame, &_params, _imgArr);
		HAL_Delay(200); // TBD move this functionality to I2C interrupt: Delay for debug testing, give time for next subpage to be prepared
	}
	// Perform broken pixel and outlier adjustement to image
	int mode = MLX90640_GetCurMode(I2C_address);
	MLX90640_BadPixelsCorrection((&_params)->brokenPixels, _imgArr, mode, &_params);
	MLX90640_BadPixelsCorrection((&_params)->outlierPixels, _imgArr, mode, &_params);
	return 0; //success
}


// Helper function to send edge sequence to serial, to display as graph on host machine
void sendGraphCoords(const vector<EdgePoint*>& edgeContour) {
	// Get the number of coords in the current edge sequence
	//size_t numCoords = edgeContour.size();
	// Init matrix to send coords via serial as [x, y, x_adjusted, y_adjusted],[...
	MatrixXf coordData(edgeContour.size(), 4);
	// Loop over coord and coord adjustements in current edge sequence and fill the matrix
	for (size_t edgeCoordIndex = 0; edgeCoordIndex < edgeContour.size(); ++edgeCoordIndex) {
		const EdgePoint* edge = edgeContour[edgeCoordIndex];
		// Fill the matrix with x, y, x_adjusted, and y_adjusted
		coordData(edgeCoordIndex, 0) = static_cast<float>(edge->coord.x());      // x_coord
		coordData(edgeCoordIndex, 1) = static_cast<float>(edge->coord.y());     // y_coord
		coordData(edgeCoordIndex, 2) = edge->adjustment.x();                     // x_adjusted
		coordData(edgeCoordIndex, 3) = edge->adjustment.y();                    // y_adjusted
	}
	sendMatrixSerial(coordData, "graph"); //send edge coords and adjustement via serial as graph data
}

// Function to send data over serial using HAL libraries
void sendSerialData(const uint8_t* data, uint16_t size) {
	const size_t timeout_ms = 3000;
    // Replace USART2 with the appropriate USART instance you're using
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)data, size, timeout_ms);
	//printf("%s", data);
}

// Function to send image matrix via serial in CSV format
template<typename MatrixType>
void sendMatrixSerial(const MatrixType& matrix, const char* uniqueTitle) {
	const int defaultPrecision = 2;
	const Index mtxRows = matrix.rows();
    const Index mtxCols = matrix.cols();
    bool isFloatingPoint = std::is_floating_point<typename MatrixType::Scalar>::value;
    int decimalPrecision = (isFloatingPoint ? defaultPrecision : 0);

    // Prepare tagData / buffer string
    char sendBuffer[mtxCols * 10]; //
    size_t buffSize = sizeof(sendBuffer);
    int charWritten = snprintf(sendBuffer, buffSize, "~txBEG~{ID:%s,X:%d,Y:%d,dt:%s}\n", uniqueTitle, mtxCols, mtxRows, (isFloatingPoint ? "f" : "i"));
    // if the provided title overflows the buffer, instead send the unique matrix memory address in its place
    if (charWritten < 0 || charWritten >= static_cast<int>(buffSize)) {
    	const void* matrixAddress = static_cast<const void*>(&matrix);
    	snprintf(sendBuffer, buffSize, "~txBEG~{ID:%p,X:%d,Y:%d,dt:%s}\n", matrixAddress, mtxCols, mtxRows, (isFloatingPoint ? "f" : "i"));
    }
    // Send serial start designator and matrix tagData
    sendSerialData(reinterpret_cast<const uint8_t*>(sendBuffer), strlen(sendBuffer));

    size_t offset = 0;

    // Iterate over each row of the image matrix
    for (Index j = 0; j < mtxRows; ++j) {
		// Iterate over each column of the image matrix
		for (Index i = 0; i < mtxCols; ++i) {
			// Convert each pixel value to string and copy to the buffer
			//offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%.*f,", decimalPrecision, static_cast<float>(matrix(j, i)));
			// Convert each pixel value to string and copy to the buffer
			int charWritten = snprintf(sendBuffer + offset, buffSize - offset, "%.*f,", decimalPrecision, static_cast<float>(matrix(j, i)));
			if (charWritten < 0 || charWritten >= static_cast<int>(buffSize - offset)) {
				// [Err]: Buffer overflow while formatting row
			    // Send serial ERROR designator
			    snprintf(sendBuffer, buffSize, "~txERR~\n\r");
			    sendSerialData(reinterpret_cast<const uint8_t*>(sendBuffer), strlen(sendBuffer));
			    return;
			} else {
				offset += charWritten;
			}
		}
        // Add newline character at the end of each row if there's space in the buffer
        if (offset + 1 < buffSize) {
        	// Add newline character at the end of each row
        	sendBuffer[offset - 1] = ';'; // Replace the last comma with semicolon
        	sendBuffer[offset++] = '\n'; // Add newline character
        } else {
            // Buffer overflow, send error designator and return
            snprintf(sendBuffer + offset, buffSize - offset, "~txERR~\n\r");
            sendSerialData(reinterpret_cast<const uint8_t*>(sendBuffer), strlen(sendBuffer));
            return;
        }
        //sendBuffer[offset - 1] = ';'; // Replace the last comma with semicolon
		// Send the buffer over serial port
		sendSerialData(reinterpret_cast<const uint8_t*>(sendBuffer), offset);
		offset = 0; // Reset offset for the next row
    }

    // Send serial end designator
    snprintf(sendBuffer, buffSize, "~txEND~\n\r");
    sendSerialData(reinterpret_cast<const uint8_t*>(sendBuffer), strlen(sendBuffer));
}

// pseudo "random" number from current ms since program start
unsigned int getSysTickSeed(void) {
    return HAL_GetTick(); // Get the current tick count as a seed
}

/* COMMENTED AS FUNCTIONS BELOW ARE RELATED TO DEBUG / TESTING
//Testing function to load matrix with incrementing numbers
void fillMatrixIncrementing(MatrixXi& matrix, const int startingSeed = 1, const int stepDistance = 1) {
    Eigen::Index rows = matrix.rows();
    Eigen::Index cols = matrix.cols();
    int value = startingSeed;
    for (Eigen::Index j = 0; j < rows; ++j) {
        for (Eigen::Index i = 0; i < cols; ++i) {
            value += stepDistance;
            matrix(j, i) = value;
        }
    }
}

// Function to print data in binary format
void printBinary(uint16_t& data, bool newLineAfter = true) {
    for (int i = 15; i >= 0; i--) {
        printf("%d", (data >> i) & 0x01);
    }
    if(newLineAfter){
    	 printf("\n\r");
    }
}

// AJ: additional function to aid in debugging MLX params struct contents
void printParamsMLX90640(paramsMLX90640 *params) {
	printf("@printParamsMLX90640:\n\r");
	// Print each field of the structure
	printf("kVdd: %d\n\r", params->kVdd);
	printf("vdd25: %d\n\r", params->vdd25);
	printf("KvPTAT: %.2f\n\r", params->KvPTAT);
	printf("KtPTAT: %.2f\n\r", params->KtPTAT);
	printf("vPTAT25: %d\n\r", params->vPTAT25);
	printf("alphaPTAT: %.2f\n\r", params->alphaPTAT);
	printf("gainEE: %d\n\r", params->gainEE);
	printf("tgc: %.2f\n\r", params->tgc);
	printf("cpKv: %.2f\n\r", params->cpKv);
	printf("cpKta: %.2f\n\r", params->cpKta);
	printf("resolutionEE: %d\n\r", params->resolutionEE);
	printf("calibrationModeEE: %d\n\r", params->calibrationModeEE);
	printf("KsTa: %.2f\n\r", params->KsTa);

	printf("ksTo: ");
	for (int i = 0; i < 5; i++) {
		printf("%.2f ", params->ksTo[i]);
	}
	printf("\n\r");

	printf("ct: ");
	for (int i = 0; i < 5; i++) {
		printf("%d ", params->ct[i]);
	}
	printf("\n\r");

	printf("alpha: ");
	for (int i = 0; i < MLX_THERMAL_ARR_SIZE; i++) {
		if(i % 32 == 0){
			printf("\n\r");
		}
		printf("%u ", params->alpha[i]);
	}
	printf("\n\r");

	printf("alphaScale: %u\n\r", params->alphaScale);

	printf("offset: ");
	for (int i = 0; i < MLX_THERMAL_ARR_SIZE; i++) {
		if(i % 32 == 0){
			printf("\n\r");
		}
		printf("%d ", params->offset[i]);
	}
	printf("\n\r");

	printf("kta: ");
	for (int i = 0; i < MLX_THERMAL_ARR_SIZE; i++) {
		if(i % 32 == 0){
			printf("\n\r");
		}
		printf("%d ", params->kta[i]);
	}
	printf("\n\r");

	printf("ktaScale: %u\n\r", params->ktaScale);

	printf("kv: ");
	for (int i = 0; i < MLX_THERMAL_ARR_SIZE; i++) {
		if(i % 32 == 0){
			printf("\n\r");
		}
		printf("%d ", params->kv[i]);
	}
	printf("\n\r");

	printf("kvScale: %u\n\r", params->kvScale);

	printf("cpAlpha: %.2f %.2f\n\r", params->cpAlpha[0], params->cpAlpha[1]);

	printf("cpOffset: %d %d\n\r", params->cpOffset[0], params->cpOffset[1]);

	printf("ilChessC: %.2f %.2f %.2f\n\r", params->ilChessC[0], params->ilChessC[1], params->ilChessC[2]);

	printf("brokenPixels: ");
	for (int i = 0; i < 5; i++) {
		printf("%u ", params->brokenPixels[i]);
	}
	printf("\n\r");

	printf("outlierPixels: ");
	for (int i = 0; i < 5; i++) {
		printf("%u ", params->outlierPixels[i]);
	}
	printf("\n\r");
}
*/

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
