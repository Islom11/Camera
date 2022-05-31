/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "camera.h"

#include "DataLinKLayer.h"
#include "CRC.h"
#include "stm32f407xx.h"
#include "bmp.h"
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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
UINT byteswritten;
bool write_flag = false;
UART_HandleTypeDef huart4;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void write(uint8_t val);
void read(uint8_t *UardRecivedData, uint16_t len);
TT_DataLinkLayerPoint<2048, 1000> ttDataLinkLayerPoint(write, read);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0x36, 0x29, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0xb0, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x29, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };
*/
/*//174*120
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0xb6, 0xf7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0xb0, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xf7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };

//*/
/*//144*176
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0x36, 0x29, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x29, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };

//*/
/*//174*120
static const uint8_t bmp_tou[54]={
		0x42, 0x4D, 0xD6, 0xF5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0xAE, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xA0, 0xF5, 0x00, 0x00, 0x74, 0x12, 0x00, 0x00, 0x74, 0x12, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };

//*/
//176*144
/*
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0xf6, 0x26, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0xae, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xc0, 0x26, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };
//*/

//171*144
/*
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0xc6, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0xab, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x0e, 0x00, 0x00, 0xc4, 0x0e, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };
//*/


//171*145
//*
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0x7a, 0x24, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0xab, 0x00, 0x00, 0x00, 0x91, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x44, 0x24, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };
//*/


//320*144
/*
static const uint8_t bmp_tou[54]={
		0x42, 0x4d, 0x36, 0x1c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00,
		0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x1c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
       };
//*/
uint8_t buffer[6];

uint8_t R,G,B;

uint16_t adres[IMAGE_SIZE]={0};


FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;


uint8_t white[690]={0xFF};

#define RGB565_TO_R(pixel)   (((pixel & 0x1F) << 3) | ((((pixel & 0x1F) << 3) & 0xE0) >> 5));
#define RGB565_TO_G(pixel)   (((pixel & 0x7E0) >> 3) | ((((pixel & 0x7E0) >> 3) & 0xC0) >> 6));
#define RGB565_TO_B(pixel)   (((pixel & 0xF800) >> 8) | ((((pixel & 0xF800) >> 8) & 0xE0) >> 5));

void createPhoto()
{
	uint8_t finish = 0x03;
	write_flag = false;
	//отправка шапки
	ttDataLinkLayerPoint.Send(bmp_tou,54);
	while(write_flag == false);
	write_flag = false;
//	ttDataLinkLayerPoint.Send(bmpData,74442);
	//ttDataLinkLayerPoint.send(adres,IMAGE_SIZE);
	//*
	uint16_t rgb_datr;
	uint32_t adress;
	for (int i = 0; i < IMAGE_SIZE; i++)
	{
//		adress = adres[i] ;
//		rgb_datr = adress & 0x0000FFFF;
//		buffer[2] = RGB565_TO_B(rgb_datr);
//		buffer[1] = RGB565_TO_G(rgb_datr);
//		buffer[0] = RGB565_TO_R(rgb_datr);



//		ttDataLinkLayerPoint.Send(buffer,3);
//		while(write_flag == false);
//		write_flag = false;
//		HAL_Delay(1);

//		rgb_datr = (adress>>16) & 0xFFFF;
		buffer[2] = RGB565_TO_B(adres[i]);
		buffer[1] = RGB565_TO_G(adres[i]);
		buffer[0] = RGB565_TO_R(adres[i]);


		ttDataLinkLayerPoint.Send(buffer,3);
		while(write_flag == false);
		write_flag = false;
//		HAL_Delay(1);

	}
	//*/
/*
	uint8_t *buf = (uint8_t *) adres;
	int length = IMAGE_SIZE * 4;
	for(int i=231; i < length; i+=2)
	{
		buffer[0] = buf[i];
		buffer[1] = buf[i];
		buffer[2] = buf[i];
		ttDataLinkLayerPoint.Send(buffer,3);
	}
	*/

	ttDataLinkLayerPoint.Send(white,690);

	for(int i = 0; i < 0xFFFF; i++);
	ttDataLinkLayerPoint.Send(&finish,1);








//	while(1);
//	fresult = f_mount(&fs, "/", 1);
//	f_getfree("", &fre_clust, &pfs);
//	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
//	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
//	fresult = f_open(&fil, "file.bmp", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//
//	fresult = f_write(&fil,bmp_tou,54,&byteswritten);
	//f_write(&fil,white,115,1);
	/*
	uint8_t *buf = (uint8_t *) adres;
	int length = IMAGE_SIZE * 4;
	for(int i=231; i < length; i+=2)
	{
		buffer[0] = buf[i];
		buffer[1] = buf[i];
		buffer[2] = buf[i];
		f_write(&fil,buffer,3,1);
	}
	//*/

	//*
//	uint16_t rgb_datr;
//	uint32_t adress;
//	for (int i = 0; i < IMAGE_SIZE; i++)
//	{
//		adress = adres[i];
//
//		rgb_datr =(uint16_t) adress;
//		/*
//		B = (uint8_t)(rgb_datr & 0x1f);
//		B = (uint8_t)(8.225*B);
//		G = (uint8_t)((rgb_datr>>5) & 0x3f);
//		G = (uint8_t)(4.047*G);
//		R = (uint8_t)((rgb_datr>>11) & 0x1f);
//		R = (uint8_t)(8.225*R);
//		buffer[2] = B;
//		buffer[1] = G;
//		buffer[0] = R;
//		*/
//		buffer[2] = RGB565_TO_B(rgb_datr);
//		buffer[1] = RGB565_TO_G(rgb_datr);
//		buffer[0] = RGB565_TO_R(rgb_datr);
//
//		f_write(&fil,buffer,3,&byteswritten);
//
//		rgb_datr = (adress>>16);
//		/*
//		B = (uint8_t)(rgb_datr & 0x1f);
//		B = (uint8_t)(8.225*B);
//		G = (uint8_t)((rgb_datr>>5) & 0x3f);
//		G = (uint8_t)(4.047*G);
//		R = (uint8_t)((rgb_datr>>11) & 0x1f);
//		R = (uint8_t)(8.225*R);
//		buffer[2] = B;
//		buffer[1] = G;
//		buffer[0] = R;
//		*/
//		buffer[2] = RGB565_TO_B(rgb_datr);
//		buffer[1] = RGB565_TO_G(rgb_datr);
//		buffer[0] = RGB565_TO_R(rgb_datr);
//
//		fresult = f_write(&fil,buffer,3,&byteswritten);
//
//
//	}
//*/
//	f_close(&fil);
}

void createPhotoFlesh()
{
		fresult = f_mount(&fs, "/", 1);
		f_getfree("", &fre_clust, &pfs);
		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
		fresult = f_open(&fil, "file.bmp", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

		fresult = f_write(&fil,bmp_tou,54,&byteswritten);

		/*
		f_write(&fil,white,115,1);
		uint8_t *buf = (uint8_t *) adres;
		int length = IMAGE_SIZE * 4;
		for(int i=231; i < length; i+=2)
		{
			buffer[0] = buf[i];
			buffer[1] = buf[i];
			buffer[2] = buf[i];
			f_write(&fil,buffer,3,1);
		}
		//*/

		//*
		uint16_t rgb_datr;
		uint32_t adress;
		for (int i = 0; i < IMAGE_SIZE; i++)
		{
//			adress = adres[i];

//			rgb_datr =(uint16_t) adress;
			/*
			B = (uint8_t)(rgb_datr & 0x1f);
			B = (uint8_t)(8.225*B);
			G = (uint8_t)((rgb_datr>>5) & 0x3f);
			G = (uint8_t)(4.047*G);
			R = (uint8_t)((rgb_datr>>11) & 0x1f);
			R = (uint8_t)(8.225*R);
			buffer[2] = B;
			buffer[1] = G;
			buffer[0] = R;
			*/
			buffer[2] = RGB565_TO_B(adres[i]);
			buffer[1] = RGB565_TO_G(adres[i]);
			buffer[0] = RGB565_TO_R(adres[i]);

//			f_write(&fil,buffer,3,&byteswritten);

//			rgb_datr = (adress>>16);
			/*
			B = (uint8_t)(rgb_datr & 0x1f);
			B = (uint8_t)(8.225*B);
			G = (uint8_t)((rgb_datr>>5) & 0x3f);
			G = (uint8_t)(4.047*G);
			R = (uint8_t)((rgb_datr>>11) & 0x1f);
			R = (uint8_t)(8.225*R);
			buffer[2] = B;
			buffer[1] = G;
			buffer[0] = R;

			*/
//			buffer[5] = RGB565_TO_B(rgb_datr);
//			buffer[4] = RGB565_TO_G(rgb_datr);
//			buffer[3] = RGB565_TO_R(rgb_datr);

			fresult = f_write(&fil,buffer,3,&byteswritten);


		}
	//*/
		f_close(&fil);
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
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_UART4_Init();
  uint8_t abc;
  HAL_UART_Receive_IT(&huart4, (uint8_t*)&abc, 1);
  /* USER CODE BEGIN 2 */

  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)adres, 176*144/2);

  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
  HAL_StatusTypeDef ret =  Camera_init(&hi2c1);


  while(ret != HAL_OK)
  {
	  ret = Camera_init(&hi2c1);
  }
  Camera_config();
  //1000 0000 0000 0011 _ 0110 0101 1010 00
  //1011 0000 1000 0110 _ 1011 0101 1000 0000
  //1000 0000 0000 1000 _ 0000 0000 0000 00

  //adres[2]=0xA1A1B1B1;
  //adres[3]=(uint32_t)&adres;
  HAL_Delay(1000);
  while(hdcmi.State != HAL_DCMI_STATE_READY)
  {

  }
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  ret = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)adres, IMAGE_SIZE);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  //HAL_DCMI_Stop(&hdcmi);




//  while((hdcmi.Instance->CR & DCMI_CR_CAPTURE)!= 0){
//
//  }


  //HAL_Delay(500);
//  ret = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)adres, IMAGE_SIZE);
//  HAL_Delay(500);
//  ret = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)adres, IMAGE_SIZE);
//  HAL_Delay(500);

  //HAL_DCMI_Stop(&hdcmi);



//  fresult = f_mount(&fs, "/", 1);



  /* Check free space */
//  f_getfree("", &fre_clust, &pfs);
//
//  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
//
//  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

  /* Open file to write/ create a file if it doesn't exist */
//  fresult = f_open(&fil, "file1.bmp", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

//    f_puts(bmp_tou, &fil);
//  f_write(&fil,bmp_tou,54,1);
/*

  uint16_t rgb_datr;
  uint32_t adress;
  for (int i = 0; i < IMAGE_SIZE; i++)
  {
	adress = adres[i];

	//adress = hdcmi.Instance->DR;


	rgb_datr = (adress>>16);
	B = (uint8_t)(rgb_datr & 0x1f);
	B = (uint8_t)(8.225*B);
	G = (uint8_t)((rgb_datr>>5) & 0x3f);
	G = (uint8_t)(4.047*G);
	R = (uint8_t)((rgb_datr>>11) & 0x1f);
	R = (uint8_t)(8.225*R);
//        B = ((uint8_t)rgb_datr) & 0x1f;
//        G = ((uint8_t)(rgb_datr>>5)) & 0x3f;
//        R = ((uint8_t)(rgb_datr>>11)) & 0x1f;
	buffer[2] = B;
	buffer[1] = G;
	buffer[0] = R;
//    	f_puts(buffer, &fil);
	f_write(&fil,buffer,3,1);




	rgb_datr =(uint16_t) adress;
	B = (uint8_t)(rgb_datr & 0x1f);
	B = (uint8_t)(8.225*B);
	G = (uint8_t)((rgb_datr>>5) & 0x3f);
	G = (uint8_t)(4.047*G);
	R = (uint8_t)((rgb_datr>>11) & 0x1f);
	R = (uint8_t)(8.225*R);

//        B = ((uint8_t)rgb_datr) & 0x1f;
//        G = ((uint8_t)(rgb_datr>>5)) & 0x3f;
//        R = ((uint8_t)(rgb_datr>>11)) & 0x1f;
	buffer[2] = B;
	buffer[1] = G;
	buffer[0] = R;
//    	f_puts(buffer, &fil);
	f_write(&fil,buffer,3,1);






//    	adres++;

  }
*/
  /* Writing text */
//    f_puts("This data is from the FILE1.txt. And it was written using ...f_puts... sdasd asd asdsa d ", &fil);

  /* Close file */
//  fresult = f_close(&fil);
//  createPhoto();

//  while(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int bul=1;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  while (1)
  {
	  if(adres[0]!=0 && adres[IMAGE_SIZE-1]==0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  }
	  if(adres[IMAGE_SIZE-1]!=0 && bul==1)
	  {
		  bul=0;
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//		  createPhotoFlesh();
		  createPhoto();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI_CS_Pin|CAMERA_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : Cam_Btn_Pin */
  GPIO_InitStruct.Pin = Cam_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Cam_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin CAMERA_RESET_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|CAMERA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}


/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
//  HAL_UART_MspInit(&huart4);
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}


volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;

extern "C" void SDTimer_Handler(void)
{
  if(Timer1 > 0)
    Timer1--;

  if(Timer2 > 0)
    Timer2--;
}


extern "C" void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  FatFsCnt++;
  if(FatFsCnt >= 10)
  {
	FatFsCnt = 0;
	SDTimer_Handler();
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}



extern "C" void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	ttDataLinkLayerPoint.RxByteHandler((uint8_t)UART4->DR);
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 4 */
void write(uint8_t val)
{
  HAL_UART_Transmit(&huart4, &val, 1, 0xFFFF);
}


void read(uint8_t *UardRecivedData, uint16_t len)
{
    if (len == 1)
    {
      if (UardRecivedData[0] == 0x02)
      {
    	  	 write_flag = true;
      }
    }
    else
    {

    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
