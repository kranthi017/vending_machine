/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOA
#define RW_Pin GPIO_PIN_5
#define RW_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_6
#define EN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOA
#define DB0_Pin GPIO_PIN_0
#define DB0_GPIO_Port GPIOB
#define DB1_Pin GPIO_PIN_1
#define DB1_GPIO_Port GPIOB
#define DB2_Pin GPIO_PIN_2
#define DB2_GPIO_Port GPIOB
#define DB3_Pin GPIO_PIN_3
#define DB3_GPIO_Port GPIOB
#define DB4_Pin GPIO_PIN_4
#define DB4_GPIO_Port GPIOB
#define DB5_Pin GPIO_PIN_5
#define DB5_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_6
#define DB6_GPIO_Port GPIOB
#define DB7_Pin GPIO_PIN_7
#define DB7_GPIO_Port GPIOB
#define ROW 2
#define COLUMN 16



uint8_t smsat[]="AT\r";
uint8_t smscmgf[]="AT+CMGF=1\r";
uint8_t smscmgr[]="AT+CMGR=x\r";
uint8_t smscmgd[]="AT+CMGD=1,x\r";
uint8_t smscpms[]="AT+CPMS=\"SM\"\r";
uint8_t smscnmi[]="AT+CNMI=2,2,0,0,0\r";
uint8_t pattern[]="Rs. ";
char gsminit[]="GSM Init....";
char start[]="Press next";
char please_select[]="Please Select";
char pay[]="Pay Rs.00.00";
char tez[]="via Tez....";
char received[]="Received";
char no_selection[]="No selection";
char thank_you[]="Thank You";
char collect[]="Enjoy your food";
char underpaid[]="You underpaid";
char pay_the_rest[]="Pay the rest";
char overpaid[]="You overpaid";
char select_more[]="Select more food";
char wait[]="Wait...";

char Nutcrackers[]="Nuts X 00";
int nuts=0,nutstotal=0;
char Bhujiya[]="Bhujiya X 00";
int bhuj=0,transaction=0,bhujtotal=0;
char error[]="Error, try again";
char welcome[]="Welcome...";
char gsm_received_message[]="Money received";
char money[]="0000000000";
char * str_to_be_paid;

GPIO_PinState button1;
GPIO_PinState button2;
GPIO_PinState button3;

int bufferlen=300;

int i,j=0,mode;
int pos=0,success=0,msg_status=0;
int position;
int under_paid=0, over_paid=0;
float value=0,to_be_paid=0,old_to_be_paid=0;
uint8_t buffer[300];
uint8_t buff;


int Period,DutyCycle=40;


uint8_t count,col, printcnt;	//
uint32_t glitchcnt;	
char char_to_be_printed;		// Used with printchar() function	   		
static char bufferlcd[8];  
char a[] = "Scan->press next";  // 8 digits integer for lcd_itoa(int) function
//DDRAM Address matrix for 20x4 LCD
char lcd_address[ROW][COLUMN]={
												{0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x90, 0x91, 0x92, 0x93},
												{0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xD0, 0xD1, 0xD2, 0xD3}};
		
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void GSM_init()
{
	memset(buffer,0x00,bufferlen);
	
	i=0;
	HAL_UART_Transmit(&huart4,smsat,3,100);

	HAL_Delay(100);
	do
	{
		if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
	}while(i<bufferlen);

	HAL_Delay(100);
	
}

void GSM_cmgf()
{
	memset(buffer,0x00,bufferlen);
	i=0;
	HAL_UART_Transmit(&huart4,smscmgf,sizeof(smscmgf)/sizeof(uint8_t),100);
	HAL_Delay(100);
	do
	{
		if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
	}while(i<bufferlen);
	

	HAL_Delay(100);
}

void GSM_read(int index)
{
	
	smscmgr[8]=index+48;		//to turn into ASCII
	memset(buffer,0x00,bufferlen);
	i=0;
	HAL_UART_Transmit(&huart4,smscmgr,sizeof(smscmgr)/sizeof(uint8_t),100);
	HAL_Delay(100);
	do
	{
		if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
	}while(i<bufferlen);
	

	HAL_Delay(100);
}

void GSM_cpms()
{
	memset(buffer,0x00,bufferlen);
	i=0;
	HAL_UART_Transmit(&huart4,smscpms,sizeof(smscpms)/sizeof(uint8_t),100);
	HAL_Delay(100);
	do
	{
		if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
	}while(i<bufferlen);
	
	HAL_Delay(100);
}
void GSM_delete(int flag)
{
	smscmgd[10]=flag+48;
	memset(buffer,0x00,bufferlen);
	i=0;
	HAL_UART_Transmit(&huart4,smscmgd,sizeof(smscmgd)/sizeof(uint8_t),100);
	HAL_Delay(100);
	do
	{
		if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
	}while(i<bufferlen);
	

	HAL_Delay(100);
}

void GSM_cnmi()
{
	memset(buffer,0x00,bufferlen);
	i=0;
	HAL_UART_Transmit(&huart4,smscnmi,sizeof(smscnmi)/sizeof(uint8_t),100);
	HAL_Delay(100);
	do
	{
		if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
	}while(i<bufferlen);
	

	HAL_Delay(100);
}

int GSM_wait_for_message()
{
	int status=0;
	memset(buffer,0x00,bufferlen);
	while(1){


		i=0;
		while(1){
			if(HAL_UART_Receive(&huart4,&buff,1,5)==HAL_OK);
				if(buff=='\r')
				{
					success=1;
					break;
				}
		}
		do
		{
			if(HAL_UART_Receive(&huart4,&buffer[i++],1,5)==HAL_OK);
		}while(i<bufferlen);
		HAL_Delay(100);
		if(i==300)
		{
			status=1;
			break;
		}
	}
	
	HAL_Delay(100);
	buff=0x00;
	return(status);
}

int search(uint8_t * pattern, int pat_size)
{
	
	for(pos=1; pos<bufferlen; pos++)
	{
		for (j=0; j<pat_size; j++)
		{
            if (buffer[pos+j] != pattern[j]){
					           break;
						}
	  }
        if (j==pat_size){            // if pat[0...M-1] = txt[i, i+1, ...i+M-1]
					
					break;
				}
	}
	return(pos);
}









void write_command(void)		// 
	{
	HAL_GPIO_WritePin(GPIOA, RS_Pin| RW_Pin | EN_Pin, GPIO_PIN_RESET); //RS, RW and EN are made 0
	HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_SET); //EN is made 1
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, RW_Pin, GPIO_PIN_SET); //RW is made 1	
	HAL_GPIO_WritePin(GPIOA, RW_Pin, GPIO_PIN_RESET); //RW is made 0
	HAL_Delay(10);
	}

/**
  * @brief Clear up LCD panel
  * @param none
	* @retval void
 */
void clrlcd(void)
{
/*
	GPIOB->ODR=0x01;
	write_command();
	HAL_GPIO_WritePin(GPIOA, RW_Pin, GPIO_PIN_RESET); //RW is made 0
*/
	HAL_GPIO_WritePin(GPIOA, RS_Pin| RW_Pin , GPIO_PIN_RESET); //RS, RW are made 0
	GPIOB->ODR=0x01;
	HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_SET); //EN is made 1
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET); //EN is made 0
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, RS_Pin, GPIO_PIN_SET);   //RS is made 1
	HAL_GPIO_WritePin(GPIOA, RW_Pin, GPIO_PIN_RESET); //RW is made 0
}
/**
  * @brief Initialize the 20x4 LCD as per sequence indicated in the datasheet
  * @param none
	* @retval void
 */

void lcd_init(void)
{
	GPIOB->ODR=0xFF; //Set DB7-DB0 High
	HAL_GPIO_WritePin(GPIOA, RS_Pin| RW_Pin | EN_Pin, GPIO_PIN_RESET); //RS, RW and EN are made 0
	GPIOB->ODR=0x30; //DB7-DB0=0x30
	write_command();
	GPIOB->ODR=0x38; //DB7-DB0=0x38
	write_command();
	GPIOB->ODR=0x08; //DB7-DB0=0x08
	write_command();
	GPIOB->ODR=0x03; //DB7-DB0=0x03
	write_command();
	GPIOB->ODR=0x06; //DB7-DB0=0x06
	write_command();
	GPIOB->ODR=0x0C; //DB7-DB0=0x0C                     0x0F will make the cursor visible and blinking =1
	write_command();
	GPIOB->ODR=0x80; //DB7-DB0=0x80
	write_command();
	clrlcd();
}

/**
  * @brief Print a single character on 20x4 LCD panel
  * @param char_to_be_printed: Single character to be printed
	* @retval void
 */
void printchar(char  char_to_be_printed)
{
 GPIOB->ODR=char_to_be_printed;
 HAL_GPIO_WritePin(GPIOA, RS_Pin| EN_Pin, GPIO_PIN_SET); //RS and EN are made 1
 HAL_Delay(10); //EN pulse width
 HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET); //EN is made 0
 HAL_GPIO_WritePin(GPIOA, RW_Pin, GPIO_PIN_SET); //RW is made 1	
 HAL_GPIO_WritePin(GPIOA, RW_Pin| RS_Pin| EN_Pin, GPIO_PIN_RESET); //RW, RS and EN are made 0
 
}

/**
  * @brief Move LCD cursor to the user specified location on the panel
  * @param address: address of the location as per datasheet
	* @retval void
 */
void lcd_setcursor(char address)
{
	GPIOB->ODR=address;
	write_command();
	HAL_GPIO_WritePin(GPIOA, RW_Pin, GPIO_PIN_RESET); //RW is made 0	
}

/**
  * @brief Print String at a user specified location on a 20x4 LCD panel
  * @param str: String to be printed with strlen < 20 characters
	* 			 row:	Row number between 0-3
	*				 col: Column number between 0-19	
  * @retval void
 */
void printtextatlocation(char * str, uint8_t row, uint8_t col)
{
	//Uncomment the clrlcd() below if right most digit stays back while printing from a 4 digit value to 3 digit value
	//clrlcd();  
	lcd_setcursor(lcd_address[0][0]); //Cursor to the first location of the first row
	lcd_setcursor(lcd_address[row][0]); //Cursor to the first location of the desired row
	lcd_setcursor(lcd_address[row][col]); //Cursor to the desired location
	
	count= strlen(str);
				for(col=0;col<count; col++)
				{
					char_to_be_printed=str[col];
					printchar(char_to_be_printed);
				}
}
/**
  * @brief Convert integer to String for display on 20x4 LCD panel
  * @param value: Integer number to be converted to string
	* @retval void
 */
char * lcd_itoa(int value) 
 {
     int original = value;        // save original value
		 int c = sizeof(bufferlcd)-1;    

     bufferlcd[c] = 0;                // write NULL in the last byte of the buffer
 
     if (value < 0)                 // if it's negative, take the absolute value
         value = -value;
     
     do                             // write LSB of value
     {
         bufferlcd[--c] = (value % 10) + '0';    
         value /= 10;
     } while (value);
 
     if (original < 0) 
         bufferlcd[--c] = '-';
 
     return &bufferlcd[c];
	 }


	 void wait_button_1()
	 {
		 while(1)
		 {
			 if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==GPIO_PIN_RESET)
			 {
				 break;
			 }
		 }
		 return;
	 }
	 
	 void wait_for_selection()
	 {
		 nuts=0;
		 bhuj=0;
		 Nutcrackers[8]='0';
		 Bhujiya[11]='0';
		 while(1)
			 {
				 
				 button1=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
				 button2=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
				 button3=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
				 
				 if(button2==GPIO_PIN_RESET)
				 {
					 Nutcrackers[8]++;
					 nuts++;
					 HAL_Delay(100);
				 }
				 
				 if(button3==GPIO_PIN_RESET)
				 {
					 Bhujiya[11]++;
					 bhuj++;
					 HAL_Delay(100);
				 }
				 
				 if(button1==GPIO_PIN_RESET)
				 {
					 HAL_Delay(100);
					 return;
					 break;
				 }
				 HAL_Delay(150);
			 }
			 
	 }
	 
	 
void str2int()
{
		float temp=0;
		int k=1;
		value=0;
		i=4;
	  while(money[i]!='.')
		{
			temp=money[i];
			temp=temp-48;
			value=(value*10)+temp;
			i++;
		}
		i++;
		/*temp=0;
		while(money[i]!=0x00)
		{
			temp=money[i];
			temp=temp-48;
			temp=temp/(10^k);
			k++;
			value=value+temp;
		}*/
}

void motor_rotate()
{
	int time=8000;
	while(nutstotal!=0 || bhujtotal !=0)
	{
		if(nutstotal!=0)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			nutstotal--;
		}
		if(bhujtotal!=0)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			bhujtotal--;
		}
		HAL_Delay(time);
		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		
	}
	return;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_UART4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		
	Period = htim1.Init.Period +1; 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //PWM Generation for Contrast Pin
	TIM1->CCR1= DutyCycle*Period/100;
	lcd_init(); //Initialize the LCD
	
	clrlcd();
	printtextatlocation(gsminit,0,0);	
	GSM_init();
	GSM_cmgf();
	GSM_delete(4);
	HAL_Delay(500);
	clrlcd();
		

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		clrlcd();
		printtextatlocation(welcome,0,0);  //Welcome message
		printtextatlocation(start,1,0);		//Press next to continue
		wait_button_1();									//Waiting for the user to press control button PA6
		to_be_paid=0;
		old_to_be_paid=0;
		transaction=1;
		
		
		
		
		while(transaction)
		{
					if(!under_paid)
					{
							clrlcd();
							printtextatlocation(please_select,0,0);		//Print select your choice
							printtextatlocation(start,1,0);					//Press next to continue
							
							wait_for_selection();
							clrlcd();
							printtextatlocation(Nutcrackers,0,0);		//Print selected choices
							printtextatlocation(Bhujiya,1,0);					
							HAL_Delay(2000);
							
					}
					to_be_paid = to_be_paid + (nuts*5) + (bhuj*10);
					nutstotal=nutstotal + nuts;
					bhujtotal=bhujtotal + bhuj;
					
					if(to_be_paid!=0)
					{
								pay[7]='0';
								pay[8]='0';
								pay[9]='.';
								pay[10]='0';
								pay[11]='0';
								
								str_to_be_paid=lcd_itoa(to_be_paid);
						
								i=0;
								while(str_to_be_paid[i]!=0)
								{
										if(to_be_paid<10)
										{
												pay[8+i]=str_to_be_paid[i++];
										}
										else 
										{
												pay[7+i]=str_to_be_paid[i++];
										}
								}
								
								clrlcd();
											//printchar Amount to be paid			
								printtextatlocation(wait,0,0);
								
   				
					
						{
							
							msg_status=0;
							GSM_init();
							GSM_cmgf();
							GSM_delete(4);
							HAL_Delay(500);
							clrlcd();
							printtextatlocation(pay,0,0);	
							printtextatlocation(tez,1,0);
							GSM_cnmi();
							while(msg_status==0)
							{
								
								//Setting up of GSM sms receiving mode
								msg_status=GSM_wait_for_message();		
								
							}
							HAL_Delay(100);
							if(msg_status==1)
							{
										clrlcd();
										printtextatlocation(gsm_received_message,0,0);
										HAL_Delay(500);
										position=search(pattern,3);
										i=0;
										while(buffer[position+i]!='c')
										{
											money[i]=buffer[position+i];
											i++;
										}
										
										str2int();
										value=(int)value;
									}
							else
							{
									clrlcd();
									printtextatlocation(error,0,0);
									HAL_Delay(1000);
							}
								}
							}
										to_be_paid=to_be_paid-value;
										
										under_paid=0;
										over_paid=0;
							if(nutstotal!=0 || bhujtotal!=0)
							{
										if(to_be_paid==0)
										{
													clrlcd();
													printtextatlocation(thank_you,0,0);							
													printtextatlocation(collect,1,0);
													transaction=0;
													HAL_Delay(1000);
													motor_rotate();
			
										}
										else if(to_be_paid>0)
										{
													clrlcd();
													printtextatlocation(underpaid,0,0);							
													printtextatlocation(pay_the_rest,1,0);
													transaction=1;
													under_paid=1;
													HAL_Delay(1000);
										}
										else if(to_be_paid<0)
										{
													clrlcd();
													printtextatlocation(overpaid,0,0);							
													printtextatlocation(select_more,1,0);
													transaction=1;
													over_paid=1;
													HAL_Delay(1000);
													
										}
									}
							else
						{
							clrlcd();
							printtextatlocation(no_selection,0,0);
							HAL_Delay(1000);
							
						}
										msg_status=0;
										value=0;
							
							
							
							HAL_Delay(2000);
							nuts=0;
							bhuj=0;
							
							
						
						
						
						old_to_be_paid=to_be_paid;
						HAL_Delay(1000);
						HAL_Delay(100);
						
		}
						nutstotal=0;
						Nutcrackers[8]='0';
						Bhujiya[11]='0';
						bhujtotal=0;
		HAL_Delay(100);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
