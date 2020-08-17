#include "stm32f3xx_hal.h"
#include "stm32f303xc.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define MPU_ADDRESS		0x68 // AD0 pin na 3.3 V in pol je 0x69, cene nevem (oz niti ne dela (?????))

#define WHO_AM_I		0x75
#define PWR_MGMT_1		0x6B
#define SMPRT_DIV		0x19
#define CONFIG			0x1A
#define GYRO_CONFIG		0x1B
#define ACC_CONFIG		0x1C
#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40
#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42
#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48

float pitch, roll, yaw, pitch1;
uint16_t P1_16;
uint16_t I1_16;
uint16_t D1_16;
uint16_t P2_16;
uint16_t I2_16;
uint16_t D2_16;
uint8_t b = 0;

// offsets calculated at power up
static int16_t gyro_x_offset = 0;
static int16_t gyro_y_offset = 0;
static int16_t gyro_z_offset = 0;
static uint32_t samples = 0;
float accel_x, accel_y, accel_z, gyros_x, gyros_y, gyros_z; //, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;

unsigned long error_stop;
unsigned int proba;
unsigned int proba2;
void Systick_Time_Measure(void);
unsigned char systickFlag;
unsigned char delayFlag;
void delay_us(unsigned long us);

float pwm_spr, P1, I1, D1, P2, I2, D2, P3, I3, D3, flag = 1;
double atan(double x);
double kat_atan;
float comp_angle, angle_1, angle_2, angle_graph;
unsigned long a=6;
float kat_obrotu;
//proby
float PWM, PID_SUM_1, PID_SUM_2;
float Power, Power2, Power1, moc, PWM_minus, PWM_plus, Speed, Displacement, Motor_Power, Speed_1, Angle_PID, Speed_2, Speed_PID, Displacement_PID_1, Displacement_PID_2, speed_graph, displacement_graph;
char start_0 = 0, start_1 = 0;
float P_Speed, I_Speed, D_Speed, P_Angle, I_Angle, D_Angle;

//Encoder variables
volatile int16_t pulse_count_1; // Licznik impulsow
volatile uint16_t positions_1; // Licznik przekreconych pozycji
volatile int16_t pulse_count_2; // Licznik impulsow
volatile uint16_t positions_2; // Licznik przekreconych pozycji
int16_t number_of_pulses_1, pulse_count_1_prev, number_of_pulses_2, pulse_count_2_prev;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim6;
float omega_1, omega_2, RPM_1, RPM_2, Mean_RPM, Mean_RPM_filtered, RPM_1_filtered, RPM_2_filtered;
unsigned int number_of_clock_cycles_1, number_of_clock_cycles_2;

volatile int16_t kroki1, kroki2;

//Encoders declarations
void Encoders_Timer_34_config(void);
void Counting_Timer_7_Config(void);
void Main_Loop_Timer_6_Config(void);
void TIM3_IRQHandler(void);
volatile uint16_t przerwanie, licznik;

//Sensor variables
long acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal = 0;
long acc_x_cal, acc_y_cal, acc_z_cal = 0;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
char set_gyro_angles = 0;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

//GPS variables
double x, y, z, Heading, longitude, latitude;
float x_podane = 215832609, y_podane = 530919237; 
float x_odleg, y_odleg, odleglosc_xy, alfa;
double czas;
float Heading_fl, roznica_katow;
float moc;
unsigned int forward, right, left;
//GPS declarations
void GPS_Data_Frame(char *Buffer);
void GPS_Autonomy_Calculation(void);

//UART variables
char UART2_Tx[42] = "yes\r\n", UART1_Tx[6] = "no\r\n";
char UART2_Rx[8] = "ABCDEFGH", UART1_Rx[124], rxBuff[8];
float HC05_angle;
float throttle, turn, turn_const, throttle_const = 0;
//UART declarations
void UART_2_Config(void);
void DMA_Ch6_Config(void);
void UART_1_Config(void);
void DMA_Ch5_Config(void);
void Robot_Steering(void);

UART_HandleTypeDef UART2handle;
DMA_HandleTypeDef myDMA_Uart2Handle;
UART_HandleTypeDef UART1handle;
DMA_HandleTypeDef myDMA_Uart1Handle;

//Clock declarations
void SystemClock_Config(void);

//Sensor declarations
void Setup_MPU6050_Registers(void);
void Read_MPU6050_Data(void);
void Calculate_Angle(void);

//Direction of motors declarations
void GPIO_Direction_Config(void);

//PID declarations
float PID_Controller(float kp, float ki, float kd, float error);
float PID_Motor_Controller(float kp, float ki, float kd, float input, float set_point);
float PID_Speed_Controller(float kp, float ki, float kd, float input, float set_point);
float PID_Angle_Controller(float kp, float ki, float kd, float input, float set_point);
void PID_Values_Set(void);

//PWM declarations
void PWM_Config(float kanal1, float kanal2);

//I2C declarations
void GPIO_Config_I2C(void);
void I2C_Config(void);
void I2C_Write(uint8_t I2Caddress, uint8_t I2Cdata);
uint8_t I2C_Read(uint8_t I2Caddress);

//SPI declarations
void GPIO_Config_SPI(void);
void SPI_Config(void);
void SPI_Transmit(uint8_t address, uint8_t data);
uint8_t SPI_Read(uint8_t address);
void Timer2_Init (void);
unsigned int Time_GetPeriod(void);

//I2C variables
#define LSM303DLHCaddress 0x32
#define MPU6050address 0xD0
I2C_HandleTypeDef myI2Chandle;
uint8_t i2cBuff[16];
int16_t accel_x_raw, accel_y_raw, accel_z_raw, mpu_temp_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

float roll;
uint8_t i;

//SPI variables
uint8_t OUT_X_H, OUT_Y_H, OUT_Z_H;
float W_X, W_Y, W_Z;  
SPI_HandleTypeDef mySPIhandle;
uint8_t spiData[2], spiRx[2];
float GyroRate;
unsigned short gyroTime;
float x_angle;
char SPI;

float flaga=0;
float plus = 0;
float RPM_pulse_counting;
int16_t pulse_count, number_of_pulses, pulse_count_prev;
uint32_t dupa;

 int main(void){
	HAL_Init();
	SystemClock_Config();
	//GPIO_Config_SPI();
	//SPI_Config();
	GPIO_Config_I2C();
	I2C_Config();
	//Timer2_Init();
	GPIO_Direction_Config();
	Setup_MPU6050_Registers();
	UART_2_Config();
	DMA_Ch6_Config();
	UART_1_Config();
	DMA_Ch5_Config();
	Encoders_Timer_34_config();
	PID_Values_Set();
	
	
	HAL_UART_Receive_DMA(&UART2handle, (uint8_t *)UART2_Rx, 8);
	HAL_UART_Receive_DMA(&UART1handle, (uint8_t *)UART1_Rx, 124);
	
	Main_Loop_Timer_6_Config();
	Counting_Timer_7_Config();
	
	while(1){}
}

//Main loop functions
void Main_Loop_Timer_6_Config(void){
	__HAL_RCC_TIM6_CLK_ENABLE();
	
	htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 32000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim6);
	//Enable Timer 6
	HAL_TIM_Base_Start_IT(&htim6);
	//Enable Timer 6 interrupt
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM6_DAC_IRQHandler(void){
	dupa++;
	HAL_TIM_IRQHandler(&htim6);
	

	Read_MPU6050_Data();
	Calculate_Angle();
	pitch = -angle_roll_output;

//	MadgwickAHRSupdateIMU(gyros_x, gyros_y, gyros_z, accel_x, accel_y, accel_z);
//	pitch = -(asinf(-2.0f * (q1*q3 - q0*q2))*(180/3.14))+0.8;
//	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*(180/3.14);
//	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*(180/3.14);
	
	Mean_RPM = (RPM_1+RPM_2)/2;
	Mean_RPM_filtered = 0.9*Mean_RPM_filtered + 0.1*Mean_RPM;
	
	Speed_PID = PID_Speed_Controller(P2, I2, D2, Mean_RPM_filtered, throttle_const);
	angle_1 = pitch + Speed_PID; // + odchylenie krokow
	
	
	if((-pitch>4 && -pitch<5) || (-pitch>-5 && -pitch<-4)) start_0 = 1;
	if(start_0==1 && -pitch<0.2 && -pitch>-0.2) start_1 = 1;
	if(start_1 == 1){
		Angle_PID = PID_Angle_Controller(P1, I1, D1, angle_1, 0);
		Power1 = PID_Motor_Controller(1, 0, 0, Angle_PID + RPM_1 - turn_const, 0);
		Power2 = PID_Motor_Controller(1, 0, 0, Angle_PID + RPM_2 + turn_const, 0);
	}
	
	if(pitch>20 || pitch<-20) Power1=0, Power2=0;
	
	
	GPS_Data_Frame(UART1_Rx);
	GPS_Autonomy_Calculation();
	Robot_Steering();
	PWM_Config(Power1, Power2);

	//sprintf(UART2_Tx, "pitch: %f\r\n", angle_roll_output);
	//HAL_UART_Transmit(&UART2handle, (uint8_t *)UART2_Tx, 42, 50);
}

//GPS Navigation
void GPS_Data_Frame(char *Buffer){
	
  char *p = Buffer + 6;		// Payload in NAV-HPPOSECEF
  char *p2 = Buffer + 42; //Payload in NAV-VELNED
	char *p3 = Buffer + 86; //Payload in NAV-HPPOSLLH

//  0 U1 Version
//  4 U4 iTOW ms GPS time of the week
//  8 I4 ecefX cm
// 12 I4 ecefY
// 16 I4 ecefZ
// 20 I1 ecefXHp 0.1 mm +/-99
// 21 I1 ecefYHp
// 22 I1 ecefZHp
// 23 U1 reserved2
// 24 U4 pAcc mm Position accuracy estimate 1e-1 (0.1 mm)


	//HPPOSECEF
	czas = *((unsigned long *) &p[0x04]);	// iTOW
	x = *((signed long *) &p[0x08]);	// ecefX cm
	y = *((signed long *) &p[0x0C]);	// ecefY
	z = *((signed long *) &p[0x10]);	// ecefZ
	//HPPOSLLH
	longitude = *((signed long *) &p2[0x08]);	// longitude
	latitude = *((signed long *) &p2[0x0C]);	// latitude
	//VELNED
	Heading = *((signed long *) &p3[0x18]);	// Heading 2-D
	Heading_fl = Heading/100000;
}

void GPS_Autonomy_Calculation(void){
	//potrzebne dane
	x_odleg = x_podane - longitude; //cm
	y_odleg = y_podane - latitude; //cm
	odleglosc_xy = sqrt((x_odleg*x_odleg)+(y_odleg*y_odleg)); //cm
	alfa = (180*atan2(x_odleg, y_odleg))/3.14; //deg
	if(alfa < 0){
		alfa = alfa + 360;
	}
	roznica_katow = Heading_fl - alfa;
	
	//funkcja ruchu
	if(fabs(roznica_katow)>180){
		moc = 360 - fabs(roznica_katow);
	}
	else{
		moc = fabs(roznica_katow);
	}
	if(odleglosc_xy > 300){
		forward = 100;
		//HAL_Delay(1500); //ms
		if(roznica_katow == 0){
			right =0;
			left = 0;
		}
		else if((roznica_katow>0 && roznica_katow <180) || (roznica_katow<-180 && roznica_katow>-360)){ //w lewo
			right = 0;
			left = moc;
		}
		else if((roznica_katow>180 && roznica_katow<360) || (roznica_katow<0 && roznica_katow>-180)){  //w prawo
			left = 0;
			right = moc;
		}
		HAL_UART_Transmit(&UART2handle, (uint8_t *)UART1_Tx, 6, 50);
	}
	else{
		forward = 0;
		right = 0;
		left = 0;
		HAL_UART_Transmit(&UART2handle, (uint8_t *)UART2_Tx, 6, 50);
	}
}

//Encoders functions
void TIM3_IRQHandler(void){
	pulse_count_1 = (int16_t)TIM3->CNT;
	number_of_pulses_1 = pulse_count_1 - pulse_count_1_prev;
	pulse_count_1_prev = pulse_count_1;
	omega_1= 25000.0f/(float)number_of_clock_cycles_1;//(2*3.14*10000)/(1632.67*number_of_clock_cycles_1);
	if(number_of_pulses_1<0) omega_1*=-1, kroki1--;
	else if(number_of_pulses_1>0) kroki1++;
	RPM_1=(omega_1*60)/(2*3.14);
	RPM_1_filtered =  0.85*RPM_1_filtered + 0.15*RPM_1;
	number_of_clock_cycles_1=0;
	//przerwanie++;
	HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void){
	pulse_count_2 = (int16_t)TIM4->CNT;
	number_of_pulses_2 = pulse_count_2 - pulse_count_2_prev;
	pulse_count_2_prev = pulse_count_2;
	omega_2=(2*3.14*10000)/(1632.67*number_of_clock_cycles_2);
	if(number_of_pulses_2<0) omega_2*=-1, kroki2--;
	else if(number_of_pulses_2>0) kroki2++;
	RPM_2=(omega_2*60)/(2*3.14);
	RPM_2_filtered =  0.85*RPM_2_filtered + 0.15*RPM_2;
	number_of_clock_cycles_2=0;
	HAL_TIM_IRQHandler(&htim4);
	//przerwanie=2;
}

void TIM7_IRQHandler(void){
	//przerwanie=5;
	HAL_TIM_IRQHandler(&htim7);
	pulse_count_1 = (int16_t)TIM3->CNT;
	pulse_count_2 = (int16_t)TIM4->CNT;
	
	number_of_pulses_1 = pulse_count_1 - pulse_count_1_prev;
	number_of_pulses_2 = pulse_count_2 - pulse_count_2_prev;
	
	pulse_count_1_prev = pulse_count_1;
	pulse_count_2_prev = pulse_count_2;
	
	RPM_1 = (number_of_pulses_1*25*60)/1632.67;
	RPM_2 = (number_of_pulses_2*25*60)/1632.67;
}

void Counting_Timer_7_Config(void){
	__HAL_RCC_TIM7_CLK_ENABLE();
	
	htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim7);
	//Enable Timer 7
	HAL_TIM_Base_Start_IT(&htim7);
	//Enable Timer 7 interrupt
	HAL_NVIC_SetPriority(TIM7_IRQn, 0, 5);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

void Encoders_Timer_34_config(void){
	//Timer 3 (encoder 1)
	GPIO_InitTypeDef myEncoder1def;
	TIM_Encoder_InitTypeDef encoder_1;
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	myEncoder1def.Mode = GPIO_MODE_AF_PP;
	myEncoder1def.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	myEncoder1def.Pull = GPIO_PULLUP;
	myEncoder1def.Alternate = GPIO_AF2_TIM3;
	myEncoder1def.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &myEncoder1def);
	
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  encoder_1.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder_1.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder_1.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_1.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder_1.IC1Filter = 15;
  encoder_1.IC2Polarity = TIM_ICPOLARITY_RISING; //bylo rising
  encoder_1.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_1.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder_1.IC2Filter = 15;
	HAL_TIM_Encoder_Init(&htim3, &encoder_1);
	
	//Timer 4 (encoder 2)
	TIM_Encoder_InitTypeDef encoder_2;
	GPIO_InitTypeDef myEncoder2def;
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	myEncoder2def.Mode = GPIO_MODE_AF_PP;
	myEncoder2def.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	myEncoder2def.Pull = GPIO_PULLDOWN;
	myEncoder2def.Alternate = GPIO_AF2_TIM3;
	myEncoder2def.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &myEncoder2def);
	
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  encoder_2.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder_2.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder_2.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_2.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder_2.IC1Filter = 15;
  encoder_2.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder_2.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_2.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder_2.IC2Filter = 15;
	HAL_TIM_Encoder_Init(&htim4, &encoder_2);
	
	//Enable encoders 1 and 2
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	//NVIC_EncodePriority
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
	//HAL_NVIC_EnableIRQ(TIM3_IRQn);
	//HAL_NVIC_EnableIRQ(TIM4_IRQn);
}


//UART functions
void UART_2_Config(void){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	
	GPIO_InitTypeDef uart2;
	uart2.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	uart2.Mode = GPIO_MODE_AF_PP;
	uart2.Pull = GPIO_PULLUP;
	uart2.Speed = GPIO_SPEED_FREQ_HIGH;
	uart2.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &uart2);
	//UART Configuration
	UART2handle.Instance = USART2;
	UART2handle.Init.BaudRate = 115200;
	UART2handle.Init.Mode = UART_MODE_TX_RX;
	UART2handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART2handle.Init.StopBits = UART_STOPBITS_1;
	UART2handle.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&UART2handle);
	
	//Systick interrupt enable
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void DMA_Ch6_Config(void){
	__HAL_RCC_DMA1_CLK_ENABLE();
	myDMA_Uart2Handle.Instance = DMA1_Channel6;
	myDMA_Uart2Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	myDMA_Uart2Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	myDMA_Uart2Handle.Init.MemInc = DMA_MINC_ENABLE;
	myDMA_Uart2Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	myDMA_Uart2Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	myDMA_Uart2Handle.Init.Mode = DMA_CIRCULAR;
	myDMA_Uart2Handle.Init.Priority = DMA_PRIORITY_LOW;
	//myDMA_Uart2Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&myDMA_Uart2Handle);
	
	__HAL_LINKDMA(&UART2handle,hdmarx,myDMA_Uart2Handle);
}

void DMA1_Channel6_IRQHandler(void){
  HAL_DMA_IRQHandler(&myDMA_Uart2Handle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
//	HAL_UART_Transmit(&UART2handle, (uint8_t *)UART2_Rx, strlen(UART2_Rx), 10);
//	HAL_UART_Transmit(&UART1handle, (uint8_t *)UART1_Rx, strlen(UART1_Rx), 10);
	
	// moze dac przeliczanie czegos, moze parsowanie wiadomosci
}

void UART_1_Config(void){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
	GPIO_InitTypeDef uart1;
	uart1.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	uart1.Mode = GPIO_MODE_AF_PP;
	uart1.Pull = GPIO_PULLUP;
	uart1.Speed = GPIO_SPEED_FREQ_HIGH;
	uart1.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &uart1);
	//UART Configuration
	UART1handle.Instance = USART1;
	UART1handle.Init.BaudRate = 19200;
	UART1handle.Init.Mode = UART_MODE_TX_RX;
	UART1handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART1handle.Init.StopBits = UART_STOPBITS_1;
	UART1handle.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&UART1handle);
	
	//Systick interrupt enable
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void DMA_Ch5_Config(void){
	__HAL_RCC_DMA1_CLK_ENABLE();
	myDMA_Uart1Handle.Instance = DMA1_Channel5;
	myDMA_Uart1Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	myDMA_Uart1Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	myDMA_Uart1Handle.Init.MemInc = DMA_MINC_ENABLE;
	myDMA_Uart1Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	myDMA_Uart1Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	myDMA_Uart1Handle.Init.Mode = DMA_CIRCULAR;
	myDMA_Uart1Handle.Init.Priority = DMA_PRIORITY_LOW;
	//myDMA_Uart1Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&myDMA_Uart1Handle);
	
	__HAL_LINKDMA(&UART1handle,hdmarx,myDMA_Uart1Handle);
}

void DMA1_Channel5_IRQHandler(void){
  HAL_DMA_IRQHandler(&myDMA_Uart1Handle);
}

void Robot_Steering(void){
////////////////Parsowanie tablicy znakow////////////////
		for(uint8_t i=0; i<8; i++){
			uint8_t out = 0;
			uint8_t k=0;
			if(out == 1) break;
			if(UART2_Rx[i] == 'X'){
				for(uint8_t j=0; j<8; j++){
					if((i+j)>7){
						i=0;
						k=0;
					}
					rxBuff[j] = UART2_Rx[i+k];
					k++;
					if(j==7) out = 1;
				} 
			}
		}
////////////////Parsowanie tablicy znakow////////////////
		
		throttle = (rxBuff[1]-48)*100 + (rxBuff[2]-48)*10 + (rxBuff[3]-48)*1;
		if(throttle<100 || throttle>300) throttle = 200;
		throttle_const = (-(throttle - 200)/4) + forward/20;
		
		turn = (rxBuff[5]-48)*100 + (rxBuff[6]-48)*10 + (rxBuff[7]-48)*1;
		if(turn<100 || turn>300) turn = 200;
		turn_const = ((turn - 200)*2) + (left/5) - (right/5);
}

//I2C functions
void GPIO_Config_I2C(void){
	//Enable ports clocks
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	//Init typedef
	GPIO_InitTypeDef myPinInit;
	//LED pins config
	myPinInit.Pin = GPIO_PIN_8;
	myPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	myPinInit.Pull = GPIO_NOPULL;
	myPinInit.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &myPinInit);
	//I2C pins config
	myPinInit.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	myPinInit.Mode = GPIO_MODE_AF_OD;
	myPinInit.Pull = GPIO_PULLUP;
	myPinInit.Speed = GPIO_SPEED_FREQ_HIGH;
	myPinInit.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &myPinInit);
	
	
	//Systick interrupt enable for HAL_Delay function
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void I2C_Config(void){
	//Enable I2C peripheral clock
	__HAL_RCC_I2C1_CLK_ENABLE();
	
	myI2Chandle.Instance = I2C1;
	myI2Chandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	myI2Chandle.Init.Timing = 0x2000090E;//myI2Chandle.Init.ClockSpeed = 100000;
	myI2Chandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	myI2Chandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;//myI2Chandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	myI2Chandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	myI2Chandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	myI2Chandle.Init.OwnAddress1 = 0;
	myI2Chandle.Init.OwnAddress2 = 0;
	HAL_I2C_Init(&myI2Chandle);
	
	
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	
}

void I2C_Write(uint8_t I2Caddress, uint8_t I2Cdata){
//ad 0x20 -> data 0x47 (xyz, 100hz)
//ad 0x23 -> data 0x58 (4g, MSB-bigendian, high res, continuus update)
	i2cBuff[0] = I2Caddress;   //Register address: Accelerometer config 1
	i2cBuff[1] = I2Cdata;   //Data to write, +-8g range
	HAL_I2C_Master_Transmit(&myI2Chandle, MPU6050address, i2cBuff, 2, 10);
}

uint8_t I2C_Read(uint8_t I2Caddress){
	i2cBuff[0] = I2Caddress;
	HAL_I2C_Master_Transmit(&myI2Chandle, MPU6050address, i2cBuff, 1, 10);
	//Read data
	i2cBuff[1] = 0x00;
	HAL_I2C_Master_Receive_IT(&myI2Chandle, MPU6050address, &i2cBuff[1], 15);
	return i2cBuff[1];
}
void I2C1_EV_IRQHandler(void){
	//przerwanie=3;
	HAL_I2C_EV_IRQHandler(&myI2Chandle);
	
}


//SPI functions
void GPIO_Config_SPI(void){
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	
	GPIO_InitTypeDef myPinInit;
	//SPI pins
	myPinInit.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	myPinInit.Mode = GPIO_MODE_AF_PP;
	myPinInit.Speed = GPIO_SPEED_FREQ_HIGH;
	myPinInit.Pull = GPIO_NOPULL;
	myPinInit.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &myPinInit);
	//LED pins configuration
	myPinInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	myPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	myPinInit.Pull = GPIO_NOPULL;
	myPinInit.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &myPinInit);
	//CS configuration
	myPinInit.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOE, &myPinInit);
	//INT1 pin cofiguration
	myPinInit.Pin = GPIO_PIN_0;
	myPinInit.Mode = GPIO_MODE_IT_RISING;
	HAL_GPIO_Init(GPIOE, &myPinInit);
	
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void SPI_Config(void){
	mySPIhandle.Instance = SPI1;
	SPI1->CR1 |= SPI_CR1_BR_1 |SPI_CR1_BR_0;		//BR[2:0]	fcplk/16
	SPI1->CR1 &= ~SPI_CR1_CPHA;		//	CPHA  1edge
	SPI1->CR1 &= ~SPI_CR1_CPOL;    //	CPOL  high
	SPI1->CR1 &= ~SPI_CR1_CRCEN;    //CRC calculation disabled
	SPI1->CRCPR = 0x7;              //SPI CRC polynomial register = 7
	SPI1->CR2 |= SPI_CR2_DS_0 |SPI_CR2_DS_1 | SPI_CR2_DS_2; //	DS[3:0]  8 bit size data 
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;//	BIDIOE  ~  __(jednokierunkowe 2 linie)
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;		// MSB first
	
	//////////////////////////////////////////////////////////////////////
	//SPI1->CR1 |= (SPI_CR1_MSTR |SPI_CR1_SSI);  // Master mode
	mySPIhandle.Init.Mode = SPI_MODE_MASTER;
	mySPIhandle.Init.NSS = SPI_NSS_SOFT;
	SPI1->CR2 &= ~SPI_CR2_FRF;           //TI mode disable (motorola enabled)
	HAL_SPI_Init(&mySPIhandle);
	///////////////////////////////////////////////////////////////////////
	///
	//SPI1->CR2 |= SPI_CR2_FRXTH;
	//
}



void SPI_Transmit(uint8_t address, uint8_t data){
	//********SPI Write Operation Routine*****
	//1. CS low
	GPIOE->BRR = GPIO_BRR_BR_3;
	//2. Transmit register adress
	spiData[0] = address;
	
	SPI1->DR = spiData[0];
	//3. Transmit 1 byte data to register
	while( !(SPI1->SR & SPI_SR_TXE) );
	spiData[0] = data; ////////////////////
	HAL_SPI_Transmit(&mySPIhandle, spiData, 1, 50);
	//4. CS high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}


uint8_t SPI_Read(uint8_t address){
	//****** SPI Read Operation Routine******
	//1. CS low
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	//2. Transmit Register adress to read from masked with 0x80
	spiData[0] = 0x80|address; ////////////////
	HAL_SPI_Transmit(&mySPIhandle, spiData, 1, 50);
	//3. Receive reguster 1 byte of data
	HAL_SPI_Receive(&mySPIhandle, spiRx, 1, 50);
	//4. CS high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	return spiRx[0];
}

void EXTI0_IRQHandler(void){ //zakreskowane w stm32f3xx.it.c
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void SysTick_Handler(void){  //zakreskowane w stm32f3xx.it.c
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	systickFlag = 1;
}

unsigned int Time_GetPeriod(void){
	static unsigned int lastTime=0;
	unsigned int period;
	period = TIM2->CNT-lastTime;
	lastTime=TIM2->CNT;
	return period;
}
void Timer2_Init (void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // frequency APB2(8MHz)
	TIM2->PSC = 8-1;                      // frequency 1MHz 
	TIM2->ARR = 1;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->CR1 = TIM_CR1_CEN;
	//NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void){
	delayFlag = 0;
	if (TIM2->SR & TIM_SR_UIF){
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

void delay_us(unsigned long us){
	NVIC_DisableIRQ(TIM2_IRQn);
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // frequency APB2(8MHz)
	TIM2->PSC = 8-1;//8000000 / (1/us);                      // frequency 1MHz 
	TIM2->ARR = us;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->CR1 = TIM_CR1_CEN;
	TIM2->CNT = 0;
	NVIC_EnableIRQ(TIM2_IRQn);
	while(delayFlag != 0);
	delayFlag = 1;
	NVIC_DisableIRQ(TIM2_IRQn);
}

void Systick_Time_Measure(void){
	SysTick->CTRL=0;
	SysTick->LOAD=1600000000; 
	SysTick->VAL=0;
	SysTick->CTRL = 0x00000007;
}

//PWM functions
void PWM_Config(float kanal1, float kanal2){
	TIM_HandleTypeDef myPWMhandle;
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
 GPIO_InitTypeDef myPWMdef;
 myPWMdef.Mode = GPIO_MODE_AF_PP;
 myPWMdef.Pin = GPIO_PIN_11 | GPIO_PIN_9;
 myPWMdef.Pull = GPIO_NOPULL;
 myPWMdef.Alternate = GPIO_AF2_TIM1;
 myPWMdef.Speed = GPIO_SPEED_FREQ_HIGH;
 HAL_GPIO_Init(GPIOE, &myPWMdef);
 
 myPWMhandle.Instance = TIM1;
 myPWMhandle.Init.Period = 10000;
 myPWMhandle.Init.Prescaler = 8;
 myPWMhandle.Init.ClockDivision = 0;
 myPWMhandle.Init.CounterMode = TIM_COUNTERMODE_UP;
 myPWMhandle.Init.RepetitionCounter = 0;
 myPWMhandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
 HAL_TIM_PWM_Init(&myPWMhandle);
 
 TIM_OC_InitTypeDef oc;
 oc.OCMode = TIM_OCMODE_PWM1;
 oc.OCPolarity = TIM_OCPOLARITY_HIGH;
 oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
 oc.OCFastMode = TIM_OCFAST_ENABLE;
 oc.OCIdleState = TIM_OCIDLESTATE_SET;
 oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 
 
 if(kanal1<0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else if(kanal1>0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	}
 if(kanal2<0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else if(kanal2>0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	}
 
	kanal1 = fabs(kanal1);
	kanal2 = fabs(kanal2);
	
	oc.Pulse = kanal1*10;
	HAL_TIM_PWM_ConfigChannel(&myPWMhandle, &oc, TIM_CHANNEL_1);
 
	oc.Pulse = kanal2*10;
	HAL_TIM_PWM_ConfigChannel(&myPWMhandle, &oc, TIM_CHANNEL_2);
 
	HAL_TIM_PWM_Start(&myPWMhandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&myPWMhandle, TIM_CHANNEL_2);
}
//PID
float PID_Motor_Controller(float kp, float ki, float kd, float input, float set_point){
	
	float error = input - set_point;

	// calculate the proportional component (current error * p scalar)
	float p_scalar = kp;
	if(p_scalar < 0) p_scalar = 0;
	float proportional = error * p_scalar;

	// calculate the integral component (summation of past errors * i scalar)
	float i_scalar = ki;
	if(i_scalar < 0) i_scalar = 0;
	static float integral = 0;
	integral += error * i_scalar;
	if(integral >  1000) integral = 1000; // limit wind-up
	if(integral < -1000) integral = -1000;

	// calculate the derivative component (change since previous error * d scalar)
	static float previous_error = 0;
	float d_scalar = kd;
	if(d_scalar < 0) d_scalar = 0;
	float derivative = (error - previous_error) * d_scalar;
	previous_error = error;
	
	return Motor_Power = proportional + integral + derivative;
}
float PID_Speed_Controller(float kp, float ki, float kd, float input, float set_point){
	
	float error = input - set_point;

	// calculate the proportional component (current error * p scalar)
	float p_scalar = kp;
	if(p_scalar < 0) p_scalar = 0;
	float proportional = error * p_scalar;
	P_Speed = proportional*100;

	// calculate the integral component (summation of past errors * i scalar)
	float i_scalar = ki;
	if(i_scalar < 0) i_scalar = 0;
	static float integral = 0;
	integral += error * i_scalar;
	if(integral >  1000) integral = 1000; // limit wind-up
	if(integral < -1000) integral = -1000;
	I_Speed = integral*100;

	// calculate the derivative component (change since previous error * d scalar)
	static float previous_error = 0;
	float d_scalar = kd;
	if(d_scalar < 0) d_scalar = 0;
	float derivative = (error - previous_error) * d_scalar;
	previous_error = error;
	D_Speed = derivative*100;
	
	return Speed = proportional + integral + derivative;
	
}
float PID_Angle_Controller(float kp, float ki, float kd, float input, float set_point){
	
	float error = input - set_point;

	// calculate the proportional component (current error * p scalar)
	float p_scalar = kp;
	if(p_scalar < 0) p_scalar = 0;
	float proportional = error * p_scalar;
	P_Angle = proportional;

	// calculate the integral component (summation of past errors * i scalar)
	float i_scalar = ki;
	if(i_scalar < 0) i_scalar = 0;
	static float integral = 0;
	integral += error * i_scalar;
	if(integral >  1000) integral = 1000; // limit wind-up
	if(integral < -1000) integral = -1000;
	I_Angle = integral;

	// calculate the derivative component (change since previous error * d scalar)
	static float previous_error = 0;
	float d_scalar = kd;
	if(d_scalar < 0) d_scalar = 0;
	float derivative = (error - previous_error) * d_scalar;
	previous_error = error;
	D_Angle = derivative;
	
	return PWM = proportional + integral + derivative;
}

void GPIO_Direction_Config(void){
	//Enable ports clocks
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	//Init typedef
	GPIO_InitTypeDef myPinInit;
	//Motor Direction pins config
	myPinInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	myPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	myPinInit.Pull = GPIO_NOPULL;
	myPinInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &myPinInit);
}

void PID_Values_Set(void){
	// Angle PID
	P1 = 50; 		//65
	I1 = 1.5;  		//2
	D1 = 850;		//600
	// Speed PID
	P2 = 0.1; 	//0.015
	I2 = 0.001;
	D2 = 0.03;
}

//MPU6050
void Read_MPU6050_Data(void){                                             //Subroutine for reading the raw gyro and accelerometer data
//  I2C_Read(0x3A);
//	
//	
//	// extract the raw values
//	accel_x_raw  = i2cBuff[2]  << 8 | i2cBuff[3];
//	accel_y_raw  = i2cBuff[4]  << 8 | i2cBuff[5];
//	accel_z_raw  = i2cBuff[6]  << 8 | i2cBuff[7];
//	mpu_temp_raw = i2cBuff[8]  << 8 | i2cBuff[9];
//	gyro_x_raw   = i2cBuff[10]  << 8 | i2cBuff[11];
//	gyro_y_raw   = i2cBuff[12] << 8 | i2cBuff[13];
//	gyro_z_raw   = i2cBuff[14] << 8 | i2cBuff[15];

//	
//	// calculate the offsets at power up
//	if(samples < 64) {
//		samples++;
//		return;
//	} else if(samples < 128) {
//		gyro_x_offset += gyro_x_raw;
//		gyro_y_offset += gyro_y_raw;
//		gyro_z_offset += gyro_z_raw;
//		samples++;
//		return;
//	} else if(samples == 128) {
//		gyro_x_offset /= 64;
//		gyro_y_offset /= 64;
//		gyro_z_offset /= 64;
//		samples++;
//	} else {
//		gyro_x_raw -= gyro_x_offset;
//		gyro_y_raw -= gyro_y_offset;
//		gyro_z_raw -= gyro_z_offset;
//	}

//	// convert accelerometer readings into G's
//	accel_x = accel_x_raw / 8192.0f;
//	accel_y = accel_y_raw / 8192.0f;
//	accel_z = accel_z_raw / 8192.0f;

//	// convert temperature reading into degrees Celsius
//	float mpu_temp = mpu_temp_raw / 340.0f + 36.53f;

//	// convert gyro readings into Radians per second
//	gyro_x = gyro_x_raw / 939.650784f;
//	gyro_y = gyro_y_raw / 939.650784f;
//	gyro_z = gyro_z_raw / 939.650784f;

	I2C_Read(ACCEL_XOUT_H-1);

	acc_x = ((i2cBuff[2] << 8) | i2cBuff[3]);
	acc_y = ((i2cBuff[4] << 8) | i2cBuff[5]);
	acc_z = ((i2cBuff[6] << 8) | i2cBuff[7]);

	gyro_x = ((i2cBuff[10] << 8) | i2cBuff[11]);
	gyro_y = ((i2cBuff[12] << 8) | i2cBuff[13]);
	gyro_z = ((i2cBuff[14] << 8) | i2cBuff[15]);

}

void Setup_MPU6050_Registers(void){
	//Setup the MPU-6050
	I2C_Write(PWR_MGMT_1, 0x00);
	I2C_Write(GYRO_CONFIG, 0x08);
	I2C_Write(ACC_CONFIG, 0x10);
	I2C_Write(CONFIG, 0x03);
	
	for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
		Read_MPU6050_Data();                                              //Read the raw acc and gyro data from the MPU-6050
		gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
		gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
		gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
		acc_x_cal += acc_x;
		acc_y_cal += acc_y;
		HAL_Delay(1);                                                          //Delay 3us to simulate the 250Hz program loop
  }
//  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
//  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
//  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
//	acc_x_cal /= 2000;
//	acc_y_cal /= 2000;
//	acc_z_cal = 0;
	gyro_x_cal = -15;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal = -257;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal = -105;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
	acc_x_cal = 276;
	acc_y_cal = -194;
	acc_z_cal = 0;
	

}
void Calculate_Angle(void){
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
	acc_x -= acc_x_cal;
	acc_y -= acc_y_cal;
	acc_z -= acc_z_cal;
	
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
}

//Clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  { 
  }
}
