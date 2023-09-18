/**
  ******************************************************************************
  * @file    GPIO_Toggle\main.c
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    26-April-2018
  * @brief   This file contains the main function for GPIO Toggle example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
//#include "stm8s_it.h"    /* SDCC patch: required by SDCC for interrupts */

/**
  * @addtogroup GPIO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */

/* automatically use built-in LED for known nucleo boards */
#if defined(STM8S_NUCLEO_208RB) || defined(STM8S_NUCLEO_207K8) 
#define LED_GPIO_PORT  (GPIOC)
#define LED_GPIO_PINS  (GPIO_PIN_5)
#elif defined(STM8S103)
/* for STM8S103F3 breakout board. building with GPIOG would result in failure (chip 
 * does not have that GPIO peripheral) */
#define LED_GPIO_PORT  (GPIOB)
#define LED_GPIO_PINS  (GPIO_PIN_5)	
#else 
#define LED_GPIO_PORT  (GPIOB)
#define LED_GPIO_PINS  (GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#endif

#define RCLK_PIN GPIO_PIN_7
#define PW_ON GPIO_PIN_3
#define BTN2_H GPIO_PIN_3   //upper btn
#define BTN1_M GPIO_PIN_4   //lower btn
#define BAT_SENSE GPIO_PIN_2

#define CHARHGING GPIO_PIN_5
#define CHARHGED GPIO_PIN_6
#define SENS_BTN_PORT (GPIOD)
//#define ACCELEROMETR_ADDR0 0x30
#define Animations_amount 3
#define BRIGHTNESS_EEPROM_ADDR 0x04003
#define PERIOD_EEPROM_ADDR 0x04002
#define ANIMATION_EEPROM_ADDR 0x04001
#define BOOL_EEPROM_ADDR 0x04000
#define STEP_DELAY 4
#define CALIBRATION_MODE 0
#define RTC_CALIBER_VALUE 0x00
#define RTC 2 // 0 M41T00; 1 - M41T81; 2 - M41T81S;
#define RTC_ADDR 0xD0
#define ACCELEROMETR_ADDR1 0x32

#if RTC >= 1
#define RTC_CALIBER_REG_ADDR 0x08
#else
#define RTC_CALIBER_REG_ADDR 0x07
#endif

//#define Bat_charge_icons_with_side_bars 0
#define Brightness 255                  // 1 - 255
#define TIME_SHOWING_DURATION 1000      // ms       2000 1400 1000 700 500 400 300
#define Charge_showing_duration 2000    // ms
#define Settings_showing_time 10000     // ms
#define ANIMATION 1
#define Enable_seconds 1
#define Enable_accelerometr 1
#define Long_btn_press 1000             // ms
#define TimErr 1000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//uint16_t grid_l = 0b0000000000001000;
//uint16_t grid_r = 0b0000000010000000;


// typedef enum {
//   Brightness_8 = (uint8_t)0xFF,
//   Brightness_7 = (uint8_t)0x7F,
//   Brightness_6 = (uint8_t)0x3F,
//   Brightness_5 = (uint8_t)0x1F,
//   Brightness_4 = (uint8_t)0x0F,
//   Brightness_3 = (uint8_t)0x07,
//   Brightness_2 = (uint8_t)0x03,
//   Brightness_1 = (uint8_t)0x01,
// } Brightnes_typeDef;
bool brightness_controll_lamps_on = TRUE;
bool low_bat_indication_blink_reserved = TRUE;
bool low_bat_indication_blink = FALSE;
bool sleep_flag = FALSE;
bool open_animation_is_showing = FALSE;
bool long_btn_press_occured = FALSE;
bool time_shoing_started_time_count = FALSE;
bool time_shwing_btn_mode = FALSE;
bool quick_increment_by_btn_long_press = FALSE;
bool show_today_step = TRUE;
bool awu_mode_no_wake_up = FALSE;
bool arm_rotate_threshold_passed;
bool time_scenario;
bool step_showing_indication_done = FALSE;
bool awu_reset_allow = FALSE;

uint8_t btn_was_pressed = 0;  //  0 - no btn, 1 - btn1_M, 2 - btn2_H

uint8_t day = 1;
uint8_t errors_count = 0;
uint32_t miliseconds;
uint32_t btn_pressed_time_start;
uint32_t time_showing_started;
uint32_t settings_showing_started;
uint16_t actual_RTC_milliceconds;
uint8_t lamp_L_symb_num = 0;
uint8_t lamp_R_symb_num = 0;
uint16_t animation_frame_r;
uint16_t animation_frame_l;
uint8_t TxBuffer[9];
uint8_t RxBuffer[8];
uint16_t time_showing_durations[] = {400, 500, 600, 700, 900, 1100, 1300, 1600, 2000};
int8_t X_acc[9];
int8_t Y_acc[9];
//int8_t Z_acc[9];
uint16_t up_down_transition;
uint16_t down_up_transition;
uint8_t step_delay = STEP_DELAY;
uint32_t step_counter_this_day = 0;
uint32_t step_counter_previous_day = 0;
uint8_t awu_end_of_day_counter;

uint8_t seg_dot_on_lamp = 0;  // 1 - left lamp 2 - right lamp 3 both lamps
uint8_t showing_data = 0;     // 0 nothing, 1 time, 2 battary charge, 3 settings, 4 open animation, 5 close animation, 6 error message, 7 step counter
uint8_t showing_setting = 0;  // 0 hour, 1 minute, 2 seconds, "Я" 3 brightness, "С" 4 enable seconds, "П" 5 time showing period,
// "animation itself" 6 Animation, "arrows" 7 Accelerometr, "ш" 8 Step counter
uint8_t error_register = 0;    // 0 no error, 1 low clock bat, 2 osc fail detected, 3 eeprom [in factRTC flag register copy]
bool enable_seconds = Enable_seconds;             // TO EEPROM bool
bool enable_accelerometr = Enable_accelerometr;   // TO EEPROM bool
bool enable_step_counter = 0;   // TO EEPROM bool
uint8_t Animation = ANIMATION;        // TO EEPROM byte
uint16_t Time_showing_duration = TIME_SHOWING_DURATION;
uint8_t time_showing_duration_level;  // 2000 1400 1000 700 500 400 300 // TO EEPROM byte
uint8_t brightness = Brightness;      // TO EEPROM byte
uint8_t brightness_level;     // 1 2 4 8 16 32 64 128 255

volatile uint16_t seg_dot = 0b0000000001000000;
uint16_t alphabet_iv3_l[] = {
  0b0111001100101000,   //  0
  0b0000010100101000,   //  1
  0b0110101100001000,   //  2
  0b0100111000101000,   //  3
  0b0001100100101000,   //  4
  0b0101101000101000,   //  5
  0b0111101000101000,   //  6
  0b0000011000011000,   //  7
  0b0111101100101000,   //  8
  0b0101101100101000,   //  9     
  0b0111111101111000,   //  full  10
  0b0000000000001000,   //  none  11
  0b0011100100101000,   //  H     12
  0b0111101000001000,   //  E     13
  0b0111111000101000,   //  B     14
  0b0001101100111000,   //  Я     15
  0b0111001000001000,   //  C     16
  0b0011001100101000,   //  П     17
  0b0000110100111000,   //  A     18
  0b0100001000001000,   //  sett1 19
  0b0010000100001000,   //  sett2 20
  0b0001000000101000,   //  sett3 21
  0b0010010100011000,   //  arrws 22
  0b0011010000011000,   //  V     23
  0b0110000000111000    //  ш     24
  //0b0000000000000000    //  
};
uint16_t alphabet_iv3_r[] = {
  //0123456789
  0b0111001110010000,   //  0
  0b0100001010100000,   //  1
  0b0111010110000000,   //  2
  0b0001011110100000,   //  3
  0b0100011010010000,   //  4
  0b0001011110010000,   //  5
  0b0011011110010000,   //  6
  0b0000100110100000,   //  7
  0b0111011110010000,   //  8
  0b0101011110010000,   //  9
  0b0111111111110000,   //  full  10
  0b0000000010000000,   //  none  11
  0b0110011010010000,   //  H     12
  0b0100001010000000,   //  I     13
  0b0001010110000000,   //  b3    14  bat >= 75
  0b0001010010000000,   //  b2    15  40 >= bat > 75
  0b0001000010000000,   //  b1    16  40 >  bat > 25
  0b0001000110000000,   //  sett1 17
  0b0110000010000000,   //  sett2 18
  0b0000001010010000,   //  sett2 19
  0b0010010010000000    //  r     20
  //0b0110001010010000,   //  ||    16  bat <=25
};
uint16_t animation_array[1][12] = {
  {0b0000000100001000, 0b0000011100001000, 0b0001111000001000,  //left lamp
  0b0001100000111000, 0b0110000001111000, 0b0110000000001000,   //left lamp
  0b0010000010000000, 0b0011100010000000, 0b0001111010000000,   //right lamp
  0b0000011011110000, 0b0100000111110000, 0b0100000110000000}   //right lamp
  //{0, 0, 0, 0, 0, 0}
};
const uint16_t seg_order_for_random_const[9] = {0x4000, 0x2000, 0x1000, 0x0800, 0x0400, 0x0200, 0x0100, 0x0020, 0x0010};
uint16_t seg_order_for_random_reserved[9];
bool lamp_R_switch = FALSE;
/* Private function prototypes -----------------------------------------------*/
void Delay (uint32_t nCount);
void Delay_ms(uint32_t delay);
void CLK_SYSCLKConfig(uint8_t CLK_Prescaler);
void TIM2_ITConfig(uint8_t TIM2_IT, FunctionalState NewState);
void TIM2_TimeBaseInit( uint8_t TIM2_Prescaler, uint16_t TIM2_Period);
void TIM2_Cmd(FunctionalState NewState);
void TIM2_ClearFlag(uint16_t TIM2_FLAG);
void I2C_Init(/*uint8_t OutputClockFreq_1_4,*/ uint8_t Ack, uint8_t InputClockFrequencyMHz );
void I2C_Send_data( uint8_t Addr, uint8_t countData, uint8_t* dataBuffer);
void I2C_Read_three_more_byte( uint8_t Addr, uint8_t countData, uint8_t* dataBuffer, uint8_t readFrom);
void I2C_Bus_init(uint8_t Addr, uint8_t direction, bool initial_check_for_busy);
void Swap_segments(uint8_t seg1, uint8_t seg2);
void Mix_segments();
void Show_easy_animation(uint8_t animation, uint8_t frame);
void Set_sleep_conditions();
//void Awu_end_of_the_day_check_for_reset();

void get_settings_from_eeprom();
void set_settings_to_eeprom();
void get_time();
void set_time();
void show_open_animation(uint8_t);
void show_close_animation(uint8_t);
void show_error(uint8_t error);
void show_settings_animation();
void wake_up_by_accelerometr();
//void Set_Brightness(Brightnes_typeDef brightness);
//void spi_interrupt(void) INTERRUPT(10);
//extern void INTERRUPT_HANDLER(SPI_IRQHandler, 10);
/* Private functions ---------------------------------------------------------*/

//void spi_interrupt(viod) INTERRUPT(10){

//}

/* Public functions ----------------------------------------------------------*/
volatile uint8_t seconds;
volatile uint8_t ten_seconds;
volatile uint8_t minutes;
volatile uint8_t ten_minutes;
volatile uint8_t hours;
volatile uint8_t ten_hours;
//uint8_t btn_touch;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  //-----------------Clock config-------------------------------------------------------------//
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!IF CHANGED, CHANGE THE I2C INIT PARAMS!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  /* Clear High speed internal clock prescaler */
  CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
  /* Set High speed internal clock prescaler */
  CLK->CKDIVR |= (uint8_t)0x08;                 /*!< High speed internal clock prescaler: 2 */
  CLK_SYSCLKConfig(0x80);                       /*!< CPU clock division factors 1 */
  CLK->ICKR |= CLK_ICKR_SWUAH;                  // MVR disabled in halt mode
  CLK->ICKR &= ~CLK_ICKR_FHWU;                  // fast wake-up from halt mode disabled
  /*------------------------------------------------------------------------------------------*/
  if (brightness == 0)
  {
    brightness = 1;
  }
  /* Initialize I/Os in Output Mode */
  //GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);    //SPI_SCK
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);    //SPI_MOSI
  GPIO_Init(GPIOC, RCLK_PIN, GPIO_MODE_OUT_PP_LOW_FAST);      //shift reg rclk pin init
  GPIO_Init(GPIOC, PW_ON, GPIO_MODE_OUT_PP_LOW_SLOW);         //PW_ON pin init
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_HIZ_FAST);    //I2C_SCL
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);    //I2C_SDA
  //GPIO_ExternalPullUpConfig(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, ENABLE);  //pull-up on i2c bus
  
  //GPIO_ExternalPullUpConfig(GPIOB, GPIO_PIN_5, ENABLE);
  //buttons and sensing pins init
  GPIO_Init(SENS_BTN_PORT, BTN1_M, GPIO_MODE_IN_PU_IT); 
  GPIO_Init(SENS_BTN_PORT, BTN2_H, GPIO_MODE_IN_PU_IT); 
  GPIO_Init(SENS_BTN_PORT, BAT_SENSE, GPIO_MODE_IN_FL_NO_IT); 
  //EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_FALL); !!!!!!!!!!same row below
  EXTI->CR1 &= (uint8_t)(~EXTI_CR1_PDIS);
  EXTI->CR1 |= (uint8_t)((uint8_t)(0x03) << 6);
  //GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT); 
  //SPI init
  SPI_Init(SPI_FIRSTBIT_LSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_1LINE_TX, SPI_NSS_SOFT, 7);
  //SPI_ITConfig(SPI_IT_TXE, ENABLE);
  SPI_Cmd(ENABLE);
  //I2C init
  I2C_Init(/*4,*/I2C_ACK_CURR,8);//400khz out, 8Mhz in
  //ADC init
  ADC1_Cmd(ENABLE);
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D2, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL3, DISABLE);
  //ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_2, ADC1_PRESSEL_FCPU_D18, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL2, DISABLE);
  ADC1_ClearFlag(ADC1_FLAG_EOC);
  //TIM4 init indication timer
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, brightness);
  TIM4_ARRPreloadConfig(ENABLE);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
  //TIM2 init milliceconds timer
  TIM2_TimeBaseInit(0x03, 999); //prescaler /8, count to 999 in order to activate interrupt every millisecond
  TIM2_ITConfig(0x01, ENABLE);  //TIM2_IT_UPDATE
  TIM2_ClearFlag(0x0001);       //TIM2_FLAG_UPDATE
  TIM2_Cmd(ENABLE);
  //AWU init
  AWU_Init(AWU_TIMEBASE_64MS);

  enableInterrupts();


  TxBuffer[0] = 0x0C; //halt bit regester address
  TxBuffer[1] = 0;
  TxBuffer[2] = 0;
  TxBuffer[3] = 0;
  I2C_Send_data(RTC_ADDR, 4, TxBuffer);

  #if RTC >= 1 && CALIBRATION_MODE == 1
  //#if CALIBRATION_MODE == 1
  TxBuffer[0] = RTC_CALIBER_REG_ADDR; //Calibration register addr
  TxBuffer[1] = 0x80; //Out bit = 1
  TxBuffer[2] = 0x00; //09h
  TxBuffer[3] = 0x40; //0Ah SQW bit = 1
  TxBuffer[4] = 0x00; //0Bh
  TxBuffer[5] = 0x00; //0Ch HT bit = 0
  I2C_Send_data(RTC_ADDR, 6, TxBuffer);
  TxBuffer[0] = 0x13; //SQW register addr
  TxBuffer[1] = 0xF0; //Out freq = 1Hz
  I2C_Send_data(RTC_ADDR, 2, TxBuffer);
  #elif RTC >= 1 && CALIBRATION_MODE == 0
  TxBuffer[0] = RTC_CALIBER_REG_ADDR; //Calibration register addr
  TxBuffer[1] = RTC_CALIBER_VALUE; 
  TxBuffer[2] = 0x00; //09h
  TxBuffer[3] = 0x00; //0Ah SQW bit = 0
  TxBuffer[4] = 0x00; //0Bh
  TxBuffer[5] = 0x00; //0Ch HT bit = 0
  I2C_Send_data(RTC_ADDR, 6, TxBuffer);
  #endif

  #if RTC == 0 && CALIBRATION_MODE == 1
  TxBuffer[0] = RTC_CALIBER_REG_ADDR; //Calibration register addr
  TxBuffer[1] = 0xC0; //Out bit and Freq Test = 1
  I2C_Send_data(RTC_ADDR, 2, TxBuffer);
  #elif RTC == 0 && CALIBRATION_MODE == 0
  TxBuffer[0] = RTC_CALIBER_REG_ADDR; //Calibration register addr
  TxBuffer[1] = RTC_CALIBER_VALUE; 
  I2C_Send_data(RTC_ADDR, 2, TxBuffer);
  #endif


  I2C_Read_three_more_byte(RTC_ADDR, 8, RxBuffer, 0);
  #if RTC >= 1
  if (RxBuffer[1] & 0x80)
  #else
  if (RxBuffer[0] & 0x80)
  #endif
  {
    #if RTC >= 1
    TxBuffer[0] = 0;
    TxBuffer[1] = 0;
    TxBuffer[2] = RxBuffer[1] & ~0x80;  
    I2C_Send_data(RTC_ADDR, 3, TxBuffer);
    #else
    TxBuffer[0] = 0;
    TxBuffer[1] = RxBuffer[0] & ~0x80;  
    I2C_Send_data(RTC_ADDR, 2, TxBuffer);
    #endif 

  }
  #if RTC >= 1
  if ((RxBuffer[2] >> 4) > 5 || (RxBuffer[2] & 0x0F) > 9 || (RxBuffer[3] & 0x0F) > 9 || (RxBuffer[3] >> 4) > 2)
  #else
  if ((RxBuffer[1] >> 4) > 5 || (RxBuffer[1] & 0x0F) > 9 || (RxBuffer[2] & 0x0F) > 9 || (RxBuffer[2] >> 4) > 2)
  #endif
  {
    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x00; 
    TxBuffer[2] = 0x00; 
    TxBuffer[3] = 0x00; 
    TxBuffer[4] = 0x01; 
    TxBuffer[5] = 0x01; 
    TxBuffer[6] = 0x01; 
    TxBuffer[7] = 0x00; 
    I2C_Send_data(RTC_ADDR, 8, TxBuffer);
  }


  #if CALIBRATION_MODE == 0

  //accelerometr init
  I2C_Read_three_more_byte(ACCELEROMETR_ADDR1, 1, RxBuffer, 0x9E);
  TxBuffer[0] = 0x9E; //CTRL_REG0 addr |= 0x80 in order to enable multiple reg transfer
  TxBuffer[1] = 0x10; //CTRL_REG0       pull-up connected to SDO/SA0 pin (Default value: 00010000)
  TxBuffer[2] = 0x00; //TEMP_CFG_REG    T disabled, ADC disabled
  TxBuffer[3] = 0x3F; //CTRL_REG1       0 0 1 1 HR / Normal / Low-power mode (25 Hz)    1: low-power mode   all axis enable
  TxBuffer[4] = 0x00; //CTRL_REG2       Write 00h into CTRL_REG2 // High-pass filter disabled
  TxBuffer[5] = 0x00; //CTRL_REG3       //all ints disabled
  TxBuffer[6] = 0x10; //CTRL_REG4       0: continuous update, 0: Data LSB @ lower address, 01: ±4 g, 0: high-resolution disabled, 00: self-test disabled, -
  TxBuffer[7] = 0x00; //CTRL_REG5       -, 0: FIFO disable, all ints disabled
  TxBuffer[8] = 0x00; //CTRL_REG6       //all ints disabled
  //I2C_Send_data(ACCELEROMETR_ADDR0, 9, TxBuffer);
  I2C_Send_data(ACCELEROMETR_ADDR1, 9, TxBuffer);

  //pw on
  get_settings_from_eeprom();
  AWU_Cmd(enable_accelerometr | enable_step_counter);

  GPIO_WriteHigh(GPIOC, PW_ON);

  lamp_L_symb_num = 12;   //H
  lamp_R_symb_num = 13;   //I
  Delay_ms(1300);
  lamp_L_symb_num = 23;   //V     //-------------------VERSION-------------------//
  lamp_R_symb_num = 20;   //r
  Delay_ms(700);
  lamp_L_symb_num = 1;    //1.  
  lamp_R_symb_num = 4;    //4
  seg_dot_on_lamp = 1;
  Delay_ms(1300);
  showing_data = 0;
  //ADC1_StartConversion();
  #endif

  while (1)
  {
    if (showing_data == 3)//-------------------SETTINGS-------------------//
    {
      if (!time_shoing_started_time_count)
      {
        settings_showing_started = miliseconds;
        time_shoing_started_time_count = TRUE;

      }

      actual_RTC_milliceconds = (miliseconds - settings_showing_started) % 1000;

      if (showing_setting == 0)//-------------------HOURS-------------------//
      {
        lamp_L_symb_num = ten_hours;
        lamp_R_symb_num = hours;
        if (actual_RTC_milliceconds < 500)  //blinking dot
        {
          seg_dot_on_lamp = 2;
        }else
        {
          seg_dot_on_lamp = 0;
        }

        if (btn_was_pressed == 2)
        {
          showing_setting = 1;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          hours++;
          if (hours >= 10)
          {
            hours = 0;
            ten_hours++;
          }
          if (ten_hours >= 2 && hours > 3)
          {
            ten_hours = 0;
            hours = 0;
          }  
        }
      }
      if (showing_setting == 1)//-------------------MINUTES-------------------//
      {
        lamp_L_symb_num = ten_minutes;
        lamp_R_symb_num = minutes;
        if (actual_RTC_milliceconds < 500)  // blinking dot
        {
          seg_dot_on_lamp = 1;
        }else
        {
          seg_dot_on_lamp = 0;
        }

        if (btn_was_pressed == 2)
        {
          showing_setting = 2;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          minutes++;
          if (minutes >= 10)
          {
            ten_minutes++;
            minutes = 0;
          }
          if (ten_minutes >= 6)
          {
            ten_minutes = 0;
          }
        }
      }
      if (showing_setting == 2)//-------------------SECONDS-------------------//
      {
        lamp_L_symb_num = ten_seconds;
        lamp_R_symb_num = seconds;
        
        if (actual_RTC_milliceconds < 500)        // blinking dot
        {
          seg_dot_on_lamp = 3;
        }else
        {
          seg_dot_on_lamp = 0;
        }
       
        if (btn_was_pressed == 2)
        {
          showing_setting = 3;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          // set 0 sec and immediately set
          set_time();
          seconds = 0;
          ten_seconds = 0; 
          seg_dot_on_lamp = 0;
        }
      }
      if (showing_setting == 3)//-------------------BRIGHTNESS-------------------//
      {
        lamp_L_symb_num = 15;
        lamp_R_symb_num = brightness_level;
        seg_dot_on_lamp = 0;
        if (brightness == 255)
        {
          brightness_level = 9;
        }else
        {
          brightness_level = 0;
          for (uint8_t i = brightness; i > 0; i = i >> 1)
          {
            brightness_level++;
          }
        }       
        if (btn_was_pressed == 2)
        {
          showing_setting = 4;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          brightness_level++;
          if (brightness_level > 9)
          {
            brightness_level = 1;
          }
          
          if (brightness_level < 9)
          {
            brightness = 1 << (brightness_level - 1);
          }else
          {
            brightness = 255;
          }
        }
      }
      if (showing_setting == 4)//-------------------SECONDS ENABLE-------------------//
      {
        lamp_L_symb_num = 16;   // "C"
        lamp_R_symb_num = enable_seconds;
        if (btn_was_pressed == 2)
        {
          showing_setting = 5;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          enable_seconds = !enable_seconds;
        }
      }
      if (showing_setting == 5)//-------------------SHOWING PERIOD-------------------//
      {
        lamp_L_symb_num = 17;   // "П"
        lamp_R_symb_num = time_showing_duration_level + 1;
        if (btn_was_pressed == 2)
        {
          showing_setting = 6;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          time_showing_duration_level++;
          if (time_showing_duration_level > 8)
          {
            time_showing_duration_level = 0;
          }
          Time_showing_duration = time_showing_durations[time_showing_duration_level];
          //time_shoing_started_time_count = FALSE;
          //showing_data = 1;
          //settings_mode = TRUE;
        }
        
      }
      if (showing_setting == 6)//-------------------ANIMATION-------------------//
      {
        lamp_L_symb_num = 18;   //animation_array[Animation][miliseconds % 420 / 70];
        lamp_R_symb_num = Animation;
        if (btn_was_pressed == 2)
        {
          showing_setting = 7;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;  // clear btn pressed
          Animation++;
          if (Animation > Animations_amount)
          {
            Animation = 0;
          }
        }
      }
      if (showing_setting == 7)//-------------------ACCELEROMETR-------------------//
      {
        lamp_L_symb_num = 22;
        lamp_R_symb_num = enable_accelerometr;
        if (btn_was_pressed == 2)
        {
          showing_setting = 8;  // next setting (step counter)
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          enable_accelerometr = !enable_accelerometr;
          AWU_Cmd(enable_accelerometr | enable_step_counter);
          btn_was_pressed = 0;  // clear btn pressed
        }
      }
      if (showing_setting == 8)//-------------------STEP COUNTER-------------------//
      {
        lamp_L_symb_num = 24;
        lamp_R_symb_num = enable_step_counter;
        if (btn_was_pressed == 2)
        {
          showing_setting = 0;  // next setting
          btn_was_pressed = 0;  // clear btn pressed
        }
        if (btn_was_pressed == 1)
        {
          enable_step_counter = !enable_step_counter;
          AWU_Cmd(enable_accelerometr | enable_step_counter);
          btn_was_pressed = 0;  // clear btn pressed
        }
      }
      if (miliseconds - settings_showing_started >= Settings_showing_time)
      {
        //seg_dot_on_lamp = 0;
        //showing_data = 5;
        
        //settings_mode = FALSE;
        set_time();
        set_settings_to_eeprom();
        //time_shoing_started_time_count = FALSE;
        Set_sleep_conditions();
      }
    }
    
    //low_bat_indication_blink = FALSE;
    if (showing_data == 2)//-------------------CHARGE-------------------//
    {
      if (!time_shoing_started_time_count)
      {
        time_showing_started = miliseconds;
        time_shoing_started_time_count = TRUE;
      }
      uint16_t bat_voltage = 0;
      for (uint8_t j = 0; j < 10; j++)
      {
        ADC1_ClearFlag(ADC1_FLAG_EOC);
        ADC1_StartConversion();
        while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC));
        bat_voltage += ADC1_GetConversionValue();
        //bat_charge_percent = (bat_voltage - 549) / 2.2;
      }
      lamp_L_symb_num = 18;       //  "A"
      seg_dot_on_lamp = 0;
      if (bat_voltage <= 5200)
      {
        lamp_R_symb_num = 16;
        low_bat_indication_blink = TRUE;
      }
      if (bat_voltage > 5201 & bat_voltage <= 5500)
      {
        lamp_R_symb_num = 16;
        low_bat_indication_blink = FALSE;
      }
      if (bat_voltage > 5501 & bat_voltage <= 6050)
      {
        lamp_R_symb_num = 15;
        low_bat_indication_blink = FALSE;
      }
      if (bat_voltage > 6051)
      {
        lamp_R_symb_num = 14;
        low_bat_indication_blink = FALSE;
      }
      if (miliseconds - time_showing_started >= Charge_showing_duration)               // end of charge showing
        {
          btn_was_pressed = 0;
          time_shoing_started_time_count = FALSE;
          // if (enable_step_counter)
          // {
          //   showing_data = 7;
          // }else{
          if (error_register)
          {
            showing_data = 6;
          }else
          {
            showing_data = 5;
          }
          // }
        }
    }
    
    if(miliseconds % 400 > 200){
      low_bat_indication_blink_reserved = TRUE;
    }else{
      low_bat_indication_blink_reserved = FALSE;
    }

    if (showing_data == 1)//-------------------TIME-------------------//            
    {
      get_time();
      if (!time_shoing_started_time_count)
      {
        time_showing_started = miliseconds;
        time_shoing_started_time_count = TRUE;
      }
      uint16_t already_showing = miliseconds - time_showing_started;
      if (!time_shwing_btn_mode)
      {
        if (already_showing < Time_showing_duration)                                                        // showing hours
        {
          lamp_R_symb_num = hours;
          lamp_L_symb_num = ten_hours;
          seg_dot_on_lamp = 2;
        }
        if (already_showing >= Time_showing_duration && already_showing < 2 * Time_showing_duration)        // showing minutes
        {
          lamp_R_symb_num = minutes;
          lamp_L_symb_num = ten_minutes;
          seg_dot_on_lamp = 1;
        }
        if (already_showing >= 2 * Time_showing_duration && already_showing < 4 * Time_showing_duration)
        { 
          if (enable_seconds)  // showing seconds if enabled
          {
            lamp_R_symb_num = seconds;
            lamp_L_symb_num = ten_seconds;
            if (actual_RTC_milliceconds < 500)        // blinking dot
            {
              seg_dot_on_lamp = 3;
            }else
            {
              seg_dot_on_lamp = 0;
            }
          }
          else
          {
            already_showing = 4 * Time_showing_duration;
          }
        }
        
        
        // else
        // { // show charge
        //   if (already_showing >= 2 * Time_showing_duration)
        //   {
        //     time_shoing_started_time_count = FALSE;
        //     seg_dot_on_lamp = 0;
        //     showing_data = 2;
        //   }
        // }
        
        if (already_showing >= 4 * Time_showing_duration) // end of time showing
        {
          seg_dot_on_lamp = 0;
          //time_shoing_started_time_count = FALSE;
          //showing_data = 5;
          Set_sleep_conditions();
        }
      }else
      {
        if (btn_was_pressed == 1 && miliseconds - time_showing_started < 3 * Time_showing_duration)         // showing hours
        {
          lamp_R_symb_num = hours;
          lamp_L_symb_num = ten_hours;
          seg_dot_on_lamp = 2;
        }

        if (btn_was_pressed == 2 && miliseconds - time_showing_started < 3 * Time_showing_duration)        // showing minutes
        {
          lamp_R_symb_num = minutes;
          lamp_L_symb_num = ten_minutes;
          seg_dot_on_lamp = 1;
        }
        if (miliseconds - time_showing_started >= 3 * Time_showing_duration)                               // end of time showing
        {
          btn_was_pressed = 0;
          seg_dot_on_lamp = 0;
          //time_shoing_started_time_count = FALSE;
          //if (settings_mode)
          //showing_data = 5;
          Set_sleep_conditions();
        }
      }
    }
     
    if (showing_data == 5)
    {
        if (Animation)
        {
          show_close_animation(Animation);
        }
        showing_data = 0;
    }
    
    if (showing_data == 6)//-------------------ERRORS-------------------//
    {
      #if (RTC == 2)

      uint8_t errno;
      for (uint8_t i = 0b10000000; i != 0; i = i >> 1){
        if ((error_register & i) == 0x10)
        {
          error_register &= ~i;  //removing error
          errno = 1;     
        }
        if ((error_register & i) == 0x04)
        {
          error_register &= ~i;  //removing error
          //errno = 2; 
          errors_count++;
        } 
        if (errno)
        {
          show_error(errno);
        }
        errno = 0;
      }
      show_error(2);
      if(!error_register)showing_data = 5;

      #else
      error_register = 0;
      showing_data = 5;
      #endif
    }
    
    if (showing_data == 7)//-------------------STEPS-------------------//
    {
      //uint8_t ten_step;
      //uint8_t step;
      uint32_t step_to_show;
      if (!time_shoing_started_time_count)
      {
        time_showing_started = miliseconds;
        time_shoing_started_time_count = TRUE;
      }
      if (!step_showing_indication_done)
      {
        lamp_L_symb_num = 24;//ш
        lamp_R_symb_num = 20;//г
        seg_dot_on_lamp = 0;
      } else
      {
        if (btn_was_pressed == 1)
        {
          btn_was_pressed = 0;
          show_today_step = !show_today_step;
        }
        
        //seg_dot_on_lamp = 0;
        if (show_today_step)
        {
          step_to_show = step_counter_this_day;
        }else
        {
          step_to_show = step_counter_previous_day;
          //seg_dot_on_lamp |= 2;                     //upper dot if previous day;
        }
        if (step_to_show >= 10000)
        {
          lamp_L_symb_num = step_to_show / 10000;
          lamp_R_symb_num = step_to_show % 10000 / 1000;
          if (show_today_step)
          {
            seg_dot_on_lamp = 0;
          }
          else
          {
            seg_dot_on_lamp = 2;
          }
          //seg_dot_on_lamp = 0;
        }else
        {
          lamp_L_symb_num = step_to_show / 1000;
          lamp_R_symb_num = step_to_show % 1000 / 100;
          if (show_today_step)
          {
            seg_dot_on_lamp = 1;
          }
          else
          {
            seg_dot_on_lamp = 3;
          }
          //seg_dot_on_lamp |= 1;
        }
      }
      
      if (miliseconds - time_showing_started >= 1000)
      {
        step_showing_indication_done = TRUE;
      }
      

      if (miliseconds - time_showing_started >= 3 * Charge_showing_duration)               // end of steps showing
        {
          btn_was_pressed = 0;
          Set_sleep_conditions();
          //time_shoing_started_time_count = FALSE;
          step_showing_indication_done = FALSE;
          // if (settings_mode)
          // {
          //   showing_data = 3;
          // }else{
            //showing_data = 5;
          // }
        }
    }
    

    if (showing_data == 0)
    {
      if (!sleep_flag)
      {
        long_btn_press_occured = FALSE;
        GPIO_WriteLow(GPIOC, PW_ON);
        SPI_SendData(0);
        I2C_Cmd(DISABLE);
        nop();
        nop();
        nop();
        SPI_SendData(0);
        TIM4_Cmd(DISABLE);
        TIM2_Cmd(DISABLE);
        ADC1_Cmd(DISABLE);
        SPI_Cmd(DISABLE);
        nop();
        
        Delay(1700);//flash on pw off deducktion
        GPIO_WriteHigh(GPIOC, RCLK_PIN);
        GPIO_WriteLow(GPIOC, RCLK_PIN);
        sleep_flag = TRUE;
        //CFG_GCR->AL = 0;
        //CFG->GCR = 0x00;
        //showing_data = 4;        // show_open animation
      }
      CLK->ICKR |= CLK_ICKR_SWUAH;                  // MVR disabled in halt modeshow_open_animation
      CLK->ICKR &= ~CLK_ICKR_FHWU;                  // fast wake-up from halt mode disabled
      halt();
    }
    if (sleep_flag && !awu_mode_no_wake_up)//-----------------------------------------waking up with lamps
    {
      TIM4_Cmd(ENABLE);
      TIM2_Cmd(ENABLE);
      CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);

      /* Set High speed internal clock prescaler */
      CLK->CKDIVR |= (uint8_t)0x08;                 /*!< High speed internal clock prescaler: 2 */
      CLK_SYSCLKConfig(0x80);                       /*!< CPU clock division factors 1 */

      sleep_flag = FALSE;
      GPIO_WriteHigh(GPIOC, PW_ON);
      SPI_Cmd(ENABLE);
      ADC1_Cmd(ENABLE);
      I2C_Cmd(ENABLE);
      error_register = 0x00;

      #if (RTC == 2 || RTC == 1)
      I2C_Read_three_more_byte(RTC_ADDR, 3, RxBuffer, 0x0f);    //reading RTC flag registers
      error_register = RxBuffer[0];    
      //resetting RTC falgs
      TxBuffer[0] = 0x0C; //halt bit regester address
      TxBuffer[1] = 0;
      TxBuffer[2] = 0;
      TxBuffer[3] = 0;
      TxBuffer[4] = 0;
      I2C_Send_data(RTC_ADDR, 5, TxBuffer);
      #endif
      if (Animation == 2)
      {
        for (uint8_t i = 0; i < 9; i++)
        {
          seg_order_for_random_reserved[i] = seg_order_for_random_const[i];
        }
        Mix_segments();
      }
    }

    if (showing_data == 4)
    {
      if (Animation)
      {
        show_open_animation(Animation);
      }
      if (time_scenario)
      {
        showing_data = 1;   //time
      }else
      {
        showing_data = 2;   //charge
      }
    }
    
    //-----long btn press detection-----//
    if ((!GPIO_ReadInputPin(SENS_BTN_PORT, BTN1_M) || !GPIO_ReadInputPin(SENS_BTN_PORT, BTN2_H)) && 
    !long_btn_press_occured && (miliseconds - btn_pressed_time_start >= Long_btn_press))
    {
      long_btn_press_occured = TRUE;
    }

    if (long_btn_press_occured)
    {
      if (showing_data == 3)
      {
        quick_increment_by_btn_long_press = TRUE;
      }
      if (showing_data != 3)
      {
        show_settings_animation();
        showing_data = 3;                  //if long press go to settings
        //settings_mode = TRUE;
        showing_setting = 0;
        get_time();
      }
      settings_showing_started = miliseconds;
      time_shoing_started_time_count = FALSE;       //reset time shoing time
      long_btn_press_occured = FALSE;   //long_btn_press_occured clear flag
    }
  }
}

INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  uint8_t auf_clear;
  auf_clear = AWU->CSR;
  awu_mode_no_wake_up = TRUE;
  I2C_Cmd(ENABLE);
  I2C_Read_three_more_byte(ACCELEROMETR_ADDR1, 5, RxBuffer, 0xA9);
  I2C_Cmd(DISABLE);
  int8_t Y_local_min = 0;
  int8_t Y_local_max = 0;
  

  up_down_transition = up_down_transition >> 1;
  down_up_transition = down_up_transition >> 1;
  for (uint8_t i = 0; i < 8; i++)
  {
    X_acc[i+1] = X_acc[i];
    Y_acc[i+1] = Y_acc[i];
    //Z_acc[i+1] = Z_acc[i];
    if(Y_acc[i+1] > Y_local_max) Y_local_max = Y_acc[i+1];
    if(Y_acc[i+1] < Y_local_min) Y_local_min = Y_acc[i+1];
  }
  X_acc[0] = RxBuffer[0];
  Y_acc[0] = RxBuffer[2];
  //Z_acc[0] = RxBuffer[4];
  if(Y_acc[0] > Y_local_max) Y_local_max = Y_acc[0];
  if(Y_acc[0] < Y_local_min) Y_local_min = Y_acc[0];

  if (enable_step_counter)
  {
    if (!step_delay)
    {
      step_delay--;
    }else if ((X_acc[0] - X_acc[4]) + (Y_acc[4] - Y_acc[0]) > 17)
    {
      step_delay = STEP_DELAY;
      step_counter_this_day++;
    }

    // if (/*awu_end_of_day_counter && !*/awu_reset_allow)
    // /*{
    //   awu_end_of_day_counter--;
    // }else*/
    // {
    //   //awu_end_of_day_counter = 1350000;
    //   step_counter_previous_day = step_counter_this_day;
    //   step_counter_this_day = 0;
    //   awu_reset_allow = FALSE; 
    // }
    //Awu_end_of_the_day_check_for_reset();
  }

  if (enable_accelerometr)
  {
    if(Y_acc[2] - Y_acc[0] > 45)up_down_transition |= 0x0100;
    if(Y_acc[0] - Y_acc[2] > 45)down_up_transition |= 0x0100;
    if (arm_rotate_threshold_passed)
    {
      if ((up_down_transition & 0x01C7) && (down_up_transition & 0x0038))
      {
        wake_up_by_accelerometr();
        time_scenario = FALSE;
      }else if (/*(Y_local_max - Y_local_min >= 50) ||*/ (up_down_transition & 0x0007))
      {
        wake_up_by_accelerometr();
        time_scenario = TRUE;
      }
      if (X_acc[0] >= -10 && X_acc[0] <= 10)
      {
        if ((Y_acc[0] <= -10 && (Y_local_max - Y_local_min) < 50) || (((Y_acc[0] + Y_acc[1] + Y_acc[2]) / 3) <= -10))
        {
          wake_up_by_accelerometr();
          time_scenario = TRUE;
        }
        // if (((Y_acc[0] + Y_acc[1] + Y_acc[2]) / 3) <= -10)
        // {
        //   up_down_transition = 0;
        //   down_up_transition = 0;
        //   wake_up_by_accelerometr();
        // }
      } 
    }
    
    
    
    // if (arm_rotate_threshold_passed && (Y_local_max - Y_local_min >= 50))
    // {
    //   arm_rotate_threshold_passed = FALSE;
    //   awu_mode_no_wake_up = FALSE;
    //   time_scenario = TRUE;
    // }
  }
  if (Y_acc[7] > 14 || X_acc[7]> 55)
  {
    arm_rotate_threshold_passed = TRUE;
  }
  //halt();
}

// void Awu_end_of_the_day_check_for_reset(){
  
// }

INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
    //btn_touch++;  
  awu_mode_no_wake_up = FALSE;
  if (!GPIO_ReadInputPin(SENS_BTN_PORT, BTN1_M) || !GPIO_ReadInputPin(SENS_BTN_PORT, BTN2_H)) // if any btn pressed
  {
    btn_pressed_time_start = miliseconds;   //counting btn pressed time
  }

  if (!GPIO_ReadInputPin(SENS_BTN_PORT, BTN1_M))
  {
    if (btn_was_pressed == 1 && showing_data == 1)   //if this btn was already pressed switching to showing charge
    {
      if (enable_step_counter)  //if step counter enabled dabl tap on this btn activates showing steps
      {
        showing_data = 7;
        show_today_step = FALSE;  // will be switched to TRUE in a steps showing block
        //step_showing_indication_done = FALSE;
      }else                     //if not, it activates showing charge
      {
        showing_data = 2;
      }
    }else{
      time_shwing_btn_mode = TRUE;
    }
    time_shoing_started_time_count = FALSE;       //reset time shoing time
    btn_was_pressed = 1;
  }
  
  if (!GPIO_ReadInputPin(SENS_BTN_PORT, BTN2_H))
  {
    if (btn_was_pressed == 2 && showing_data == 1)   //if this btn was already pressed switching to showing charge
    {
      showing_data = 2;
    }else{
      time_shwing_btn_mode = TRUE;
      //time_shoing_started_time_count = FALSE;       //reset time shoing time
    }
    time_shoing_started_time_count = FALSE;       //reset time shoing time
    btn_was_pressed = 2;
  }
  
  //btn_pressed_time_start = miliseconds;   //counting btn pressed time
  if (sleep_flag) 
  {
    showing_data = 4;        // show_open animation
    btn_was_pressed = 0;    //if watch just turnded on, ignoring the meaning of btn was pressed
    time_shwing_btn_mode = FALSE;
    time_scenario = TRUE;
  }

  if (GPIO_ReadInputPin(SENS_BTN_PORT, BTN1_M) && GPIO_ReadInputPin(SENS_BTN_PORT, BTN2_H)) // if all btn released
  {
    quick_increment_by_btn_long_press = FALSE;
  }
}

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  miliseconds++;
  TIM2_ClearFlag(0x0001);       //TIM2_FLAG_UPDATE
  if (showing_data == 3)
  {
    if ((miliseconds - settings_showing_started) % 1000 == 0)
    {
      seconds++;
      if (seconds > 9)
      {
        seconds = 0;
        ten_seconds++;
      }
      if (ten_seconds > 5)
      {
        ten_seconds = 0;
      }
    }
  }
  if (quick_increment_by_btn_long_press)
  {
    if (!GPIO_ReadInputPin(SENS_BTN_PORT, BTN1_M) && miliseconds % 400 == 0)//if btn still bressed
    {
      btn_was_pressed = 1;
    }
  }
}

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 
uint8_t data_to_send;
uint16_t alphabet_row;
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  if (lamp_R_switch)
  {
    alphabet_row = ~alphabet_iv3_r[lamp_R_symb_num];
    if (showing_data == 2 && low_bat_indication_blink_reserved & low_bat_indication_blink)
    {
      alphabet_row |= 0b0000000010000000;
    }
    
    if (seg_dot_on_lamp == 2)
    {
      alphabet_row &= ~seg_dot;
    }
    if (showing_data == 4 || showing_data == 5)
    {
      alphabet_row = ~animation_frame_r;
    }
    
    //alphabet_row = alphabet_iv3_r[11];
    //alphabet_row |= (0b0100000000000000 >> lamp_R_symb_num);
  }else{
    alphabet_row = ~alphabet_iv3_l[lamp_L_symb_num];
    if (seg_dot_on_lamp == 1)
    {
      alphabet_row &= ~seg_dot;
    }
    if (showing_data == 4 || showing_data == 5)
    {
      alphabet_row = ~animation_frame_l;
    }
  }
  
  if (seg_dot_on_lamp == 3 /*&& actual_RTC_milliceconds < 500*/)   // dot controll while shoing seconds
  {
    alphabet_row &= ~seg_dot;
  }
  
  data_to_send = alphabet_row;

  if (brightness != 255)
  {
    if (!brightness_controll_lamps_on)
    {
      TIM4_SetAutoreload(255 - brightness);
      lamp_R_switch = !lamp_R_switch;
    }else{
      TIM4_SetAutoreload(brightness);
      data_to_send = /*0b00001100*/0b10111000;//0b0000000010001000;
      alphabet_row = 0b0000110010111000;
      //data_to_send = ~data_to_send;
    }
    brightness_controll_lamps_on = !brightness_controll_lamps_on;
  }else
  {
    lamp_R_switch = !lamp_R_switch;  
  }

  SPI_SendData(data_to_send);
  nop();
  nop();
  nop();
  nop();
  nop();
  data_to_send = alphabet_row >> 8;
  SPI_SendData(data_to_send);
  nop();
  nop();
  nop();
  nop();
  nop();
  GPIO_WriteHigh(GPIOC, RCLK_PIN);
  GPIO_WriteLow(GPIOC, RCLK_PIN); 
}

void get_time(){
  // seconds = miliseconds % 10000 / 1000;
  // ten_seconds = 3;
  // minutes = 1;
  // ten_minutes = 4;
  // hours = 9;
  // ten_hours = 1;

  //todo remove halt bit before time reading // done in wake-up sequence

uint8_t old_day = day;
  #if RTC >= 1

  I2C_Read_three_more_byte(RTC_ADDR, 5, RxBuffer, 0);
  seconds = RxBuffer[1] & 0x0F;
  ten_seconds = (RxBuffer[1] >> 4) & 0x07;
  minutes = RxBuffer[2] & 0x0F;
  ten_minutes = RxBuffer[2] >> 4;
  hours = RxBuffer[3] & 0x0F;
  ten_hours = (RxBuffer[3] >> 4) & 0x03;
  day = RxBuffer[4] & 0x03;
  actual_RTC_milliceconds = (RxBuffer[0] >> 4) * 100 + (RxBuffer[0] & 0x0F) * 10;//miliseconds % 1000 / 100;
  
  #else

  I2C_Read_three_more_byte(RTC_ADDR, 4, RxBuffer, 0);
  seconds = RxBuffer[0] & 0x0F;
  ten_seconds = (RxBuffer[0] >> 4) & 0x07;
  minutes = RxBuffer[1] & 0x0F;
  ten_minutes = (RxBuffer[1] >> 4) & 0x07;
  hours = RxBuffer[2] & 0x0F;
  ten_hours = (RxBuffer[2] >> 4) & 0x03;
  day = RxBuffer[3] & 0x03;
  actual_RTC_milliceconds = miliseconds % 1000 / 100;
  #endif

//uint32_t awu_counter_old = awu_end_of_day_counter;
  
  
  awu_end_of_day_counter = /*1350000 - (uint32_t)((ten_hours * 10 + hours) * 56250 + (ten_minutes*10 + minutes) * 938 +*/ (ten_seconds*10 + seconds) * 9;
  //avoiding multiple reset  
  //if(awu_end_of_day_counter > awu_counter_old) awu_reset_allow = TRUE;
  //awu_reset_allow = FALSE;
  if (day != old_day){
    //awu_reset_allow = TRUE;
    step_counter_previous_day = step_counter_this_day;
    step_counter_this_day = 0;
    //awu_reset_allow = FALSE; 
  }
  //else{awu_reset_allow = FALSE;}
  
  // if (awu_reset_allow)
  //   {
  //     awu_reset_allow = FALSE;
  //     Awu_end_of_the_day_check_for_reset();
  //   }
}

void set_time(){
  #if RTC >= 1

  TxBuffer[0] = 0;
  TxBuffer[1] = 0;
  TxBuffer[2] = seconds |= (ten_seconds << 4);
  TxBuffer[3] = minutes |= (ten_minutes << 4);
  TxBuffer[4] = hours |= (ten_hours << 4);
  TxBuffer[5] = 0;
  TxBuffer[6] = 0;
  TxBuffer[7] = 0;
  TxBuffer[8] = 0;
  I2C_Send_data(RTC_ADDR, 9, TxBuffer);

  #else

  TxBuffer[0] = 0;
  TxBuffer[1] = seconds |= (ten_seconds << 4);
  TxBuffer[2] = minutes |= (ten_minutes << 4);
  TxBuffer[3] = hours |= (ten_hours << 4);
  TxBuffer[4] = 0;
  TxBuffer[5] = 0;
  TxBuffer[6] = 0;
  I2C_Send_data(RTC_ADDR, 7, TxBuffer);

  #endif
}

void show_open_animation(uint8_t animation){
  if (animation == 3)
  {
    get_time();
    if (time_scenario)
    {
      animation_frame_l = alphabet_iv3_l[ten_hours];
      animation_frame_r = alphabet_iv3_r[hours];
    }else{
      animation_frame_l = alphabet_iv3_l[18];    //A
      animation_frame_r = alphabet_iv3_r[15];    //bat 2
    }
    
    
    uint8_t temp_brightness = brightness;
    brightness = 1;
    while (brightness < temp_brightness && brightness < 128)
    {
      Delay_ms(70);
      brightness = brightness << 1;
    }
    if (temp_brightness == 255)
    {
      Delay_ms(70);
      brightness = 160;
    }
    Delay_ms(70);
    brightness = temp_brightness;
  }
  
  if (animation == 2)
  { 
    animation_frame_l = alphabet_iv3_l[11]; // grids
    animation_frame_r = alphabet_iv3_r[11];
    Delay_ms(150);
    for (uint8_t i = 0; i < 9; i++)
    {//in
      animation_frame_l |= seg_order_for_random_reserved[i];
      animation_frame_r |= seg_order_for_random_reserved[i];
      Delay_ms(40);
    }
    for (uint8_t i = 0; i < 9; i++)
    {//out
    uint16_t temp = ~seg_order_for_random_reserved[i];
      if (time_scenario)
      {
        animation_frame_l &= (temp | alphabet_iv3_l[ten_hours]);
        animation_frame_r &= (temp | alphabet_iv3_r[hours]);
      }else
      {
        animation_frame_l &= (temp | alphabet_iv3_l[18]);    //A
        animation_frame_r &= (temp | alphabet_iv3_r[15]);    //bat 2
      }
      Delay_ms(40);
    }
  }
  if (Animation == 1)
  {
    uint8_t index = animation - 1;
    for (uint8_t i = 0; i < 6; i++)
    {
      Show_easy_animation(index, i);//just optimisation
      // animation_frame_l = animation_array[index][i];
      // animation_frame_r = animation_array[index][i+6];
      // Delay_ms(70);
    }
  }
}

void show_close_animation(uint8_t animation){
  if (animation == 3)
  { 
    animation_frame_l = alphabet_iv3_l[lamp_L_symb_num];
    animation_frame_r = alphabet_iv3_r[lamp_R_symb_num];
    uint8_t temp_brightness = brightness;
    //brightness = 1;
    while (brightness > 1)
    {
      Delay_ms(50);
      if (brightness == 255)
      {
        brightness = 128;
      }else
      {
        brightness = brightness >> 1;
      }
    }
    Delay_ms(50);
    brightness = temp_brightness;
  }
  if (animation == 2)
  {
    animation_frame_l = alphabet_iv3_l[lamp_L_symb_num];
    animation_frame_r = alphabet_iv3_r[lamp_R_symb_num];
    for (int8_t i = 8; i >=0; i--)
    {//in
      animation_frame_l |= seg_order_for_random_reserved[i];
      animation_frame_r |= seg_order_for_random_reserved[i];
      Delay_ms(60);
    }
    for (int8_t i = 8; i >=0; i--)
    {//out
      animation_frame_l &= ~seg_order_for_random_reserved[i];
      animation_frame_r &= ~seg_order_for_random_reserved[i];
      Delay_ms(60);
    }
  }
  if (Animation == 1)
  {
    uint8_t index = animation - 1;
    for (int8_t i = 5; i >= 0; i--)
    {
      Show_easy_animation(index, i);//just optimisation
      // animation_frame_l = animation_array[index][i];
      // animation_frame_r = animation_array[index][i+6];
      // Delay_ms(70);
    }
  }
}

void Show_easy_animation(uint8_t animation, uint8_t frame){
  animation_frame_l = animation_array[animation][frame];
  animation_frame_r = animation_array[animation][frame+6];
  Delay_ms(70);
}

void show_settings_animation(){
  
  seg_dot_on_lamp = 0;
  for (uint8_t i = 0; i < 9; i++)
  {
    lamp_L_symb_num = 19 + i % 3;
    lamp_R_symb_num = 17 + i % 3;
    Delay_ms(100);
  }
}

void get_settings_from_eeprom(){
  uint8_t bool_settings = 0;
  brightness = *(PointerAttr uint8_t *) (MemoryAddressCast)BRIGHTNESS_EEPROM_ADDR;
  time_showing_duration_level = *(PointerAttr uint8_t *) (MemoryAddressCast)PERIOD_EEPROM_ADDR;
  Animation =  *(PointerAttr uint8_t *) (MemoryAddressCast)ANIMATION_EEPROM_ADDR;
  bool_settings = *(PointerAttr uint8_t *) (MemoryAddressCast)BOOL_EEPROM_ADDR;
  enable_step_counter = (bool_settings >> 2) & 0x01;
  enable_seconds = (bool_settings >> 1) & 0x01;
  enable_accelerometr = bool_settings & 0x01;
  if (brightness == 0 || Animation == 0xFF)//if EEPROM contains invalid data replace it with  normal data
  {
    brightness = Brightness;
    Animation = ANIMATION;
    time_showing_duration_level = 6;
    enable_seconds = 1;
    enable_accelerometr = 1;
    enable_step_counter = 1;
  }
}

void set_settings_to_eeprom(){
  //settings map:|  31----24  | 23----16 | 15------8 |                  7------1                |
  //             | brightness |  period  | animation | -----step_counter, seconds, accelerometr |
  //data type:   |    byte    |   byte   |    byte   |                    bool                  |

  uint8_t bool_settings = 0;
  // settings_word |= ((uint32_t)brightness << 24);
  // settings_word |= ((uint32_t)time_showing_duration_level << 16);
  // settings_word |= ((uint32_t)Animation << 8);
  bool_settings |= (enable_step_counter << 2);
  bool_settings |= (enable_seconds << 1);
  bool_settings |= enable_accelerometr;

  /* Unlock data memory */
    FLASH->DUKR = 0xAE; 
    FLASH->DUKR = 0x56;

  // check protection off
  while (!(FLASH->IAPSR & FLASH_IAPSR_DUL));
  *(PointerAttr uint8_t*) (uint16_t)0x4003 = brightness;
  *(PointerAttr uint8_t*) (uint16_t)0x4002 = time_showing_duration_level;
  *(PointerAttr uint8_t*) (uint16_t)0x4001 = Animation;
  *(PointerAttr uint8_t*) (uint16_t)0x4000 = bool_settings;

  /* Lock memory */
  FLASH->IAPSR &= (uint8_t)0xF7;  /*!< Data EEPROM memory */;
}

#if (RTC == 2)
void show_error(uint8_t error){
  seg_dot_on_lamp = 0;
  for (uint8_t j = 0; j < 4; j++)
  {
    lamp_L_symb_num = 13;
    lamp_R_symb_num = error;
    if (error == 2)
    {
      lamp_L_symb_num = errors_count / 10;
      lamp_R_symb_num = errors_count % 10;//&0x0F;
    }
    Delay_ms(200);
    lamp_L_symb_num = 11;
    lamp_R_symb_num = 11;
    Delay_ms(200);
  }
}
#endif

void Set_sleep_conditions(){
  time_shoing_started_time_count = FALSE;
  showing_data = 5;
}

void wake_up_by_accelerometr(){
  up_down_transition = 0;
  down_up_transition = 0;
  arm_rotate_threshold_passed = FALSE;
  awu_mode_no_wake_up = FALSE;
  showing_data = 4;        // show_open animation
  btn_was_pressed = 0;    //if watch just turnded on, ignoring the meaning of btn was pressed
  time_shwing_btn_mode = FALSE;
  for (uint8_t i = 0; i < 8; i++) //reset accelerometr data history 
  {
    Y_acc[i] = 0;
    X_acc[i] = 0;
  }
}

void Swap_segments(uint8_t seg1, uint8_t seg2){
  uint16_t temp;
  if (seg1 > 8)seg1 = 8;
  if (seg2 > 8)seg2 = 8;
  temp = seg_order_for_random_reserved[seg1];
  seg_order_for_random_reserved[seg1] = seg_order_for_random_reserved[seg2];
  seg_order_for_random_reserved[seg2] = temp;
}

void Mix_segments(){
  get_time();
  //uint32_t my_random_seed = seconds * 100000000 + actual_RTC_milliceconds * 1000000 + 
  ADC1_ClearFlag(ADC1_FLAG_EOC);
  ADC1_StartConversion();
  //while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC));
  uint16_t temp[] = {actual_RTC_milliceconds / 10, miliseconds % 100,
   awu_end_of_day_counter % 100, ADC1_GetConversionValue() % 100};
  Swap_segments(0, seconds);
  for (uint8_t i = 1; i < 5; i++)
  {
    uint8_t index = 2 * i;
    Swap_segments(index - 1, temp[i - 1] / 10);
    Swap_segments(index, temp[i - 1] % 10);
  }
  // temp = miliseconds % 100;
  // Swap_segments(3, temp / 10);
  // Swap_segments(4, temp % 10);
  // temp = ADC1_GetConversionValue() % 100;
  // Swap_segments(5, temp / 10);
  // Swap_segments(6, temp % 10);
  // temp = awu_end_of_day_counter % 100;
  // Swap_segments(7, temp / 10);
  // Swap_segments(8, temp % 10);
}

void Delay_ms(uint32_t delay)
{
  uint32_t delay_started = miliseconds;
  while (miliseconds - delay_started < delay);
}

/**
  * @brief Delay
  * @param nCount
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

void CLK_SYSCLKConfig(uint8_t CLK_Prescaler)
{
  /* check the parameters */
  //assert_param(IS_CLK_PRESCALER_OK(CLK_Prescaler));
  
  if (((uint8_t)CLK_Prescaler & (uint8_t)0x80) == 0x00) /* Bit7 = 0 means HSI divider */
  {
    CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
    CLK->CKDIVR |= (uint8_t)((uint8_t)CLK_Prescaler & (uint8_t)CLK_CKDIVR_HSIDIV);
  }
  else /* Bit7 = 1 means CPU divider */
  {
    CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_CPUDIV);
    CLK->CKDIVR |= (uint8_t)((uint8_t)CLK_Prescaler & (uint8_t)CLK_CKDIVR_CPUDIV);
  }
}

void TIM2_ITConfig(uint8_t TIM2_IT, FunctionalState NewState)
{
  /* Check the parameters */
  //assert_param(IS_TIM2_IT_OK(TIM2_IT));
  //assert_param(IS_FUNCTIONALSTATE_OK(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    TIM2->IER |= (uint8_t)TIM2_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    TIM2->IER &= (uint8_t)(~TIM2_IT);
  }
}

void TIM2_TimeBaseInit( uint8_t TIM2_Prescaler, uint16_t TIM2_Period)
{
  /* Set the Prescaler value */
  TIM2->PSCR = (uint8_t)(TIM2_Prescaler);
  /* Set the Autoreload value */
  TIM2->ARRH = (uint8_t)(TIM2_Period >> 8);
  TIM2->ARRL = (uint8_t)(TIM2_Period);
}

void TIM2_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  //assert_param(IS_FUNCTIONALSTATE_OK(NewState));
  
  /* set or Reset the CEN Bit */
  if (NewState != DISABLE)
  {
    TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
  }
  else
  {
    TIM2->CR1 &= (uint8_t)(~TIM2_CR1_CEN);
  }
}

void TIM2_ClearFlag(uint16_t TIM2_FLAG)
{
  /* Check the parameters */
  //assert_param(IS_TIM2_CLEAR_FLAG_OK(TIM2_FLAG));
  
  /* Clear the flags (rc_w0) clear this bit by writing 0. Writing �1� has no effect*/
  TIM2->SR1 = (uint8_t)(~((uint8_t)(TIM2_FLAG)));
  TIM2->SR2 = (uint8_t)(~((uint8_t)((uint8_t)TIM2_FLAG >> 8)));
}

void I2C_Init(/*uint8_t OutputClockFreq_1_4,*/ uint8_t Ack, uint8_t InputClockFrequencyMHz ){

  uint8_t tmpccrh = 0;

  /*------------------------- I2C FREQ Configuration ------------------------*/
  /* Clear frequency bits */
  I2C->FREQR &= (uint8_t)(~I2C_FREQR_FREQ);
  /* Write new value */
  I2C->FREQR |= InputClockFrequencyMHz;

  /*--------------------------- I2C CCR Configuration ------------------------*/
  /* Disable I2C to configure TRISER */
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);

  /* Clear CCRH & CCRL */
  I2C->CCRH &= (uint8_t)(~(I2C_CCRH_FS | I2C_CCRH_DUTY | I2C_CCRH_CCR));
  I2C->CCRL &= (uint8_t)(~I2C_CCRL_CCR);
  // if (OutputClockFreq_1_4 == 1)
  // {

  //   /* Set Maximum Rise Time: 1000ns max in Standard Mode
  //   = [1000ns/(1/InputClockFrequencyMHz.10e6)]+1
  //   = InputClockFrequencyMHz+1 */
  //   I2C->TRISER = (uint8_t)(InputClockFrequencyMHz + (uint8_t)1);
  // }
  // if (OutputClockFreq_1_4 == 4)
  // {

    /* Set Maximum Rise Time: 300ns max in Fast Mode
    = [300ns/(1/InputClockFrequencyMHz.10e6)]+1
    = [(InputClockFrequencyMHz * 3)/10]+1 */
    //tmpval = ((InputClockFrequencyMHz * 3) / 10) + 1;
    I2C->TRISER = (uint8_t)((InputClockFrequencyMHz * 3) / 10) + 1;
    tmpccrh = I2C_CCRH_FS;
  // }
  I2C->CCRH = tmpccrh;
  I2C->CCRL = (uint8_t)(InputClockFrequencyMHz * 10 / (/*OutputClockFreq_1_4*/ 4 << 1));

  /* Enable I2C */
  I2C->CR1 |= I2C_CR1_PE;
  
  /* Configure I2C acknowledgement */
  I2C_AcknowledgeConfig(Ack);

  I2C->OARH = (uint8_t)I2C_OARH_ADDCONF;
}

void I2C_Bus_init(uint8_t Addr, uint8_t direction, bool initial_check_for_busy){
  uint16_t time_out = TimErr;//Счетчик от зависания функции.
	if(initial_check_for_busy) while((I2C_GetFlagStatus(I2C_FLAG_BUSBUSY)) && (time_out--));//Проверяем занятость линии I2C.
	I2C_GenerateSTART(ENABLE);//Запуск I2C для передачи данных.
	time_out = TimErr;//Счетчик от зависания функции.
	while((!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))&& (time_out--));//Ждём установки бита MASTER.
	I2C_Send7bitAddress((uint8_t)Addr, direction);//Отсылаем адрес вызываемого устройства.
	time_out = TimErr;//Счетчик от зависания функции.
	while((!I2C_GetFlagStatus(I2C_FLAG_ADDRESSSENTMATCHED))&& (time_out--));//Ждём когда нужное устройство подтвердит.
	(void)I2C->SR1; (void)I2C->SR3;//Комбинация для сброса ADDR.
}

void I2C_Send_data(uint8_t Addr, uint8_t countData, uint8_t* dataBuffer){
  uint16_t time_out = TimErr;
	I2C_Bus_init(Addr, I2C_DIRECTION_TX, TRUE);
	while(countData){//Повторяем пока не отправятся все данные.
		time_out = TimErr;//Счетчик от зависания функции.
		while( !(I2C->SR1 & 128) && ( time_out--));//Ждём когда буфер передающего регистра будет пуст.
		I2C_SendData(*dataBuffer);
		*dataBuffer++;
		countData--;
	}
	time_out = TimErr;//Счетчик от зависания функции.
	while(( !I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && ( time_out--));//Ждём окончания отправки данных.
	I2C_GenerateSTOP(ENABLE);//Установка STOP бита на линии.
	time_out = TimErr;//Счетчик от зависания функции.
	while(( I2C->CR2 & I2C_CR2_STOP)&& ( time_out--));//Ждём остановки передачи и STOP на линии.
}

void I2C_Read_three_more_byte( uint8_t Addr, uint8_t countData, uint8_t* dataBuffer, uint8_t readFrom){
	uint16_t time_out = TimErr;//Счетчик от зависания функции.
  // if (readFrom)
  // {
    I2C_Bus_init(Addr, I2C_DIRECTION_TX, TRUE);
    
	  I2C_AcknowledgeConfig( I2C_ACK_CURR);//Устанавливаем контроль бита ACK.
    time_out = TimErr;//Счетчик от зависания функции.
		while(!(I2C->SR1 & 128) && ( time_out--));//Ждём когда буфер передающего регистра будет пуст.
		I2C_SendData(readFrom);
	  time_out = TimErr;//Счетчик от зависания функции.
	  while(( !I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && ( time_out--));//Ждём окончания отправки данных.
    I2C_Bus_init(Addr, I2C_DIRECTION_RX, FALSE);
  // }else{
    //I2C_Bus_init(Addr, I2C_DIRECTION_RX, TRUE);
  // }
  
	
	/*Функционал для приёма трёх байтов.*/
	(void)I2C->SR1; (void)I2C->SR3;//Комбинация для сброса ADDR.
	if( countData>3){//Если принимаем больше трёх байтов.
		while(countData > 3){
			time_out = TimErr;//Счетчик от зависания функции.
			while ((!I2C_GetFlagStatus( I2C_FLAG_TRANSFERFINISHED))&&(time_out--));//Ждём отправку всех данных устройству.
			*dataBuffer = I2C_ReceiveData();
			*dataBuffer++;
			countData--;
		}
	}
	time_out= TimErr;//Счетчик от зависания функции.
	while ((!I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED))&&(time_out--));//Ждём отправку всех данных устройству.
	I2C_AcknowledgeConfig(I2C_ACK_NONE);//Отключаем ACK после приёма последнего байта, что бы не отправлялись данные.
	*dataBuffer = I2C_ReceiveData();
	*dataBuffer++;
	I2C_GenerateSTOP(ENABLE);//Установка STOP бита на линии.
	*dataBuffer = I2C_ReceiveData();
	*dataBuffer++;
	time_out= TimErr;//Счетчик от зависания функции.
	while((!I2C_GetFlagStatus( I2C_FLAG_RXNOTEMPTY))&& ( time_out--));//Ждём когда придут данные.
	*dataBuffer = I2C_ReceiveData();
	time_out= TimErr;//Счетчик от зависания функции.
	while((I2C->CR2 & I2C_CR2_STOP)&& (time_out--));//Ждём остановки передачи и STOP на линии.
	//I2C_AcknowledgeConfig( I2C_ACK_CURR);//Устанавливаем контроль бита ACK.
}

// #ifdef USE_FULL_ASSERT

// /**
//   * @brief  Reports the name of the source file and the source line number
//   *   where the assert_param error has occurred.
//   * @param file: pointer to the source file name
//   * @param line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t* file, uint32_t line)
// { 
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

//   /* Infinite loop */
//   uint8_t i = 0;
//   while (1)
//   {
//      lamp_R_symb_num = i % 10;
//      i++;
//   }
// }
// #endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
