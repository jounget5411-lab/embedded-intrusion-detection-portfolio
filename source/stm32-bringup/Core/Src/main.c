/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>   // strlen 사용용
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t reg;
    uint8_t  val;
} sensor_reg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 초음파
#define SOUND_SPEED_CM_PER_US  0.0343f   // 343m/s ≈ 0.0343 cm/us
#define DIST_LED_TH_CM   30.0f


// ------------------ Zynq handshake pins ------------------
#define ZYNQ_ALARM_GPIO_PORT   GPIOA
#define ZYNQ_ALARM_PIN         GPIO_PIN_4   // alarm - PA4 (STM -> Zynq)

#define ZYNQ_BUSY_GPIO_PORT    GPIOB
#define ZYNQ_BUSY_PIN          GPIO_PIN_0   // busy  - PB0 (Zynq -> STM)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ===== HC-SR04 측정 변수 =====
static volatile uint32_t ic_rise = 0;
static volatile uint32_t ic_fall = 0;
static volatile uint8_t  capture_state = 0; // 0: rising 대기, 1: falling 대기
static volatile uint32_t echo_us = 0;
static volatile uint8_t  echo_done = 0;

static volatile float distance_cm = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
// 아두캠
static void Check_VSYNC_Signal(void);
static void I2C1_ScanBus(void);
static void Cam_Read_OV5642_ID(void);
static void Arduchip_Test(void);
static void Camera_Init_640x480_RGB565(void);
void ProcessUartCommand(void);
static void uart_print(const char *s);

// 초음파
static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);
static void HCSR04_Trigger(void);
static float HCSR04_Read_cm(uint32_t timeout_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ArduChip 레지스터 (수정 금지)
#define ARDUCHIP_TEST1       0x00
#define ARDUCHIP_FIFO        0x04
#define ARDUCHIP_TRIG        0x41
#define FIFO_CLEAR_MASK      0x01
#define FIFO_START_MASK      0x02
#define FIFO_SIZE1           0x42
#define FIFO_SIZE2           0x43
#define FIFO_SIZE3           0x44
#define CAP_DONE_MASK        0x08
#define BURST_FIFO_READ      0x3C

#define OV5642_I2C_ADDR      (0x78)
#define CAM_CS_GPIO_Port     GPIOB
#define CAM_CS_Pin           GPIO_PIN_6
#define CAM_CS_LOW()         HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET)
#define CAM_CS_HIGH()        HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET)

// === 통신 프로토콜 정의 ===
// (우선 확실하게 나오는 320x240으로 테스트 후, 성공하면 640으로 늘립시다)
#define FRAME_WIDTH       320u
#define FRAME_HEIGHT      240u
#define BYTES_PER_PIXEL   2u
#define FRAME_SIZE_BYTES  (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)
#define FRAME_MAGIC       0x1234ABCDu
#define UART_CMD_CAPTURE  'C'

// PC 헤더
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t  format;
    uint8_t  reserved[3];
} FrameHeader;

// 함수 선언
static void uart_print(const char *s);
static uint8_t SPI1_TxRx(uint8_t data);
static void Arduchip_WriteReg(uint8_t addr, uint8_t value);
static uint8_t Arduchip_ReadReg(uint8_t addr);
static HAL_StatusTypeDef OV5642_LoadRegTable(const sensor_reg_t *table);

// ★★★ [핵심] OV5642 초기화 레지스터 (QVGA Preview - 검증된 값) ★★★
// 이 설정은 아두캠 라이브러리에서 가져온 "가장 확실한" 설정입니다.
const sensor_reg_t OV5642_INIT_TABLE[] = {
    {0x3103,0x93}, {0x3008,0x82}, {0x3017,0x7f}, {0x3018,0xfc}, {0x3810,0xc2}, {0x3615,0xf0},
    {0x3000,0x00}, {0x3001,0x00}, {0x3002,0x5c}, {0x3003,0x00}, {0x3004,0xff}, {0x3005,0xff},
    {0x3006,0x43}, {0x3007,0x37}, {0x3011,0x08}, {0x3010,0x10}, {0x460c,0x22}, {0x3815,0x04},
    {0x370c,0xa0}, {0x3602,0xfc}, {0x3612,0xff}, {0x3634,0xc0}, {0x3613,0x00}, {0x3605,0x7c},
    {0x3621,0x09}, {0x3622,0x60}, {0x3604,0x40}, {0x3603,0xa7}, {0x3603,0x27}, {0x4000,0x21},
    {0x401d,0x22}, {0x3600,0x54}, {0x3605,0x04}, {0x3606,0x3f}, {0x3c01,0x80}, {0x5000,0x4f},
    {0x5020,0x04}, {0x5181,0x79}, {0x5182,0x00}, {0x5185,0x22}, {0x5197,0x01}, {0x5001,0xff},
    {0x5500,0x0a}, {0x5504,0x00}, {0x5505,0x7f}, {0x5080,0x08}, {0x300e,0x18}, {0x4610,0x00},
    {0x471d,0x05}, {0x4708,0x06}, {0x3808,0x02}, {0x3809,0x80}, {0x380a,0x01}, {0x380b,0xe0},
    {0x380e,0x07}, {0x380f,0xd0}, {0x501f,0x00}, {0x5000,0x4f}, {0x4300,0x30}, {0x3503,0x07},
    {0x3501,0x73}, {0x3502,0x80}, {0x350b,0x00}, {0x3503,0x07}, {0x3824,0x11}, {0x3501,0x1e},
    {0x3502,0x80}, {0x350b,0x7f}, {0x380c,0x0c}, {0x380d,0x80}, {0x380e,0x03}, {0x380f,0xe8},
    {0x3a0d,0x04}, {0x3a0e,0x03}, {0x3818,0xc1}, {0x3705,0xdb}, {0x370a,0x81}, {0x3801,0x80},
    {0x3621,0x87}, {0x3801,0x50}, {0x3803,0x08}, {0x3827,0x08}, {0x3810,0x40}, {0x3804,0x05},
    {0x3805,0x00}, {0x5682,0x05}, {0x5683,0x00}, {0x3806,0x03}, {0x3807,0xc0}, {0x5686,0x03},
    {0x5687,0xbc}, {0x3a00,0x78}, {0x3a1a,0x05}, {0x3a13,0x30}, {0x3a18,0x00}, {0x3a19,0x7c},
    {0x3a08,0x12}, {0x3a09,0xc0}, {0x3a0a,0x0f}, {0x3a0b,0xa0}, {0x350c,0x07}, {0x350d,0xd0},
    {0x3500,0x00}, {0x3501,0x00}, {0x3502,0x00}, {0x350a,0x00}, {0x350b,0x00}, {0x3503,0x00},
    {0x528a,0x02}, {0x528b,0x04}, {0x528c,0x08}, {0x528d,0x08}, {0x528e,0x08}, {0x528f,0x10},
    {0x5290,0x10}, {0x5292,0x00}, {0x5293,0x02}, {0x5294,0x00}, {0x5295,0x02}, {0x5296,0x00},
    {0x5297,0x02}, {0x5298,0x00}, {0x5299,0x02}, {0x529a,0x00}, {0x529b,0x02}, {0x529c,0x00},
    {0x529d,0x02}, {0x529e,0x00}, {0x529f,0x02}, {0x3030,0x0b}, {0x3a02,0x00}, {0x3a03,0x7d},
    {0x3a04,0x00}, {0x3a14,0x00}, {0x3a15,0x7d}, {0x3a16,0x00}, {0x3a00,0x78}, {0x3a08,0x09},
    {0x3a09,0x60}, {0x3a0a,0x07}, {0x3a0b,0xd0}, {0x3a0d,0x08}, {0x3a0e,0x06}, {0x5193,0x70},
    {0x589b,0x04}, {0x589a,0xc5}, {0x401e,0x20}, {0x4001,0x42}, {0x401c,0x04}, {0x528a,0x01},
    {0x528b,0x04}, {0x528c,0x08}, {0x528d,0x10}, {0x528e,0x20}, {0x528f,0x28}, {0x5290,0x30},
    {0x5292,0x00}, {0x5293,0x01}, {0x5294,0x00}, {0x5295,0x04}, {0x5296,0x00}, {0x5297,0x08},
    {0x5298,0x00}, {0x5299,0x10}, {0x529a,0x00}, {0x529b,0x20}, {0x529c,0x00}, {0x529d,0x28},
    {0x529e,0x00}, {0x529f,0x30}, {0x5282,0x00}, {0x5300,0x00}, {0x5301,0x20}, {0x5302,0x00},
    {0x5303,0x7c}, {0x530c,0x00}, {0x530d,0x0c}, {0x530e,0x20}, {0x530f,0x80}, {0x5310,0x20},
    {0x5311,0x80}, {0x5308,0x20}, {0x5309,0x40}, {0x5304,0x00}, {0x5305,0x30}, {0x5306,0x00},
    {0x5307,0x80}, {0x5314,0x08}, {0x5315,0x20}, {0x5319,0x30}, {0x5316,0x10}, {0x5317,0x00},
    {0x5318,0x02}, {0x5402,0x3f}, {0x5403,0x00}, {0x3406,0x00}, {0x5180,0xff}, {0x5181,0x52},
    {0x5182,0x11}, {0x5183,0x14}, {0x5184,0x25}, {0x5185,0x24}, {0x5186,0x06}, {0x5187,0x08},
    {0x5188,0x08}, {0x5189,0x7c}, {0x518a,0x60}, {0x518b,0xb2}, {0x518c,0xb2}, {0x518d,0x44},
    {0x518e,0x3d}, {0x518f,0x58}, {0x5190,0x46}, {0x5191,0xf8}, {0x5192,0x04}, {0x5193,0x70},
    {0x5194,0xf0}, {0x5195,0xf0}, {0x5196,0x03}, {0x5197,0x01}, {0x5198,0x04}, {0x5199,0x12},
    {0x519a,0x04}, {0x519b,0x00}, {0x519c,0x06}, {0x519d,0x82}, {0x519e,0x00}, {0x5025,0x80},
    {0x5583,0x40}, {0x5584,0x40}, {0x5580,0x02}, {0x5000,0xcf}, {0x3710,0x10}, {0x3632,0x51},
    {0x3702,0x10}, {0x3703,0xb2}, {0x3704,0x18}, {0x370b,0x40}, {0x370d,0x03}, {0x3631,0x01},
    {0x3632,0x52}, {0x3606,0x24}, {0x3620,0x96}, {0x5785,0x07}, {0x3a13,0x30}, {0x3600,0x52},
    {0x3604,0x48}, {0x3606,0x1b}, {0x370d,0x0b}, {0x370f,0xc0}, {0x3709,0x01}, {0x3823,0x00},
    {0x5007,0x00}, {0x5009,0x00}, {0x5011,0x00}, {0x5013,0x00}, {0x519e,0x00}, {0x5086,0x00},
    {0x5087,0x00}, {0x5088,0x00}, {0x5089,0x00}, {0x302b,0x00}, {0x3808,0x01}, {0x3809,0x40},
    {0x380a,0x00}, {0x380b,0xf0}, {0x3a00,0x78}, {0x5001,0xFF}, {0x5583,0x50}, {0x5584,0x50},
    {0x5580,0x02}, {0x3c01,0x80}, {0x3c00,0x04}, {0xFFFF, 0xFF}
};



// ================= DWT us delay =================
static void DWT_Delay_Init(void)
{
    // Cortex-M3 (STM32F1)는 DWT 사용 가능
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Trace enable
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Cycle counter enable
}

static void delay_us(uint32_t us)
{
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {}
}

// ================= HC-SR04 Trigger =================
static void HCSR04_Trigger(void)
{
    // PA1 = TRIG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    delay_us(2);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    delay_us(10);  // 10us pulse
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

// ================= HC-SR04 Read distance (cm) =================
// timeout_ms 안에 echo_done 안되면 -1 반환
static float HCSR04_Read_cm(uint32_t timeout_ms)
{
    echo_done = 0;
    capture_state = 0;
    echo_us = 0;

    // Rising edge부터 캡처하도록 세팅
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    // 인터럽트 캡처 시작
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    // Trigger
    HCSR04_Trigger();

    // 대기 (blocking)
    uint32_t t0 = HAL_GetTick();
    while (!echo_done)
    {
        if ((HAL_GetTick() - t0) > timeout_ms)
        {
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
            return -1.0f; // timeout
        }
    }

    // echo_us -> distance(cm)
    float cm = (echo_us * SOUND_SPEED_CM_PER_US) / 2.0f;
    return cm;
}

// ================= TIM Input Capture Callback =================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (capture_state == 0)
        {
            // Rising edge time
            ic_rise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            capture_state = 1;

            // 다음은 Falling edge 캡처
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            // Falling edge time
            ic_fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            // 1MHz timer 기준 -> tick == us
            if (ic_fall >= ic_rise) echo_us = (ic_fall - ic_rise);
            else                    echo_us = ((htim->Init.Period + 1U) - ic_rise) + ic_fall;

            // 끝
            echo_done = 1;
            capture_state = 0;

            // 다시 rising으로 복귀
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

            // 캡처 종료(필요시 계속 켜도 되지만, 여기서는 측정마다 켰다 꺼서 안정적)
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
        }
    }
}



// ================= SPI2 -> ZYNQ Streaming =================
#define SPI2_CMD_START   0xC1  // Zynq가 MOSI로 이 바이트 보내면 STM이 캡처+스트림 준비
#define SPI2_STATUS_OK   0x5A
#define SPI2_STATUS_BUSY 0xBC

typedef enum {
  S2_IDLE = 0,
  S2_SEND_STATUS,
  S2_SEND_HDR,
  S2_SEND_LEN,
  S2_SEND_IMG,
  S2_DONE
} s2_state_t;

static volatile s2_state_t s2_state = S2_IDLE;

// SPI2 1바이트 송수신용
static uint8_t s2_tx = 0xFF;
static uint8_t s2_rx = 0x00;

// 이미지 스트리밍용
static uint32_t s2_img_len = 0;
static uint32_t s2_remaining = 0;

// 헤더/길이 바이트 스트림
static FrameHeader s2_hdr;
static uint8_t s2_len_bytes[4];
static uint32_t s2_hdr_idx = 0;
static uint32_t s2_len_idx = 0;

// FIFO에서 2KB씩 읽어와서 SPI2로 내보내는 버퍼
static uint8_t s2_chunk[2048];
static uint32_t s2_chunk_idx = 0;
static uint32_t s2_chunk_valid = 0;

static void SPI2_SlaveKick_1byte(void);
static void SPI2_PrepareNewFrame(void);
static void SPI2_FillNextChunk(void);

// SPI2 “1바이트 트리거” 시작
static void SPI2_SlaveKick_1byte(void)
{
  // 1바이트씩 계속 IT로 돌리면(느리지만) 구현이 제일 확실함
  HAL_SPI_TransmitReceive_IT(&hspi2, &s2_tx, &s2_rx, 1);
}

// Arducam FIFO에서 다음 chunk 채우기 (BURST_FIFO_READ 상태 유지 필요)
static void SPI2_FillNextChunk(void)
{
  if (s2_remaining == 0) {
    s2_chunk_valid = 0;
    return;
  }

  uint32_t to_read = (s2_remaining > sizeof(s2_chunk)) ? sizeof(s2_chunk) : s2_remaining;
  for (uint32_t i=0; i<to_read; i++) {
    s2_chunk[i] = SPI1_TxRx(0x00); // SPI1로 FIFO 1바이트씩 읽기
  }
  s2_chunk_valid = to_read;
  s2_chunk_idx = 0;
}

// 새 프레임 캡처 + FIFO burst-read 준비 + 헤더/길이 준비
static void SPI2_PrepareNewFrame(void)
{
  // 1) 캡처 시작
  Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
  Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_START_MASK);

  uint32_t t0 = HAL_GetTick();
  while (!(Arduchip_ReadReg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
    if (HAL_GetTick() - t0 > 2000) {
      // 실패 시 IDLE로 복귀
      s2_state = S2_IDLE;
      return;
    }
  }

  // 2) 길이 읽기
  uint32_t len1 = Arduchip_ReadReg(FIFO_SIZE1);
  uint32_t len2 = Arduchip_ReadReg(FIFO_SIZE2);
  uint32_t len3 = Arduchip_ReadReg(FIFO_SIZE3) & 0x7f;
  uint32_t length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
  if (length == 0) {
    s2_state = S2_IDLE;
    return;
  }

  s2_img_len = length;
  s2_remaining = length;

  // 3) 헤더 준비(PC UART용 헤더와 동일 포맷 사용)
  s2_hdr.magic  = FRAME_MAGIC;
  s2_hdr.width  = FRAME_WIDTH;
  s2_hdr.height = FRAME_HEIGHT;
  s2_hdr.format = 2; // RGB565
  s2_hdr.reserved[0]=0; s2_hdr.reserved[1]=0; s2_hdr.reserved[2]=0;
  s2_hdr_idx = 0;

  // 4) 길이 4바이트(LSB first)
  s2_len_bytes[0] = (uint8_t)(s2_img_len & 0xFF);
  s2_len_bytes[1] = (uint8_t)((s2_img_len >> 8) & 0xFF);
  s2_len_bytes[2] = (uint8_t)((s2_img_len >> 16) & 0xFF);
  s2_len_bytes[3] = (uint8_t)((s2_img_len >> 24) & 0xFF);
  s2_len_idx = 0;

  // 5) FIFO burst-read 모드로 진입 (이 상태에서 SPI1_TxRx(0x00)로 데이터가 쭉 나옴)
  CAM_CS_LOW();
  SPI1_TxRx(BURST_FIFO_READ);

  // 6) 첫 chunk 채움
  SPI2_FillNextChunk();

  // 7) 상태 전환: STATUS -> HDR -> LEN -> IMG
  s2_state = S2_SEND_STATUS;
}




static inline bool Zynq_IsBusy(void)
{
    return (HAL_GPIO_ReadPin(ZYNQ_BUSY_GPIO_PORT, ZYNQ_BUSY_PIN) == GPIO_PIN_SET);
}


// 알람을 "펄스"로 보낼지, "레벨"로 보낼지는 Zynq 쪽 인터럽트 방식에 따라 달라짐.
// 보통 인터럽트는 엣지 트리거가 편해서 펄스로 추천.
static inline void Zynq_AlarmHigh(void)
{
    HAL_GPIO_WritePin(ZYNQ_ALARM_GPIO_PORT, ZYNQ_ALARM_PIN, GPIO_PIN_SET);
}

static inline void Zynq_AlarmLow(void)
{
    HAL_GPIO_WritePin(ZYNQ_ALARM_GPIO_PORT, ZYNQ_ALARM_PIN, GPIO_PIN_RESET);
}


// busy=0일 때만 알람 펄스 발생(예: 50us)
// 반환값: true=보냄, false=busy라서 안보냄
bool Zynq_SendAlarmPulse_IfIdle(uint32_t pulse_us)
{
    if (Zynq_IsBusy()) {
        return false;  // 연산중이면 안 보냄
    }

    // 짧게 High 펄스
    Zynq_AlarmHigh();

    // F103에서 microsecond delay는 DWT 쓰거나 타이머 쓰는 게 정석인데,
    // 간단히는 아래처럼 짧은 루프(대충)도 가능.
    // 정확한 us가 필요하면 TIM 기반 delay로 바꾸자.
    for (volatile uint32_t i = 0; i < (pulse_us * 10); i++) {
        __NOP();
    }

    Zynq_AlarmLow();
    return true;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  //============== 카메라 =====================
  CAM_CS_HIGH();
  HAL_UART_Transmit(&huart2, (uint8_t*)"UART ALIVE CHECK\r\n", 18, 100);
  // 1. 센서 ID 확인 및 테스트
  I2C1_ScanBus();
  Cam_Read_OV5642_ID();
  Arduchip_Test();

  // 2. 카메라 설정 (기초설정 + 640x480 + RGB565)
  // (USER CODE BEGIN 4에 정의된 함수 호출)
  Camera_Init_640x480_RGB565();
  Check_VSYNC_Signal();
  uart_print("Ready to capture. Waiting for 'C' from PC...\r\n");



  // 초음파
  DWT_Delay_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    float cm = HCSR04_Read_cm(60);

    if (cm < 100)
    {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // ON
       Zynq_SendAlarmPulse_IfIdle(50); // 50us
    }
    else
    {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // OFF
    }


    ProcessUartCommand();
    HAL_Delay(200);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  // PC에서 명령이 오면 처리 (버튼 없이 자동)

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ★ VSYNC 신호 진단 함수 (심장 박동 체크)
static void Check_VSYNC_Signal(void)
{
    uart_print("\r\n[DIAGNOSTIC] Checking VSYNC Signal (1 sec)...\r\n");

    uint32_t high_cnt = 0;
    uint32_t low_cnt = 0;
    uint32_t start = HAL_GetTick();

    // 1초 동안 VSYNC 상태를 1000번 정도 읽어봅니다.
    while (HAL_GetTick() - start < 1000)
    {
        // ARDUCHIP_TRIG(0x41) 레지스터의 0번 비트가 VSYNC 신호입니다.
        uint8_t status = Arduchip_ReadReg(ARDUCHIP_TRIG);
        if (status & 0x01) high_cnt++;
        else low_cnt++;

        // 너무 빠르지 않게 약간 딜레이
        for(volatile int i=0; i<1000; i++);
    }

    char buf[64];
    snprintf(buf, sizeof(buf), " -> High: %lu, Low: %lu\r\n", high_cnt, low_cnt);
    uart_print(buf);

    if (high_cnt > 0 && low_cnt > 0) {
        uart_print(" -> [RESULT] VSYNC Toggling! SENSOR IS ALIVE.\r\n");
    } else {
        uart_print(" -> [RESULT] VSYNC Dead. Sensor is NOT outputting frames.\r\n");
    }
    uart_print("------------------------------------------\r\n");
}



static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

// I2C 레지스터 테이블 로드 함수
static HAL_StatusTypeDef OV5642_LoadRegTable(const sensor_reg_t *table)
{
    HAL_StatusTypeDef ret;
    int i = 0;

    while (1)
    {
        uint16_t reg = table[i].reg;
        uint8_t  val = table[i].val;

        // 종료 마커: {0xFFFF, 0xFF}
        if (reg == 0xFFFF && val == 0xFF)
            break;

        ret = HAL_I2C_Mem_Write(&hi2c1,
                                OV5642_I2C_ADDR,
                                reg,
                                I2C_MEMADD_SIZE_16BIT,
                                &val,
                                1,
                                100);
        if (ret != HAL_OK)
        {
            char buf[96];
            snprintf(buf, sizeof(buf),
                     "OV5642_LoadRegTable: write 0x%04X=0x%02X failed, err=0x%08lX\r\n",
                     reg, val, HAL_I2C_GetError(&hi2c1));
            uart_print(buf);
            return ret;
        }
        // 안정성을 위해 센서 설정 시 약간의 딜레이
        HAL_Delay(1);
        i++;
    }
    return HAL_OK;
}

// I2C 레지스터 읽기
static HAL_StatusTypeDef OV5642_ReadReg(uint16_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(
      &hi2c1,
      OV5642_I2C_ADDR,
      reg,
      I2C_MEMADD_SIZE_16BIT,
      value,
      1,
      100
  );
}

// 센서 ID 확인
static void Cam_Read_OV5642_ID(void)
{
    uint8_t id_high = 0, id_low = 0;

    // I2C ACK 테스트
    if (HAL_I2C_IsDeviceReady(&hi2c1, OV5642_I2C_ADDR, 3, 100) != HAL_OK) {
        uart_print("I2C Device NOT Ready!\r\n");
        return;
    }

    OV5642_ReadReg(0x300A, &id_high);
    OV5642_ReadReg(0x300B, &id_low);

    char buf[64];
    snprintf(buf, sizeof(buf), "OV5642 ID = 0x%02X 0x%02X (expect 0x56 0x42)\r\n", id_high, id_low);
    uart_print(buf);
}

// I2C 스캔
static void I2C1_ScanBus(void)
{
    HAL_StatusTypeDef ret;
    char buf[64];
    uart_print("I2C1 scan start\r\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        ret = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10);
        if (ret == HAL_OK) {
            snprintf(buf, sizeof(buf), "  - Found 0x%02X\r\n", addr << 1);
            uart_print(buf);
        }
    }
    uart_print("I2C1 scan done\r\n");
}

// SPI TxRx
static uint8_t SPI1_TxRx(uint8_t data)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

// ArduChip 레지스터 쓰기 (수정됨: 안정성 강화)
static void Arduchip_WriteReg(uint8_t addr, uint8_t value)
{
    CAM_CS_LOW();
    HAL_Delay(1); // ★ [핵심] 칩이 준비될 시간을 줍니다 (1ms)
    SPI1_TxRx(addr | 0x80);  // 쓰기 명령 전송


    SPI1_TxRx(value);        // 데이터 전송
    CAM_CS_HIGH();

    HAL_Delay(1); // 기존 안정성 딜레이 유지
}
// ArduChip 레지스터 읽기
static uint8_t Arduchip_ReadReg(uint8_t addr)
{
    uint8_t val;
    CAM_CS_LOW();
    HAL_Delay(1); // ★ [핵심] 읽기 전에도 준비 시간 부여
    SPI1_TxRx(addr & 0x7F);
    val = SPI1_TxRx(0x00);
    CAM_CS_HIGH();
    return val;
}

// ArduChip 테스트
/* USER CODE BEGIN 4 내의 Arduchip_Test 함수 교체 */

static void Arduchip_Test(void)
{
    char buf[64];
    uart_print("\r\n================================\r\n");
    uart_print("   SPI DIAGNOSTIC TEST START    \r\n");
    uart_print("================================\r\n");

    // 1. 단순 읽기/쓰기 테스트 (비트 이동 확인용)
    // 0x01, 0x02, 0x04 ... 순서대로 보내봅니다.
    uint8_t test_patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

    for (int i = 0; i < 8; i++) {
        uint8_t target = test_patterns[i];

        Arduchip_WriteReg(ARDUCHIP_TEST1, target);
        uint8_t read_back = Arduchip_ReadReg(ARDUCHIP_TEST1);

        snprintf(buf, sizeof(buf), "Write: 0x%02X -> Read: 0x%02X ", target, read_back);
        uart_print(buf);

        if (read_back == target) {
            uart_print("[OK]\r\n");
        } else if (read_back == (target << 1)) {
            uart_print("[FAIL] Left Shift (Try 2EDGE)\r\n");
        } else if (read_back == (target >> 1)) {
            uart_print("[FAIL] Right Shift (Try 1EDGE)\r\n");
        } else if (read_back == 0x00 || read_back == 0xFF) {
            uart_print("[FAIL] Stuck (Check Wiring/Power)\r\n");
        } else {
            uart_print("[FAIL] Unknown Error\r\n");
        }
        HAL_Delay(10);
    }

    uart_print("--------------------------------\r\n");

    // 2. 연속 쓰기 테스트 (명령 씹힘 확인용)
    // 0x55를 쓰고 바로 0xAA를 썼을 때 잘 바뀌는지 봅니다.
    Arduchip_WriteReg(ARDUCHIP_TEST1, 0x55);
    uint8_t val1 = Arduchip_ReadReg(ARDUCHIP_TEST1);

    Arduchip_WriteReg(ARDUCHIP_TEST1, 0xAA);
    uint8_t val2 = Arduchip_ReadReg(ARDUCHIP_TEST1);

    snprintf(buf, sizeof(buf), "0x55->0x%02X, 0xAA->0x%02X ", val1, val2);
    uart_print(buf);

    if (val1 == 0x55 && val2 == 0xAA) {
        uart_print("-> [PERFECT]\r\n");
    } else if (val1 == 0x55 && val2 == 0x55) {
        uart_print("-> [FAIL] Write Ignored (Slow down SPI)\r\n");
    } else {
        uart_print("-> [FAIL] Communication Broken\r\n");
    }

    uart_print("================================\r\n");
}

// === [중요] 초기화 함수 (엔진 시동 + 해상도 + 포맷) ===
static void Camera_Init_640x480_RGB565(void)
{
    uart_print("Init Camera 320x240 RGB565...\r\n");

    // 1. I2C로 센서 소프트웨어 리셋
    uint8_t val = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, OV5642_I2C_ADDR, 0x3008, I2C_MEMADD_SIZE_16BIT, &val, 1, 100);
    HAL_Delay(100);

    // 2. [추가됨] ArduChip(CPLD) 리셋 - 이게 없어서 FIFO가 안 찼던 겁니다!
    Arduchip_WriteReg(0x07, 0x80); // 리셋 시작
    HAL_Delay(100);
    Arduchip_WriteReg(0x07, 0x00); // 리셋 종료
    HAL_Delay(100);

    // 3. [추가됨] VSYNC 레벨 설정 (Active Low/High 맞추기)
    // ARDUCHIP_TIM(0x03) 레지스터의 1번 비트(0x02)를 1로 세팅
    Arduchip_WriteReg(0x03, 0x02);
    HAL_Delay(100);

    // 4. OV5642 센서 설정 로드 (QVGA)
    OV5642_LoadRegTable(OV5642_INIT_TABLE);


    // 5. FIFO 초기화
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

    uart_print("Init Done.\r\n");
}

// === 캡처 및 전송 함수 (수정됨) ===
void SendOneFrameOverUart(void)
{
    // 1. 캡처 시작
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // FIFO 초기화
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_START_MASK); // 캡처 시작

    // 2. 완료 대기 (텍스트 전송 금지)
    uint32_t t0 = HAL_GetTick();
    while ( !(Arduchip_ReadReg(ARDUCHIP_TRIG) & CAP_DONE_MASK) )
    {
        if (HAL_GetTick() - t0 > 2000) return; // 2초 지나면 포기
    }

    // 3. 길이 확인
    uint32_t len1 = Arduchip_ReadReg(FIFO_SIZE1);
    uint32_t len2 = Arduchip_ReadReg(FIFO_SIZE2);
    uint32_t len3 = Arduchip_ReadReg(FIFO_SIZE3) & 0x7f;
    uint32_t length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;

    // 길이가 0이면 전송 안 함 (PC 에러 방지)
    if (length == 0) return;

    // 4. 헤더 전송
    FrameHeader hdr;
    hdr.magic  = FRAME_MAGIC;
    hdr.width  = FRAME_WIDTH;
    hdr.height = FRAME_HEIGHT;
    hdr.format = 2; // RGB565
    hdr.reserved[0] = 0; hdr.reserved[1] = 0; hdr.reserved[2] = 0;

    HAL_UART_Transmit(&huart2, (uint8_t *)&hdr, sizeof(FrameHeader), HAL_MAX_DELAY);

    // 5. 이미지 데이터 전송
    CAM_CS_LOW();
    SPI1_TxRx(BURST_FIFO_READ);

    uint8_t chunk[2048];
    uint32_t remaining = length;

    while (remaining > 0)
    {
        uint32_t to_read = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;
        for (uint32_t i = 0; i < to_read; i++) chunk[i] = SPI1_TxRx(0x00);
        HAL_UART_Transmit(&huart2, chunk, to_read, HAL_MAX_DELAY);
        remaining -= to_read;
    }
    CAM_CS_HIGH();

    // 6. 정리
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

// PC 명령 처리
void ProcessUartCommand(void)
{
    uint8_t cmd;
    if (HAL_UART_Receive(&huart2, &cmd, 1, 10) == HAL_OK)
    {
        if (cmd == UART_CMD_CAPTURE) SendOneFrameOverUart();
    }
}

// (사용 안 함, 이전 코드 호환성용)
//static void Cam_Test_CS_Toggle(void) {}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI2) return;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 보드 LED 같은거
  s2_tx = 'A';
  SPI2_SlaveKick_1byte();
}

/*void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI2) return;



  // Zynq가 MOSI로 start cmd를 보내면 캡처 + 스트림 준비
  if (s2_state == S2_IDLE) {
    if (s2_rx == SPI2_CMD_START) {
      SPI2_PrepareNewFrame();
    }
    s2_tx = 0xFF;           // IDLE에서는 의미없는 바이트
    SPI2_SlaveKick_1byte();
    return;
  }

  // 상태 머신: “다음에 내보낼 바이트”를 s2_tx에 넣고 다시 1바이트 IT 재개
  switch (s2_state)
  {
    case S2_SEND_STATUS:
      s2_tx = SPI2_STATUS_OK;
      s2_state = S2_SEND_HDR;
      break;

    case S2_SEND_HDR: {
      uint8_t *p = (uint8_t*)&s2_hdr;
      s2_tx = p[s2_hdr_idx++];
      if (s2_hdr_idx >= sizeof(FrameHeader)) {
        s2_state = S2_SEND_LEN;
      }
    } break;

    case S2_SEND_LEN:
      s2_tx = s2_len_bytes[s2_len_idx++];
      if (s2_len_idx >= 4) {
        s2_state = S2_SEND_IMG;
      }
      break;

    case S2_SEND_IMG:
      // chunk 소진 시 다음 chunk를 FIFO에서 채움
      if (s2_chunk_idx >= s2_chunk_valid) {
        SPI2_FillNextChunk();
        if (s2_chunk_valid == 0) {
          // 끝
          CAM_CS_HIGH();
          Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
          s2_state = S2_DONE;
          s2_tx = 0xEE;
          break;
        }
      }

      s2_tx = s2_chunk[s2_chunk_idx++];
      s2_remaining--;
      if (s2_remaining == 0) {
        // 다음 콜백에서 마무리 처리되게 둠
      }
      break;

    case S2_DONE:
    default:
      // 전송 끝났으면 IDLE로 복귀
      s2_state = S2_IDLE;
      s2_tx = 0xFF;
      break;


  }


  SPI2_SlaveKick_1byte();
  return;

} */

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
