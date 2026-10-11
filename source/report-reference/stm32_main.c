/* Reference restoration from the original P3 report appendix.
 * Physical pages 70-94. Layout/page labels cleaned only.
 * Not the archived source file; not compiled or hardware-retested.
 * See docs/code-map.md and docs/reproduction.md for version/dependency gaps.
 */

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
   ******************************************************************************
   */
 /* USER CODE END Header */

 /* Includes ------------------------------------------------------------------*/
 #include "main.h"

 /* Private includes ----------------------------------------------------------*/
 /* USER CODE BEGIN Includes */
 #include <string.h>
 #include <stdio.h>
 #include <stdbool.h>
 /* USER CODE END Includes */

 /* Private typedef -----------------------------------------------------------*/
 /* USER CODE BEGIN PTD */
 typedef struct {
     uint16_t reg;
     uint8_t  val;
 } sensor_reg_t;

 /* Zynq <-> STM32 protocol (matches your Zynq code) */
 typedef struct __attribute__((packed)) {
     uint32_t magic;     // 0x1234ABCD
     uint16_t w;         // 320
     uint16_t h;         // 240
     uint32_t total;     // 153600 (or 0 if not ready)
     uint32_t remain;    // remaining bytes (or 0)
 } InfoResp;
 /* USER CODE END PTD */

 /* Private define ------------------------------------------------------------*/

/* Source report physical page 71 */
/* USER CODE BEGIN PD */

// ------------------ Ultrasonic ------------------
#define SOUND_SPEED_CM_PER_US  0.0343f
#define DIST_LED_TH_CM         30.0f

// ------------------ Zynq handshake pins ------------------
// ALARM: STM32 -> Zynq (IRQ input)
#define ZYNQ_ALARM_GPIO_PORT   GPIOA
#define ZYNQ_ALARM_PIN         GPIO_PIN_4   // PA4

// BUSY: Zynq -> STM32 (optional, if Zynq actually drives it)
#define ZYNQ_BUSY_GPIO_PORT    GPIOB
#define ZYNQ_BUSY_PIN          GPIO_PIN_0   // PB0

// READY: STM32 -> Zynq (Zynq polls before reading RESP)
#define ZYNQ_READY_GPIO_PORT   GPIOB
#define ZYNQ_READY_PIN         GPIO_PIN_1   // PB1  (*** added ***)

// ------------------ ArduCam / OV5642 ------------------
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

// ArduCam CS (SPI1 master uses this)
#define CAM_CS_GPIO_Port     GPIOB
#define CAM_CS_Pin           GPIO_PIN_6
#define CAM_CS_LOW()         HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin,
GPIO_PIN_RESET)
#define CAM_CS_HIGH()        HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin,
GPIO_PIN_SET)

// ------------------ Frame fixed size (Zynq expects EXACT) ------------------
#define FRAME_WIDTH       320u
#define FRAME_HEIGHT      240u
#define BYTES_PER_PIXEL   2u
#define FRAME_SIZE_BYTES  (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)  //

#define CHUNK_SIZE        2048u
#define NUM_CHUNKS        (FRAME_SIZE_BYTES / CHUNK_SIZE)                  // 75

#if ((FRAME_SIZE_BYTES % CHUNK_SIZE) != 0)

/* Source report physical page 72 */
#error "FRAME_SIZE_BYTES must be multiple of CHUNK_SIZE"
#endif

#define FRAME_MAGIC       0x1234ABCDu

// ------------------ Zynq protocol commands ------------------
#define CMD_INFO   0x10
#define CMD_CHUNK  0x11
#define CMD_RESET  0x12

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
static volatile uint8_t  capture_state = 0;
static volatile uint32_t echo_us = 0;
static volatile uint8_t  echo_done = 0;

// 거리
static volatile float distance_cm = 0.0f;

// PC UART capture flag (optional)
static uint8_t pc_capture_requested = 0;

// ------------------ Camera frame / FIFO streaming state ------------------
static volatile uint8_t  g_capture_pending = 0;     // set by RESET or by alarm
logic
static volatile uint8_t  g_frame_ready     = 0;     // set when capture done
static volatile uint8_t  g_streaming       = 0;     // set when chunk streaming
started
static volatile uint32_t g_chunks_sent     = 0;
static volatile uint32_t g_bytes_remain    = 0;
static volatile uint8_t  g_cam_burst_active = 0;    // keep CAM_CS low + burst
mode during whole frame

/* Source report physical page 73 */
// ------------------ SPI2 slave protocol engine ------------------
typedef enum {
  SPI2_ST_WAIT_CMD = 0,
  SPI2_ST_RESP_INFLIGHT
} spi2_state_t;

static volatile spi2_state_t g_spi2_state = SPI2_ST_WAIT_CMD;

// command rx (1 byte)
static uint8_t g_cmd_rx = 0x00;
static uint8_t g_cmd_tx = 0xFF; // dummy output during CMD phase

static volatile uint8_t g_cmd_pending = 0;
static volatile uint8_t g_pending_cmd = 0;

// response buffers
static uint8_t g_resp_ok[4] = { 'O','K','\r','\n' };
static uint8_t g_resp_err[4] = { 'E','R','R','\n' };
static uint8_t g_resp_info[16];
static uint8_t g_resp_chunk[CHUNK_SIZE];

// RX discard for response (SPI is full-duplex)
static uint8_t g_resp_rx_discard[CHUNK_SIZE];

// response control
static uint8_t  *g_resp_ptr = NULL;
static uint16_t  g_resp_len = 0;
static volatile uint8_t g_cleanup_after_resp = 0;

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
// camera
static void Check_VSYNC_Signal(void);
static void I2C1_ScanBus(void);
static void Cam_Read_OV5642_ID(void);
static void Arduchip_Test(void);
static void Camera_Init_640x480_RGB565(void);
void ProcessUartCommand(void);
static void uart_print(const char *s);
void SendOneFrameOverUart(void);

/* Source report physical page 74 */
// ultrasonic
static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);
static void HCSR04_Trigger(void);
static float HCSR04_Read_cm(uint32_t timeout_ms);

// low-level cam SPI
static uint8_t SPI1_TxRx(uint8_t data);
static void Arduchip_WriteReg(uint8_t addr, uint8_t value);
static uint8_t Arduchip_ReadReg(uint8_t addr);
static HAL_StatusTypeDef OV5642_LoadRegTable(const sensor_reg_t *table);

// zynq handshake
static inline bool Zynq_IsBusy(void);
static inline void Zynq_AlarmHigh(void);
static inline void Zynq_AlarmLow(void);
static inline void Zynq_ReadyHigh(void);
static inline void Zynq_ReadyLow(void);
static bool Zynq_SendAlarmPulse_IfIdle(uint32_t pulse_us);

// protocol worker
static void SPI2_ArmCmdRx_1byte(void);
static void SPI2_PrepareResponse(uint8_t *tx, uint16_t len);
static void SPI2_ProcessPendingCmd_inMain(void);
static void Camera_CaptureToFIFO_Blocking(void);
static void Camera_BurstRead_FillChunk(uint8_t *dst, uint32_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ★★★ [핵심] OV5642 초기화 레지스터 (당신이 준 그대로) ★★★
const sensor_reg_t OV5642_INIT_TABLE[] = {
    {0x3103,0x93}, {0x3008,0x82}, {0x3017,0x7f}, {0x3018,0xfc}, {0x3810,0xc2},
{0x3615,0xf0},
    {0x3000,0x00}, {0x3001,0x00}, {0x3002,0x5c}, {0x3003,0x00}, {0x3004,0xff},
{0x3005,0xff},
    {0x3006,0x43}, {0x3007,0x37}, {0x3011,0x08}, {0x3010,0x10}, {0x460c,0x22},
{0x3815,0x04},
    {0x370c,0xa0}, {0x3602,0xfc}, {0x3612,0xff}, {0x3634,0xc0}, {0x3613,0x00},
{0x3605,0x7c},
    {0x3621,0x09}, {0x3622,0x60}, {0x3604,0x40}, {0x3603,0xa7}, {0x3603,0x27},
{0x4000,0x21},
    {0x401d,0x22}, {0x3600,0x54}, {0x3605,0x04}, {0x3606,0x3f}, {0x3c01,0x80},
{0x5000,0x4f},
    {0x5020,0x04}, {0x5181,0x79}, {0x5182,0x00}, {0x5185,0x22}, {0x5197,0x01},
{0x5001,0xff},
    {0x5500,0x0a}, {0x5504,0x00}, {0x5505,0x7f}, {0x5080,0x08}, {0x300e,0x18},
{0x4610,0x00},
    {0x471d,0x05}, {0x4708,0x06}, {0x3808,0x02}, {0x3809,0x80}, {0x380a,0x01},

/* Source report physical page 75 */
{0x380b,0xe0},
    {0x380e,0x07}, {0x380f,0xd0}, {0x501f,0x00}, {0x5000,0x4f}, {0x4300,0x30},
{0x3503,0x07},
    {0x3501,0x73}, {0x3502,0x80}, {0x350b,0x00}, {0x3503,0x07}, {0x3824,0x11},
{0x3501,0x1e},
    {0x3502,0x80}, {0x350b,0x7f}, {0x380c,0x0c}, {0x380d,0x80}, {0x380e,0x03},
{0x380f,0xe8},
    {0x3a0d,0x04}, {0x3a0e,0x03}, {0x3818,0xc1}, {0x3705,0xdb}, {0x370a,0x81},
{0x3801,0x80},
    {0x3621,0x87}, {0x3801,0x50}, {0x3803,0x08}, {0x3827,0x08}, {0x3810,0x40},
{0x3804,0x05},
    {0x3805,0x00}, {0x5682,0x05}, {0x5683,0x00}, {0x3806,0x03}, {0x3807,0xc0},
{0x5686,0x03},
    {0x5687,0xbc}, {0x3a00,0x78}, {0x3a1a,0x05}, {0x3a13,0x30}, {0x3a18,0x00},
{0x3a19,0x7c},
    {0x3a08,0x12}, {0x3a09,0xc0}, {0x3a0a,0x0f}, {0x3a0b,0xa0}, {0x350c,0x07},
{0x350d,0xd0},
    {0x3500,0x00}, {0x3501,0x00}, {0x3502,0x00}, {0x350a,0x00}, {0x350b,0x00},
{0x3503,0x00},
    {0x528a,0x02}, {0x528b,0x04}, {0x528c,0x08}, {0x528d,0x08}, {0x528e,0x08},
{0x528f,0x10},
    {0x5290,0x10}, {0x5292,0x00}, {0x5293,0x02}, {0x5294,0x00}, {0x5295,0x02},
{0x5296,0x00},
    {0x5297,0x02}, {0x5298,0x00}, {0x5299,0x02}, {0x529a,0x00}, {0x529b,0x02},
{0x529c,0x00},
    {0x529d,0x02}, {0x529e,0x00}, {0x529f,0x02}, {0x3030,0x0b}, {0x3a02,0x00},
{0x3a03,0x7d},
    {0x3a04,0x00}, {0x3a14,0x00}, {0x3a15,0x7d}, {0x3a16,0x00}, {0x3a00,0x78},
{0x3a08,0x09},
    {0x3a09,0x60}, {0x3a0a,0x07}, {0x3a0b,0xd0}, {0x3a0d,0x08}, {0x3a0e,0x06},
{0x5193,0x70},
    {0x589b,0x04}, {0x589a,0xc5}, {0x401e,0x20}, {0x4001,0x42}, {0x401c,0x04},
{0x528a,0x01},
    {0x528b,0x04}, {0x528c,0x08}, {0x528d,0x10}, {0x528e,0x20}, {0x528f,0x28},
{0x5290,0x30},
    {0x5292,0x00}, {0x5293,0x01}, {0x5294,0x00}, {0x5295,0x04}, {0x5296,0x00},
{0x5297,0x08},
    {0x5298,0x00}, {0x5299,0x10}, {0x529a,0x00}, {0x529b,0x20}, {0x529c,0x00},
{0x529d,0x28},
    {0x529e,0x00}, {0x529f,0x30}, {0x5282,0x00}, {0x5300,0x00}, {0x5301,0x20},
{0x5302,0x00},
    {0x5303,0x7c}, {0x530c,0x00}, {0x530d,0x0c}, {0x530e,0x20}, {0x530f,0x80},
{0x5310,0x20},
    {0x5311,0x80}, {0x5308,0x20}, {0x5309,0x40}, {0x5304,0x00}, {0x5305,0x30},
{0x5306,0x00},
    {0x5307,0x80}, {0x5314,0x08}, {0x5315,0x20}, {0x5319,0x30}, {0x5316,0x10},
{0x5317,0x00},
    {0x5318,0x02}, {0x5402,0x3f}, {0x5403,0x00}, {0x3406,0x00}, {0x5180,0xff},
{0x5181,0x52},
    {0x5182,0x11}, {0x5183,0x14}, {0x5184,0x25}, {0x5185,0x24}, {0x5186,0x06},
{0x5187,0x08},

/* Source report physical page 76 */
{0x5188,0x08}, {0x5189,0x7c}, {0x518a,0x60}, {0x518b,0xb2}, {0x518c,0xb2},
{0x518d,0x44},
    {0x518e,0x3d}, {0x518f,0x58}, {0x5190,0x46}, {0x5191,0xf8}, {0x5192,0x04},
{0x5193,0x70},
    {0x5194,0xf0}, {0x5195,0xf0}, {0x5196,0x03}, {0x5197,0x01}, {0x5198,0x04},
{0x5199,0x12},
    {0x519a,0x04}, {0x519b,0x00}, {0x519c,0x06}, {0x519d,0x82}, {0x519e,0x00},
{0x5025,0x80},
    {0x5583,0x40}, {0x5584,0x40}, {0x5580,0x02}, {0x5000,0xcf}, {0x3710,0x10},
{0x3632,0x51},
    {0x3702,0x10}, {0x3703,0xb2}, {0x3704,0x18}, {0x370b,0x40}, {0x370d,0x03},
{0x3631,0x01},
    {0x3632,0x52}, {0x3606,0x24}, {0x3620,0x96}, {0x5785,0x07}, {0x3a13,0x30},
{0x3600,0x52},
    {0x3604,0x48}, {0x3606,0x1b}, {0x370d,0x0b}, {0x370f,0xc0}, {0x3709,0x01},
{0x3823,0x00},
    {0x5007,0x00}, {0x5009,0x00}, {0x5011,0x00}, {0x5013,0x00}, {0x519e,0x00},
{0x5086,0x00},
    {0x5087,0x00}, {0x5088,0x00}, {0x5089,0x00}, {0x302b,0x00}, {0x3808,0x01},
{0x3809,0x40},
    {0x380a,0x00}, {0x380b,0xf0}, {0x3a00,0x78}, {0x5001,0xFF}, {0x5583,0x50},
{0x5584,0x50},
    {0x5580,0x02}, {0x3c01,0x80}, {0x3c00,0x04}, {0xFFFF, 0xFF}
};

// ---------------- DWT us delay ----------------
static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {}
}

// ---------------- HC-SR04 ----------------
static void HCSR04_Trigger(void)
{
    // PA1 = TRIG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    delay_us(2);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

/* Source report physical page 77 */
static float HCSR04_Read_cm(uint32_t timeout_ms)
{
    echo_done = 0;
    capture_state = 0;
    echo_us = 0;

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    HCSR04_Trigger();

    uint32_t t0 = HAL_GetTick();
    while (!echo_done) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
            return -1.0f;
        }
    }

    float cm = (echo_us * SOUND_SPEED_CM_PER_US) / 2.0f;
    return cm;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (capture_state == 0) {
            ic_rise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            capture_state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            ic_fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (ic_fall >= ic_rise) echo_us = (ic_fall - ic_rise);
            else echo_us = ((htim->Init.Period + 1U) - ic_rise) + ic_fall;

            echo_done = 1;
            capture_state = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
        }
    }
}

// ---------------- UART helper ----------------

/* Source report physical page 78 */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

// ---------------- I2C table load ----------------
static HAL_StatusTypeDef OV5642_LoadRegTable(const sensor_reg_t *table)
{
    HAL_StatusTypeDef ret;
    int i = 0;

    while (1) {
        uint16_t reg = table[i].reg;
        uint8_t  val = table[i].val;

        if (reg == 0xFFFF && val == 0xFF) break;

        ret = HAL_I2C_Mem_Write(&hi2c1, OV5642_I2C_ADDR, reg,
                                I2C_MEMADD_SIZE_16BIT, &val, 1, 100);
        if (ret != HAL_OK) {
            char buf[96];
            snprintf(buf, sizeof(buf),
                     "OV5642_LoadRegTable fail reg=0x%04X val=0x%02X
err=0x%08lX\r\n",
                     reg, val, HAL_I2C_GetError(&hi2c1));
            uart_print(buf);
            return ret;
        }
        HAL_Delay(1);
        i++;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef OV5642_ReadReg(uint16_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1, OV5642_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_16BIT, value, 1, 100);
}

// ---------------- camera id / scan / test ----------------
static void Cam_Read_OV5642_ID(void)
{
    uint8_t id_high = 0, id_low = 0;

    if (HAL_I2C_IsDeviceReady(&hi2c1, OV5642_I2C_ADDR, 3, 100) != HAL_OK) {
        uart_print("I2C Device NOT Ready!\r\n");
        return;
    }

    OV5642_ReadReg(0x300A, &id_high);

/* Source report physical page 79 */
OV5642_ReadReg(0x300B, &id_low);

    char buf[64];
    snprintf(buf, sizeof(buf), "OV5642 ID = 0x%02X 0x%02X (expect 0x56
0x42)\r\n",
             id_high, id_low);
    uart_print(buf);
}

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

// ---------------- SPI1 low-level ----------------
static uint8_t SPI1_TxRx(uint8_t data)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void Arduchip_WriteReg(uint8_t addr, uint8_t value)
{
    CAM_CS_LOW();
    HAL_Delay(1);
    SPI1_TxRx(addr | 0x80);
    SPI1_TxRx(value);
    CAM_CS_HIGH();
    HAL_Delay(1);
}

static uint8_t Arduchip_ReadReg(uint8_t addr)
{
    uint8_t val;
    CAM_CS_LOW();
    HAL_Delay(1);
    SPI1_TxRx(addr & 0x7F);
    val = SPI1_TxRx(0x00);
    CAM_CS_HIGH();
    return val;

/* Source report physical page 80 */
}

static void Arduchip_Test(void)
{
    char buf[64];
    uart_print("\r\n================================\r\n");
    uart_print("   SPI DIAGNOSTIC TEST START    \r\n");
    uart_print("================================\r\n");

    uint8_t test_patterns[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

    for (int i=0;i<8;i++) {
        uint8_t target = test_patterns[i];
        Arduchip_WriteReg(ARDUCHIP_TEST1, target);
        uint8_t read_back = Arduchip_ReadReg(ARDUCHIP_TEST1);

        snprintf(buf,sizeof(buf),"Write: 0x%02X -> Read: 0x%02X ", target,
read_back);
        uart_print(buf);

        if (read_back == target) uart_print("[OK]\r\n");
        else if (read_back == (uint8_t)(target<<1)) uart_print("[FAIL] Left
Shift (Try 2EDGE)\r\n");
        else if (read_back == (uint8_t)(target>>1)) uart_print("[FAIL] Right
Shift (Try 1EDGE)\r\n");
        else if (read_back == 0x00 || read_back == 0xFF) uart_print("[FAIL]
Stuck (Wiring/Power)\r\n");
        else uart_print("[FAIL] Unknown\r\n");

        HAL_Delay(10);
    }

    uart_print("--------------------------------\r\n");
    Arduchip_WriteReg(ARDUCHIP_TEST1, 0x55);
    uint8_t val1 = Arduchip_ReadReg(ARDUCHIP_TEST1);
    Arduchip_WriteReg(ARDUCHIP_TEST1, 0xAA);
    uint8_t val2 = Arduchip_ReadReg(ARDUCHIP_TEST1);

    snprintf(buf,sizeof(buf),"0x55->0x%02X, 0xAA->0x%02X ", val1, val2);
    uart_print(buf);

    if (val1==0x55 && val2==0xAA) uart_print("-> [PERFECT]\r\n");
    else if (val1==0x55 && val2==0x55) uart_print("-> [FAIL] Write Ignored (Slow
SPI)\r\n");
    else uart_print("-> [FAIL] Broken\r\n");

    uart_print("================================\r\n");
}

// ---------------- VSYNC diag ----------------
static void Check_VSYNC_Signal(void)

/* Source report physical page 81 */
{
    uart_print("\r\n[DIAGNOSTIC] Checking VSYNC Signal (1 sec)...\r\n");
    uint32_t high_cnt=0, low_cnt=0;
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < 1000) {
        uint8_t status = Arduchip_ReadReg(ARDUCHIP_TRIG);
        if (status & 0x01) high_cnt++;
        else low_cnt++;
        for (volatile int i=0;i<1000;i++);
    }

    char buf[64];
    snprintf(buf,sizeof(buf)," -> High: %lu, Low: %lu\r\n", high_cnt, low_cnt);
    uart_print(buf);

    if (high_cnt>0 && low_cnt>0) uart_print(" -> [RESULT] VSYNC Toggling! SENSOR
ALIVE.\r\n");
    else uart_print(" -> [RESULT] VSYNC Dead.\r\n");

    uart_print("------------------------------------------\r\n");
}

// ---------------- Camera init ----------------
static void Camera_Init_640x480_RGB565(void)
{
    uart_print("Init Camera 320x240 RGB565...\r\n");

    uint8_t val = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, OV5642_I2C_ADDR, 0x3008, I2C_MEMADD_SIZE_16BIT,
&val, 1, 100);
    HAL_Delay(100);

    Arduchip_WriteReg(0x07, 0x80);
    HAL_Delay(100);
    Arduchip_WriteReg(0x07, 0x00);
    HAL_Delay(100);

    Arduchip_WriteReg(0x03, 0x02);
    HAL_Delay(100);

    OV5642_LoadRegTable(OV5642_INIT_TABLE);

    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

    uart_print("Init Done.\r\n");
}

// ---------------- Optional PC UART capture ----------------
void SendOneFrameOverUart(void)
{

/* Source report physical page 82 */
Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_START_MASK);

    uint32_t t0 = HAL_GetTick();
    while (!(Arduchip_ReadReg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
        if (HAL_GetTick() - t0 > 2000) return;
    }

    // read fifo len
    uint32_t len1 = Arduchip_ReadReg(FIFO_SIZE1);
    uint32_t len2 = Arduchip_ReadReg(FIFO_SIZE2);
    uint32_t len3 = Arduchip_ReadReg(FIFO_SIZE3) & 0x7f;
    uint32_t length = ((len3<<16) | (len2<<8) | len1) & 0x07fffff;
    if (length == 0) return;

    // header (PC-only)
    typedef struct __attribute__((packed)) {
        uint32_t magic;
        uint16_t width;
        uint16_t height;
        uint8_t  format;
        uint8_t  reserved[3];
    } FrameHeader;

    FrameHeader hdr;
    hdr.magic = FRAME_MAGIC;
    hdr.width = FRAME_WIDTH;
    hdr.height= FRAME_HEIGHT;
    hdr.format= 2;
    hdr.reserved[0]=hdr.reserved[1]=hdr.reserved[2]=0;

    HAL_UART_Transmit(&huart2, (uint8_t*)&hdr, sizeof(hdr), HAL_MAX_DELAY);

    CAM_CS_LOW();
    SPI1_TxRx(BURST_FIFO_READ);

    uint8_t chunk[2048];
    uint32_t remaining = length;
    while (remaining > 0) {
        uint32_t to_read = (remaining > sizeof(chunk)) ? sizeof(chunk) :
remaining;
        for (uint32_t i=0;i<to_read;i++) chunk[i] = SPI1_TxRx(0x00);
        HAL_UART_Transmit(&huart2, chunk, to_read, HAL_MAX_DELAY);
        remaining -= to_read;
    }
    CAM_CS_HIGH();

    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ProcessUartCommand(void)

/* Source report physical page 83 */
{
    uint8_t cmd;
    if (HAL_UART_Receive(&huart2, &cmd, 1, 10) == HAL_OK) {
        pc_capture_requested = 1;
        uart_print("Waiting for Object...\r\n");
    }
}

// ---------------- Zynq handshake helpers ----------------
static inline bool Zynq_IsBusy(void)
{
    return (HAL_GPIO_ReadPin(ZYNQ_BUSY_GPIO_PORT, ZYNQ_BUSY_PIN) ==
GPIO_PIN_SET);
}
static inline void Zynq_AlarmHigh(void) {
HAL_GPIO_WritePin(ZYNQ_ALARM_GPIO_PORT, ZYNQ_ALARM_PIN, GPIO_PIN_SET); }
static inline void Zynq_AlarmLow(void)  {
HAL_GPIO_WritePin(ZYNQ_ALARM_GPIO_PORT, ZYNQ_ALARM_PIN, GPIO_PIN_RESET); }
static inline void Zynq_ReadyHigh(void) {
HAL_GPIO_WritePin(ZYNQ_READY_GPIO_PORT, ZYNQ_READY_PIN, GPIO_PIN_SET); }
static inline void Zynq_ReadyLow(void)  {
HAL_GPIO_WritePin(ZYNQ_READY_GPIO_PORT, ZYNQ_READY_PIN, GPIO_PIN_RESET); }

static bool Zynq_SendAlarmPulse_IfIdle(uint32_t pulse_us)
{
    if (Zynq_IsBusy()) return false;
    Zynq_AlarmHigh();
    delay_us(pulse_us);
    Zynq_AlarmLow();
    return true;
}

// ---------------- Camera capture for Zynq protocol ----------------
// NOTE: capture is done in main loop (blocking) to avoid doing heavy work in
SPI ISR.
static void Camera_CaptureToFIFO_Blocking(void)
{
    // stop any previous burst session
    if (g_cam_burst_active) {
        CAM_CS_HIGH();
        g_cam_burst_active = 0;
    }

    // clear & start capture
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
    Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_START_MASK);

    uint32_t t0 = HAL_GetTick();
    while (!(Arduchip_ReadReg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
        if (HAL_GetTick() - t0 > 2000) {
            g_frame_ready = 0;

/* Source report physical page 84 */
return;
        }
    }

    // (optional) length check
    uint32_t len1 = Arduchip_ReadReg(FIFO_SIZE1);
    uint32_t len2 = Arduchip_ReadReg(FIFO_SIZE2);
    uint32_t len3 = Arduchip_ReadReg(FIFO_SIZE3) & 0x7f;
    uint32_t length = ((len3<<16) | (len2<<8) | len1) & 0x07fffff;

    // We will stream EXACT 153600 bytes to match Zynq.
    // If length is smaller, Zynq will still read 153600 -> remaining part will
become 0x00 (we pad).
    // If length is larger, we ignore extra.
    (void)length;

    g_chunks_sent  = 0;
    g_bytes_remain = FRAME_SIZE_BYTES;
    g_frame_ready  = 1;
    g_streaming    = 0;
}

// Fill chunk by reading from FIFO sequentially.
// We keep CAM_CS low + BURST mode across whole frame (so pointer does not
reset).
static void Camera_BurstRead_FillChunk(uint8_t *dst, uint32_t len)
{
    if (!g_cam_burst_active) {
        CAM_CS_LOW();
        SPI1_TxRx(BURST_FIFO_READ);
        g_cam_burst_active = 1;
    }

    for (uint32_t i=0;i<len;i++) {
        dst[i] = SPI1_TxRx(0x00);
    }
}

// ---------------- SPI2 protocol engine ----------------
static void SPI2_ArmCmdRx_1byte(void)
{
    // Arm 1-byte TransmitReceive in IT mode (cmd phase)
    // Zynq does: CS low, 1 byte clock, CS high
    HAL_SPI_TransmitReceive_IT(&hspi2, &g_cmd_tx, &g_cmd_rx, 1);
}

static void SPI2_PrepareResponse(uint8_t *tx, uint16_t len)
{
    // Arm response DMA first, then raise READY.
    // Zynq waits READY=1 before doing CS low + clocks.
    g_resp_ptr = tx;

/* Source report physical page 85 */
g_resp_len = len;

    // clear READY while preparing
    Zynq_ReadyLow();

    // safety: cap rx discard buffer
    if (len > CHUNK_SIZE) len = CHUNK_SIZE;

    if (HAL_SPI_TransmitReceive_DMA(&hspi2, g_resp_ptr, g_resp_rx_discard, len)
!= HAL_OK) {
        // fail -> respond ERR next time
        g_cmd_pending = 0;
        g_resp_ptr = NULL;
        g_resp_len = 0;
        Zynq_ReadyLow();
        SPI2_ArmCmdRx_1byte();
        return;
    }

    g_spi2_state = SPI2_ST_RESP_INFLIGHT;
    Zynq_ReadyHigh();
}

static void SPI2_ProcessPendingCmd_inMain(void)
{
    if (!g_cmd_pending) return;
    if (g_spi2_state != SPI2_ST_WAIT_CMD) return;

    uint8_t cmd = g_pending_cmd;
    g_cmd_pending = 0;

    // Default: no cleanup after response
    g_cleanup_after_resp = 0;

    if (cmd == CMD_RESET) {
        // ACK quickly; start capture in main loop after this response.
        // Reset "availability" first so INFO returns 0 while capturing.
        g_frame_ready  = 0;
        g_streaming    = 0;
        g_chunks_sent  = 0;
        g_bytes_remain = 0;
        g_capture_pending = 1;

        SPI2_PrepareResponse(g_resp_ok, 4);
        return;
    }

    if (cmd == CMD_INFO) {
        InfoResp info;
        info.magic = FRAME_MAGIC;
        info.w     = FRAME_WIDTH;

/* Source report physical page 86 */
info.h     = FRAME_HEIGHT;

        if (g_frame_ready) {
            info.total  = FRAME_SIZE_BYTES;
            info.remain = g_bytes_remain;
        } else {
            info.total  = 0;
            info.remain = 0;
        }

        // pack to little-endian bytes (STM32 is little-endian -> memcpy OK)
        memcpy(g_resp_info, &info, sizeof(InfoResp));

        SPI2_PrepareResponse(g_resp_info, 16);
        return;
    }

    if (cmd == CMD_CHUNK) {
        if (!g_frame_ready || g_bytes_remain == 0 || g_chunks_sent >=
NUM_CHUNKS) {
            // Not ready -> send 0-filled chunk to keep master in sync (avoid
deadlock)
            memset(g_resp_chunk, 0, CHUNK_SIZE);
            SPI2_PrepareResponse(g_resp_chunk, CHUNK_SIZE);
            return;
        }

        // Ensure streaming started
        g_streaming = 1;

        // Fill next chunk from FIFO
        uint32_t to_read = CHUNK_SIZE;
        Camera_BurstRead_FillChunk(g_resp_chunk, to_read);

        g_chunks_sent++;
        if (g_bytes_remain >= CHUNK_SIZE) g_bytes_remain -= CHUNK_SIZE;
        else g_bytes_remain = 0;

        // After last chunk, cleanup AFTER the response DMA completes
        if (g_chunks_sent >= NUM_CHUNKS || g_bytes_remain == 0) {
            g_cleanup_after_resp = 1;
        }

        SPI2_PrepareResponse(g_resp_chunk, CHUNK_SIZE);
        return;
    }

    // Unknown cmd
    SPI2_PrepareResponse(g_resp_err, 4);
}

/* Source report physical page 87 */
/* SPI2 callbacks:
   - CMD phase completes here -> set pending cmd
   - RESP phase DMA completes -> lower READY, cleanup, re-arm CMD receive
*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) return;

    if (g_spi2_state == SPI2_ST_WAIT_CMD) {
        // 1-byte CMD received
        g_pending_cmd = g_cmd_rx;
        g_cmd_pending = 1;
        // Do NOT arm another SPI operation here. Main loop will arm RESP DMA.
        return;
    }

    // IT completion for 1-byte cmd path can also land here depending on HAL.
    // But response is DMA, so DMA completion uses HAL_SPI_TxRxCpltCallback as
well.
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) return;

    // Abort and recover
    Zynq_ReadyLow();
    g_spi2_state = SPI2_ST_WAIT_CMD;
    g_cmd_pending = 0;
    g_cleanup_after_resp = 0;

    // stop burst if needed (safe)
    if (g_cam_burst_active) {
        CAM_CS_HIGH();
        g_cam_burst_active = 0;
    }

    SPI2_ArmCmdRx_1byte();
}

void HAL_SPI_TxRxCpltCallback_DMA(SPI_HandleTypeDef *hspi); // not used (HAL
version dependent)

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;

/* Source report physical page 88 */
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi); // already declared
above

// DMA complete hook: some HALs call HAL_SPI_TxRxCpltCallback for DMA too.
// To be robust, also use HAL_SPI_TxRxCpltCallback and check current state in
main loop.
// We'll finalize DMA completion in HAL_SPI_TxRxCpltCallback by checking SPI2
state from IRQ context is messy,
// so instead we use HAL_SPI_TxRxCpltCallback itself for both; handle DMA
completion here:
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) return;

    if (g_spi2_state == SPI2_ST_WAIT_CMD) {
        // CMD phase complete
        g_pending_cmd = g_cmd_rx;
        g_cmd_pending = 1;
        return;
    }

    // RESP DMA complete
    Zynq_ReadyLow();

    if (g_cleanup_after_resp) {
        // End burst session and clear FIFO (frame consumed)
        if (g_cam_burst_active) {
            CAM_CS_HIGH();
            g_cam_burst_active = 0;
        }
        Arduchip_WriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

        g_cleanup_after_resp = 0;
        g_frame_ready = 0;
        g_streaming   = 0;
        g_chunks_sent = 0;
        g_bytes_remain= 0;
    }

    g_spi2_state = SPI2_ST_WAIT_CMD;

    // re-arm next CMD
    SPI2_ArmCmdRx_1byte();
}

/* Source report physical page 89 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  CAM_CS_HIGH();
  uart_print("UART ALIVE CHECK\r\n");

  I2C1_ScanBus();
  Cam_Read_OV5642_ID();
  Arduchip_Test();

  Camera_Init_640x480_RGB565();
  Check_VSYNC_Signal();
  uart_print("Ready.\r\n");

  DWT_Delay_Init();

  // READY default low
  Zynq_ReadyLow();
  Zynq_AlarmLow();

  // Arm first CMD receive
  g_spi2_state = SPI2_ST_WAIT_CMD;
  SPI2_ArmCmdRx_1byte();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1) SPI2 command engine (main-thread does heavy work)
    SPI2_ProcessPendingCmd_inMain();

/* Source report physical page 90 */
// 2) If capture requested (by CMD_RESET), do capture here (blocking)
    //    (SPI2 still works via IRQ/DMA while we are here)
    if (g_capture_pending && (g_spi2_state == SPI2_ST_WAIT_CMD)) {
        // Don't capture while streaming (safety)
        if (!g_streaming) {
            g_capture_pending = 0;
            uart_print("[CAPTURE] start\r\n");
            Camera_CaptureToFIFO_Blocking();
            if (g_frame_ready) {
                uart_print("[CAPTURE] done -> frame_ready=1\r\n");
            } else {
                uart_print("[CAPTURE] fail\r\n");
            }
        }
    }

    // 3) Ultrasonic demo: send alarm pulse when object is close AND Zynq not
busy.
    //    We avoid spamming: only pulse when NOT already
frame_ready/capturing/streaming.
    if (!g_frame_ready && !g_capture_pending && !g_streaming) {
        float cm = HCSR04_Read_cm(60);
        distance_cm = cm;

        if (cm > 0.0f && cm <= DIST_LED_TH_CM) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // local LED ON
            // pulse to wake Zynq (Zynq will send CMD_RESET after IRQ)
            (void)Zynq_SendAlarmPulse_IfIdle(50);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // OFF
        }

        // Optional PC capture path
        if (pc_capture_requested == 1) {
            if (cm > 0.0f && cm <= 30.0f) {
                uart_print("Object Detected! Capturing...\r\n");
                SendOneFrameOverUart();
                pc_capture_requested = 0;
            }
        }
        ProcessUartCommand();

        HAL_Delay(50);
    } else {
        // when busy with Zynq protocol, keep loop responsive
        HAL_Delay(1);
    }

  }
  /* USER CODE END WHILE */

/* Source report physical page 91 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
Error_Handler(); }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI1 Initialization Function
  */

/* Source report physical page 92 */
static void MX_SPI1_Init(void)
{
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI2 Initialization Function
  */
static void MX_SPI2_Init(void)
{
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
  if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

/* Source report physical page 93 */
if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
Error_Handler(); }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
Error_Handler(); }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
Error_Handler(); }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PA1(TRIG), PA4(ALARM), PA8(LED)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);
  // PB6(CAM_CS)

/* Source report physical page 94 */
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
   // PB1(READY)
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

   GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   // PB0 BUSY input
   GPIO_InitStruct.Pin = GPIO_PIN_0;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_PULLDOWN;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   // PB1 READY output (added)
   GPIO_InitStruct.Pin = GPIO_PIN_1;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   // PB6 CAM_CS output
   GPIO_InitStruct.Pin = GPIO_PIN_6;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 }

 /**
   * @brief  This function is executed in case of error occurrence.
   */
 void Error_Handler(void)
 {
   __disable_irq();
   while (1) {}
 }

 #ifdef  USE_FULL_ASSERT
 void assert_failed(uint8_t *file, uint32_t line)
 {
   (void)file; (void)line;
 }
 #endif /* USE_FULL_ASSERT */
