/* Reference restoration from the original P3 report appendix.
 * Physical pages 95-109. Layout/page labels cleaned only.
 * Not the archived source file; not compiled or hardware-retested.
 * See docs/code-map.md and docs/reproduction.md for version/dependency gaps.
 */

#include <stdio.h>
 #include <stdint.h>
 #include <stddef.h>
 #include <math.h>
 #include <string.h>

 /* Xilinx / BSP */
 #include "xparameters.h"
 #include "xscugic.h"
 #include "xgpiops.h"
 #include "xspips.h"
 #include "xil_printf.h"
 #include "xil_cache.h"
 #include "xil_exception.h"

 /* FreeRTOS */
 #include "FreeRTOS.h"
 #include "task.h"
 #include "semphr.h"
 #include "queue.h"

 /* FatFs */
 #include "ff.h"

 /* Your CNN weights */
 #include "tinycnn3_weights.h"

 /* ----------------------------
  * Hardware IDs / Pins (EDIT THESE)
  * ---------------------------- */
 #define SPI_DEVICE_ID      XPAR_XSPIPS_0_DEVICE_ID
 #define INTC_DEVICE_ID     XPAR_SCUGIC_SINGLE_DEVICE_ID
 #define GPIO_DEVICE_ID     XPAR_XGPIOPS_0_DEVICE_ID

 // “STM32가 Zynq를 깨우는 pulse”가 들어오는 인터럽트 ID (PL->PS IRQ라면 이 값이 맞아야 함)
 #define STM32_INTR_ID      61

 // GPIO 핀 번호(PS GPIO 번호). 반드시 실제 핀맵에 맞게 수정하세요.
 #define PIN_SPI_CS         54   // Zynq -> STM32 NSS (CS)
 #define PIN_STATUS_LED     55   // 상태 표시 LED
 #define PIN_READY_IN       56   // STM32 READY -> Zynq input (polling)
 #define PIN_BUSY_IN        57   // (옵션) STM32 BUSY -> Zynq input (디버그용, 없어도 됨)

 /* ----------------------------
  * STM32 SPI protocol
  * ---------------------------- */
 #define CMD_INFO   0x10
 #define CMD_CHUNK  0x11
 #define CMD_RESET  0x12

/* Source report physical page 96 */
#define MAGIC_CODE 0x1234ABCDu

/* INFO response: 16 bytes
 * [0..3]  magic (LE)
 * [4..5]  w (LE)
 * [6..7]  h (LE)
 * [8..11] total_bytes (LE)
 * [12..15] remain_bytes (LE)
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t w;
    uint16_t h;
    uint32_t total;
    uint32_t remain;
} InfoResp;

/* ----------------------------
 * Image sizes
 * ---------------------------- */
#define IMG_W              320
#define IMG_H              240
#define BYTES_PER_PIXEL    2
#define FRAME_SIZE         (IMG_W * IMG_H * BYTES_PER_PIXEL)   // 153600
#define CHUNK_SIZE         2048
#define NUM_CHUNKS         (FRAME_SIZE / CHUNK_SIZE)           // 75

#if (FRAME_SIZE % CHUNK_SIZE) != 0
#error "FRAME_SIZE must be multiple of CHUNK_SIZE for this design"
#endif

/* ----------------------------
 * Globals
 * ---------------------------- */
static XScuGic  gIntc;
static XGpioPs  gGpio;
static XSpiPs   gSpi;

static FATFS gFatFs;
static FIL   gFil;

/* Buffers (DDR/global) */
static uint8_t gFrameBuffer[FRAME_SIZE] __attribute__((aligned(64)));
static uint8_t gDummyTx[CHUNK_SIZE]     __attribute__((aligned(64))); // all zero OK

/* RTOS objects */
static SemaphoreHandle_t gWakeSem;
static QueueHandle_t     gFrameQ;
static QueueHandle_t     gResultQ;

/* Buffer ownership control */
static volatile uint32_t gBusy = 0;  // 0: idle, 1: processing frame

/* ----------------------------
 * Minimal low-power idle hook (WFI)
 * ---------------------------- */

/* Source report physical page 97 */
void vApplicationIdleHook(void) {
    __asm__ volatile ("wfi");
}

/* ----------------------------
 * IRQ handler (wake-up pulse)
 * ---------------------------- */
static void STM32_Intr_Handler(void *CallbackRef)
{
    (void)CallbackRef;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 이미 처리 중이면 wake 신호 무시(버퍼 덮어쓰기 방지)
    if (gBusy == 0) {
        xSemaphoreGiveFromISR(gWakeSem, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // NOTE:
    // 인터럽트 소스가 AXI GPIO/PL IP라면 여기서 “원인 clear/ack”가 반드시 필요할 수 있음.
}

/* ----------------------------
 * GPIO helpers
 * ---------------------------- */
static inline void cs_low(void)  { XGpioPs_WritePin(&gGpio, PIN_SPI_CS, 0); }
static inline void cs_high(void) { XGpioPs_WritePin(&gGpio, PIN_SPI_CS, 1); }

static inline int ready_read(void) {
    return (int)XGpioPs_ReadPin(&gGpio, PIN_READY_IN);
}

static inline int busy_read(void) {
    return (int)XGpioPs_ReadPin(&gGpio, PIN_BUSY_IN);
}

/* ----------------------------
 * SPI helpers
 * ---------------------------- */
static int spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    int st = XSpiPs_PolledTransfer(&gSpi, (uint8_t*)tx, rx, len);
    return st;
}

/* CMD 1바이트 전송 (CS 한 번) */
static int spi_send_cmd(uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx = 0;

    cs_low();
    int st = spi_xfer(&tx, &rx, 1);
    cs_high();

/* Source report physical page 98 */
return st;
}

/* READY 기다리기 (timeout_ms) */
static int wait_ready(int timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t limit = pdMS_TO_TICKS(timeout_ms);

    while (ready_read() == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if ((xTaskGetTickCount() - start) > limit) {
            return -1;
        }
    }
    return 0;
}

/* RESP 읽기 (CS separate) */
static int spi_read_resp(uint8_t *out, uint32_t len)
{
    cs_low();
    int st = spi_xfer(gDummyTx, out, len);
    cs_high();
    return st;
}

/* ----------------------------
 * Protocol: RESET / INFO / CHUNK
 * ---------------------------- */
static int proto_reset(void)
{
    uint8_t resp[4] = {0};

    if (spi_send_cmd(CMD_RESET) != XST_SUCCESS) return -1;
    if (wait_ready(500) != 0) return -2;
    if (spi_read_resp(resp, 4) != XST_SUCCESS) return -3;

    return 0;
}

static int proto_info(InfoResp *out)
{
    uint8_t resp[16] = {0};

    if (spi_send_cmd(CMD_INFO) != XST_SUCCESS) return -1;
    if (wait_ready(500) != 0) return -2;
    if (spi_read_resp(resp, 16) != XST_SUCCESS) return -3;

    memcpy(out, resp, sizeof(InfoResp));
    return 0;
}

static int proto_read_frame_75chunks(uint8_t *dst153600)
{
    for (int i = 0; i < NUM_CHUNKS; i++) {

/* Source report physical page 99 */
uint8_t *p = dst153600 + (i * CHUNK_SIZE);

        if (spi_send_cmd(CMD_CHUNK) != XST_SUCCESS) return -1;
        if (wait_ready(1000) != 0) return -2;
        if (spi_read_resp(p, CHUNK_SIZE) != XST_SUCCESS) return -3;
    }
    return 0;
}

/* ----------------------------
 * AI pipeline (Similarity-based OWNER decision)
 * ---------------------------- */
#define SRC_W 320
#define SRC_H 240
#define DST_W 64
#define DST_H 64

#define CLS_BACKGROUND 0
#define CLS_OWNER      1
#define CLS_STRANGER   2

static uint8_t g_gray320[SRC_W * SRC_H];
static uint8_t g_gray64[DST_W * DST_H];
static float   g_input[DST_W * DST_H];

static float g_conv1_out[CONV1_OUT_CH * DST_H * DST_W];
static float g_pool1_out[CONV1_OUT_CH * (DST_H/2) * (DST_W/2)];
static float g_conv2_out[CONV2_OUT_CH * (DST_H/2) * (DST_W/2)];
static float g_pool2_out[CONV2_OUT_CH * (DST_H/4) * (DST_W/4)];
static float g_fc1_out[FC1_OUT];   // <-- 이걸 "임베딩"처럼 사용

/* ---- Similarity settings ---- */
#define USE_BG_PROB_CUT      1
#define BG_THR               0.60f

// 현장 튜닝 값 (대충 0.75~0.90 사이에서 맞추는 편)
#define OWNER_SIM_THR        0.85f


// (주인 얼굴 여러 장에서 g_fc1_out을 뽑아 평균낸 벡터)
static const float OWNER_TEMPLATE[32] = {
    0.12f, 0.03f, 0.00f, 0.58f, 0.41f, 0.07f, 0.22f, 0.00f,
    0.35f, 0.90f, 0.14f, 0.01f, 0.66f, 0.28f, 0.05f, 0.11f,
    0.02f, 0.44f, 0.19f, 0.00f, 0.73f, 0.09f, 0.00f, 0.31f,
    0.08f, 0.52f, 0.06f, 0.00f, 0.27f, 0.13f, 0.04f, 0.62f
};

/* ---- vector helpers ---- */
static float vec_dot(const float *a, const float *b, int n)
{
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += a[i] * b[i];
    return s;
}

static float vec_norm(const float *a, int n)

/* Source report physical page 100 */
{
    float s = vec_dot(a, a, n);
    if (s <= 0.0f) return 0.0f;
    return sqrtf(s);
}

static float cosine_sim(const float *a, const float *b, int n)
{
    float na = vec_norm(a, n);
    float nb = vec_norm(b, n);
    if (na <= 0.0f || nb <= 0.0f) return 0.0f;
    return vec_dot(a, b, n) / (na * nb);
}

/* ---- utils ---- */
static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float relu(float x) { return (x > 0.0f) ? x : 0.0f; }

static void rgb565be_to_gray8_320x240(const uint8_t *src_rgb565be, uint8_t *dst_gray)
{
    for (int i = 0; i < SRC_W * SRC_H; i++) {
        uint16_t hi = src_rgb565be[2*i + 0];
        uint16_t lo = src_rgb565be[2*i + 1];
        uint16_t p  = (uint16_t)((hi << 8) | lo);

        uint8_t r = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
        uint8_t g = (uint8_t)(((p >>  5) & 0x3F) * 255 / 63);
        uint8_t b = (uint8_t)(((p >>  0) & 0x1F) * 255 / 31);

        uint16_t y = (uint16_t)(77 * r + 150 * g + 29 * b);
        dst_gray[i] = (uint8_t)((y + 128) >> 8);
    }
}

static void resize_bilinear_gray8_320x240_to_64x64(const uint8_t *src, uint8_t *dst)
{
    const float scale_x = (float)SRC_W / (float)DST_W;
    const float scale_y = (float)SRC_H / (float)DST_H;

    for (int oy = 0; oy < DST_H; oy++) {
        float sy = ((float)oy + 0.5f) * scale_y - 0.5f;
        int y0 = (int)floorf(sy);
        int y1 = y0 + 1;
        float wy = sy - (float)y0;

        y0 = clampi(y0, 0, SRC_H - 1);
        y1 = clampi(y1, 0, SRC_H - 1);

        for (int ox = 0; ox < DST_W; ox++) {
            float sx = ((float)ox + 0.5f) * scale_x - 0.5f;
            int x0 = (int)floorf(sx);
            int x1 = x0 + 1;

/* Source report physical page 101 */
float wx = sx - (float)x0;

            x0 = clampi(x0, 0, SRC_W - 1);
            x1 = clampi(x1, 0, SRC_W - 1);

            uint8_t p00 = src[y0 * SRC_W + x0];
            uint8_t p10 = src[y0 * SRC_W + x1];
            uint8_t p01 = src[y1 * SRC_W + x0];
            uint8_t p11 = src[y1 * SRC_W + x1];

            float top = (1.0f - wx) * (float)p00 + wx * (float)p10;
            float bot = (1.0f - wx) * (float)p01 + wx * (float)p11;
            float val = (1.0f - wy) * top + wy * bot;

            int iv = (int)(val + 0.5f);
            if (iv < 0) iv = 0;
            if (iv > 255) iv = 255;
            dst[oy * DST_W + ox] = (uint8_t)iv;
        }
    }
}

static void preprocess_gray64_u8_to_input_float(const uint8_t *gray64, float
*out_input_1chw)
{
    for (int i = 0; i < DST_W * DST_H; i++) {
        out_input_1chw[i] = ((float)gray64[i] / 127.5f) - 1.0f; // [-1,1]
    }
}

static void conv2d_same_3x3(
    const float *in, int in_ch, int in_h, int in_w,
    const float *w, const float *b, int out_ch,
    float *out
){
    for (int oc = 0; oc < out_ch; oc++) {
        for (int y = 0; y < in_h; y++) {
            for (int x = 0; x < in_w; x++) {
                float acc = b[oc];

                for (int ic = 0; ic < in_ch; ic++) {
                    for (int ky = -1; ky <= 1; ky++) {
                        int sy = y + ky;
                        if ((unsigned)sy >= (unsigned)in_h) continue;
                        for (int kx = -1; kx <= 1; kx++) {
                            int sx = x + kx;
                            if ((unsigned)sx >= (unsigned)in_w) continue;

                            int widx = (((oc * in_ch + ic) * 3 + (ky + 1)) * 3 + (kx +
1));
                            float ww = w[widx];
                            float vv = in[(ic * in_h + sy) * in_w + sx];
                            acc += ww * vv;
                        }
                    }
                }

/* Source report physical page 102 */
out[(oc * in_h + y) * in_w + x] = acc;
            }
        }
    }
}

static void relu_inplace(float *v, int n)
{
    for (int i = 0; i < n; i++) v[i] = relu(v[i]);
}

static void maxpool2x2(
    const float *in, int ch, int in_h, int in_w,
    float *out
){
    int out_h = in_h >> 1;
    int out_w = in_w >> 1;

    for (int c = 0; c < ch; c++) {
        for (int oy = 0; oy < out_h; oy++) {
            int y0 = oy * 2;
            int y1 = y0 + 1;
            for (int ox = 0; ox < out_w; ox++) {
                int x0 = ox * 2;
                int x1 = x0 + 1;

                float p00 = in[(c * in_h + y0) * in_w + x0];
                float p10 = in[(c * in_h + y0) * in_w + x1];
                float p01 = in[(c * in_h + y1) * in_w + x0];
                float p11 = in[(c * in_h + y1) * in_w + x1];

                float m = p00;
                if (p10 > m) m = p10;
                if (p01 > m) m = p01;
                if (p11 > m) m = p11;

                out[(c * out_h + oy) * out_w + ox] = m;
            }
        }
    }
}

static void fc(
    const float *in, int in_n,
    const float *w, const float *b,
    float *out, int out_n
){
    for (int o = 0; o < out_n; o++) {
        float acc = b[o];
        const float *row = &w[o * in_n];
        for (int i = 0; i < in_n; i++) {
            acc += row[i] * in[i];
        }
        out[o] = acc;
    }

/* Source report physical page 103 */
}

static void softmax3(const float *logits, float *probs)
{
    float m = logits[0];
    if (logits[1] > m) m = logits[1];
    if (logits[2] > m) m = logits[2];

    float e0 = expf(logits[0] - m);
    float e1 = expf(logits[1] - m);
    float e2 = expf(logits[2] - m);
    float s  = e0 + e1 + e2;

    if (s > 0.0f) {
        float inv = 1.0f / s;
        probs[0] = e0 * inv;
        probs[1] = e1 * inv;
        probs[2] = e2 * inv;
    } else {
        probs[0] = probs[1] = probs[2] = 0.0f;
    }
}

static void tinycnn3_forward_1ch64(const float *input_1chw, float *out_logits3)
{
    conv2d_same_3x3(input_1chw, CONV1_IN_CH, DST_H, DST_W,
                    CONV1_WEIGHT, CONV1_BIAS, CONV1_OUT_CH,
                    g_conv1_out);
    relu_inplace(g_conv1_out, CONV1_OUT_CH * DST_H * DST_W);
    maxpool2x2(g_conv1_out, CONV1_OUT_CH, DST_H, DST_W, g_pool1_out);

    int h1 = DST_H / 2;
    int w1 = DST_W / 2;
    conv2d_same_3x3(g_pool1_out, CONV2_IN_CH, h1, w1,
                    CONV2_WEIGHT, CONV2_BIAS, CONV2_OUT_CH,
                    g_conv2_out);
    relu_inplace(g_conv2_out, CONV2_OUT_CH * h1 * w1);
    maxpool2x2(g_conv2_out, CONV2_OUT_CH, h1, w1, g_pool2_out);

    fc(g_pool2_out, FC1_IN, FC1_WEIGHT, FC1_BIAS, g_fc1_out, FC1_OUT);
    relu_inplace(g_fc1_out, FC1_OUT);

    fc(g_fc1_out, FC2_IN, FC2_WEIGHT, FC2_BIAS, out_logits3, FC2_OUT);
}

/*
 * similarity-based predict:
 * - BG는 softmax로 컷(선택)
 * - OWNER/STRANGER는 FC1 임베딩 cosine similarity로 결정
 */
static int tinycnn3_predict_1ch64(const float *input_1chw, float *out_probs3,
                                 float owner_thr, float bg_thr)
{
    (void)owner_thr;
    (void)bg_thr;

/* Source report physical page 104 */
float logits[3];
    tinycnn3_forward_1ch64(input_1chw, logits);

    float probs[3] = {0};
    softmax3(logits, probs);

    if (out_probs3) {
        out_probs3[0] = probs[0];
        out_probs3[1] = probs[1];
        out_probs3[2] = probs[2];
    }

#if USE_BG_PROB_CUT
    if (probs[0] >= BG_THR) return CLS_BACKGROUND;
#endif

    float sim = cosine_sim(g_fc1_out, OWNER_TEMPLATE, FC1_OUT);
    if (sim >= OWNER_SIM_THR) return CLS_OWNER;
    return CLS_STRANGER;
}

static void run_ai_pipeline_once(const uint8_t *rgb565be_153600,
                                uint8_t *out_class,
                                float out_probs3[3])
{
    rgb565be_to_gray8_320x240(rgb565be_153600, g_gray320);
    resize_bilinear_gray8_320x240_to_64x64(g_gray320, g_gray64);
    preprocess_gray64_u8_to_input_float(g_gray64, g_input);

    float probs[3];
    int cls = tinycnn3_predict_1ch64(g_input, probs, 0.90f, 0.60f);

    *out_class = (uint8_t)cls;
    if (out_probs3) {
        out_probs3[0] = probs[0];
        out_probs3[1] = probs[1];
        out_probs3[2] = probs[2];
    }
}

/* ----------------------------
 * RTOS messages
 * ---------------------------- */
typedef struct {
    uint8_t *rgb565be; // points to gFrameBuffer
    uint32_t frame_id;
} FrameMsg;

typedef struct {
    uint32_t frame_id;
    uint8_t  cls;
    float    probs[3]; // bg 컷용(현재 구조)
    float    sim;      // NEW: owner cosine similarity
} ResultMsg;

/* ----------------------------

/* Source report physical page 105 */
* Hardware setup
 * ---------------------------- */
static int Setup_Hardware(void)
{
    int Status;
    XGpioPs_Config *GpioConfig;
    XSpiPs_Config  *SpiConfig;
    XScuGic_Config *IntcConfig;

    xil_printf("===== System =====\r\nBooting...\r\n");

    memset(gDummyTx, 0, sizeof(gDummyTx));

    // GPIO init
    GpioConfig = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    if (!GpioConfig) return XST_FAILURE;
    Status = XGpioPs_CfgInitialize(&gGpio, GpioConfig, GpioConfig->BaseAddr);
    if (Status != XST_SUCCESS) return Status;

    // CS output
    XGpioPs_SetDirectionPin(&gGpio, PIN_SPI_CS, 1);
    XGpioPs_SetOutputEnablePin(&gGpio, PIN_SPI_CS, 1);
    cs_high();

    // status LED output
    XGpioPs_SetDirectionPin(&gGpio, PIN_STATUS_LED, 1);
    XGpioPs_SetOutputEnablePin(&gGpio, PIN_STATUS_LED, 1);
    XGpioPs_WritePin(&gGpio, PIN_STATUS_LED, 0);

    // READY input
    XGpioPs_SetDirectionPin(&gGpio, PIN_READY_IN, 0);

    // BUSY input (optional)
    XGpioPs_SetDirectionPin(&gGpio, PIN_BUSY_IN, 0);

    // SPI init
    SpiConfig = XSpiPs_LookupConfig(SPI_DEVICE_ID);
    if (!SpiConfig) return XST_FAILURE;
    Status = XSpiPs_CfgInitialize(&gSpi, SpiConfig, SpiConfig->BaseAddress);
    if (Status != XST_SUCCESS) return Status;

    // Mode0 + Master
    Status = XSpiPs_SetOptions(&gSpi,
                              XSPIPS_MASTER_OPTION
                              | XSPIPS_FORCE_SSELECT_OPTION);
    if (Status != XST_SUCCESS) return Status;

    // SPI clock prescaler (조절 가능)
    XSpiPs_SetClkPrescaler(&gSpi, XSPIPS_CLK_PRESCALE_64);

    // CS는 GPIO로 직접 제어하므로, SlaveSelect는 의미 없어도 무방.
    XSpiPs_SetSlaveSelect(&gSpi, 0);

    // GIC init
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (!IntcConfig) return XST_FAILURE;

/* Source report physical page 106 */
Status = XScuGic_CfgInitialize(&gIntc, IntcConfig, IntcConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) return Status;

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 &gIntc);
    Xil_ExceptionEnable();

    // Rising edge
    XScuGic_SetPriorityTriggerType(&gIntc, STM32_INTR_ID, 0xA0, 0x3);
    XScuGic_Connect(&gIntc, STM32_INTR_ID, (Xil_InterruptHandler)STM32_Intr_Handler,
NULL);
    XScuGic_Enable(&gIntc, STM32_INTR_ID);

    xil_printf("HW init OK (GPIO/SPI/IRQ)\r\n");

    // SD mount
    if (f_mount(&gFatFs, "0:/", 1) == FR_OK) {
        xil_printf("SD mount OK\r\n");
    } else {
        xil_printf("SD mount FAILED\r\n");
    }

    return XST_SUCCESS;
}

/* ----------------------------
 * Tasks
 * ---------------------------- */
static void RxTask(void *arg)
{
    (void)arg;
    FrameMsg msg;
    InfoResp info;
    uint32_t frame_id = 0;

    xil_printf("RTOS scheduler start\r\n");
    xil_printf("===== Idle =====\r\n");
    xil_printf("Waiting for STM32 interrupt...\r\n");

    for (;;) {
        // wait wake pulse
        xSemaphoreTake(gWakeSem, portMAX_DELAY);

        // lock ownership (prevent overwrite)
        gBusy = 1;
        frame_id++;

        xil_printf("\r\n===== RxTask =====\r\n");
        xil_printf("Wake pulse received. Start protocol.\r\n");

        // optional: show BUSY pin if wired
        xil_printf("READY=%d BUSY=%d\r\n", ready_read(), busy_read());

        // RESET (sync)

/* Source report physical page 107 */
int r = proto_reset();
        if (r != 0) {
            xil_printf("RESET failed (%d)\r\n", r);
            gBusy = 0;
            continue;
        }

        // INFO poll until total>0 or timeout
        TickType_t t0 = xTaskGetTickCount();
        const TickType_t tout = pdMS_TO_TICKS(2000);

        do {
            r = proto_info(&info);
            if (r != 0) {
                xil_printf("INFO failed (%d)\r\n", r);
                break;
            }

            if (info.magic != MAGIC_CODE) {
                xil_printf("Bad MAGIC: 0x%08lx\r\n", (unsigned long)info.magic);
                r = -10;
                break;
            }

            if (info.total == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                break;
            }
        } while ((xTaskGetTickCount() - t0) < tout);

        if (r != 0 || info.total == 0) {
            xil_printf("No frame available or INFO timeout.\r\n");
            gBusy = 0;
            continue;
        }

        xil_printf("INFO: %dx%d total=%lu remain=%lu\r\n",
                   info.w, info.h,
                   (unsigned long)info.total,
                   (unsigned long)info.remain);

        // receive 75 chunks
        r = proto_read_frame_75chunks(gFrameBuffer);
        if (r != 0) {
            xil_printf("CHUNK receive failed (%d)\r\n", r);
            gBusy = 0;
            continue;
        }

        // optional: read INFO again to see remain goes 0
        (void)proto_info(&info);
        xil_printf("RX_DONE: got %d bytes (remain=%lu)\r\n",
                   FRAME_SIZE, (unsigned long)info.remain);

        msg.rgb565be = gFrameBuffer;

/* Source report physical page 108 */
msg.frame_id = frame_id;

        xQueueSend(gFrameQ, &msg, portMAX_DELAY);
    }
}

static void AITask(void *arg)
{
    (void)arg;
    FrameMsg in;
    ResultMsg out;

    for (;;) {
        xQueueReceive(gFrameQ, &in, portMAX_DELAY);

        xil_printf("\r\n===== AITask =====\r\n");
        xil_printf("AI_START frame_id=%lu\r\n", (unsigned long)in.frame_id);

        XGpioPs_WritePin(&gGpio, PIN_STATUS_LED, 1);

        out.frame_id = in.frame_id;
        run_ai_pipeline_once(in.rgb565be, &out.cls, out.probs);

        // NEW: similarity 로그용
        out.sim = cosine_sim(g_fc1_out, OWNER_TEMPLATE, FC1_OUT);

        XGpioPs_WritePin(&gGpio, PIN_STATUS_LED, 0);

        xil_printf("AI_DONE cls=%d  P(bg)=%.3f  sim=%.3f\r\n",
                   out.cls, out.probs[0], out.sim);

        xQueueSend(gResultQ, &out, portMAX_DELAY);
    }
}

static void SaveTask(void *arg)
{
    (void)arg;
    ResultMsg r;
    UINT bw;

    for (;;) {
        xQueueReceive(gResultQ, &r, portMAX_DELAY);

        xil_printf("\r\n===== SaveTask =====\r\n");

        // stranger일 때만 저장
        if (r.cls != CLS_STRANGER) {
            xil_printf("Not stranger -> skip save.\r\n");
            gBusy = 0;
            xil_printf("===== Idle =====\r\nWaiting for STM32 interrupt...\r\n");
            continue;
        }

        char path[64];
        snprintf(path, sizeof(path), "0:/intr_%06lu.bin", (unsigned long)r.frame_id);

/* Source report physical page 109 */
xil_printf("SAVE_START %s (%d bytes)\r\n", path, FRAME_SIZE);

        Xil_DCacheFlushRange((UINTPTR)gFrameBuffer, FRAME_SIZE);

        FRESULT fr = f_open(&gFil, path, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr != FR_OK) {
            xil_printf("File open error: %d\r\n", fr);
            gBusy = 0;
            xil_printf("===== Idle =====\r\nWaiting for STM32 interrupt...\r\n");
            continue;
        }

        fr = f_write(&gFil, gFrameBuffer, FRAME_SIZE, &bw);
        f_close(&gFil);

        xil_printf("SAVE_DONE fr=%d wrote=%u\r\n", fr, (unsigned)bw);

        gBusy = 0;
        xil_printf("===== Idle =====\r\nWaiting for STM32 interrupt...\r\n");
    }
}

/* ----------------------------
 * main
 * ---------------------------- */
int main(void)
{
    if (Setup_Hardware() != XST_SUCCESS) {
        xil_printf("Hardware init failed.\r\n");
        while (1) {}
    }

    gWakeSem = xSemaphoreCreateBinary();
    gFrameQ  = xQueueCreate(1, sizeof(FrameMsg));
    gResultQ = xQueueCreate(1, sizeof(ResultMsg));

    if (!gWakeSem || !gFrameQ || !gResultQ) {
        xil_printf("RTOS object create failed.\r\n");
        while (1) {}
    }

    xTaskCreate(RxTask,   "RxTask",   4096, NULL, 4, NULL);
    xTaskCreate(AITask,   "AITask",   4096, NULL, 3, NULL);
    xTaskCreate(SaveTask, "SaveTask", 4096, NULL, 2, NULL);

    vTaskStartScheduler();

    while (1) {}
    return 0;
}
