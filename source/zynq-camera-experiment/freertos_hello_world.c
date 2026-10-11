/* Standard Includes */
#include <stdio.h>
#include <stdlib.h>
#include "xparameters.h"
#include "xscugic.h"
#include "xgpiops.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xspips.h"
#include "xiicps.h"
#include "xil_printf.h"

// ID 정의
#define SPI_DEVICE_ID      XPAR_XSPIPS_0_DEVICE_ID
#define IIC_DEVICE_ID      XPAR_XIICPS_0_DEVICE_ID
#define IIC_SLAVE_ADDR     0x30  // OV2640 Address

// 인터럽트 ID
#ifndef XPAR_FABRIC_IRQ_F2P_0_0_INTR
#define STM32_INTR_ID   61
#else
#define STM32_INTR_ID   XPAR_FABRIC_IRQ_F2P_0_0_INTR
#endif

#define INTC_DEVICE_ID  XPAR_SCUGIC_SINGLE_DEVICE_ID
#define GPIO_DEVICE_ID  XPAR_XGPIOPS_0_DEVICE_ID

// GPIO 핀 번호
#define PIN_CAM_CS      54
#define PIN_STATUS      55

/* 전역 변수 및 객체 */
XScuGic xInterruptController;
XGpioPs xGpio;
XSpiPs SpiInstance;  // 전역으로 이동
XIicPs IicInstance;
SemaphoreHandle_t xWakeUpSemaphore;

/* 함수 프로토타입 */
void Test_SPI_Send(void); // 프로토타입 선언
int Receive_Image_From_STM32(void);

/* [함수 1] 인터럽트 핸들러 */
void STM32_Intr_Handler(void *CallbackRef)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xWakeUpSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* [함수 2] 메인 태스크 */
void Camera_Task(void *pvParameters)
{
    xil_printf("[Task] Camera System Ready. Waiting for STM32...\r\n");

    while(1) {
        // [WAIT] STM32 신호 대기
        xSemaphoreTake(xWakeUpSemaphore, portMAX_DELAY);

        // [WAKE UP]
        xil_printf("[Task] WAKE UP! Processing Start...\r\n");

        // 1. Status Pin High (3.3V)
        XGpioPs_WritePin(&xGpio, PIN_STATUS, 1);

        // 2. stm32로부터 이미지 데이터 받아옴
        Receive_Image_From_STM32();

        // (테스트용 딜레이 - 실제로는 여기서 안면인식 수행)
        vTaskDelay(pdMS_TO_TICKS(1000));

        xil_printf("[Task] Job Done. Going to Sleep.\r\n");

        // 3. Status Pin Low (0V)
        XGpioPs_WritePin(&xGpio, PIN_STATUS, 0);
    }
}

/* [함수 3] 하드웨어 초기화 */
int Setup_Hardware(void)
{
    int Status;
    XScuGic_Config *IntcConfig;
    XGpioPs_Config *GpioConfig;
    XSpiPs_Config *SpiConfig; // SPI 설정 추가

    // --- 1. GPIO 초기화 ---
    GpioConfig = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    XGpioPs_CfgInitialize(&xGpio, GpioConfig, GpioConfig->BaseAddr);

    // 핀 방향 설정
    XGpioPs_SetDirectionPin(&xGpio, PIN_CAM_CS, 1);
    XGpioPs_SetOutputEnablePin(&xGpio, PIN_CAM_CS, 1);
    XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 1); // CS Idle High (Active Low이므로 평소엔 1)

    XGpioPs_SetDirectionPin(&xGpio, PIN_STATUS, 1);
    XGpioPs_SetOutputEnablePin(&xGpio, PIN_STATUS, 1);
    XGpioPs_WritePin(&xGpio, PIN_STATUS, 0);

    // --- 2. SPI 초기화 (여기로 이동!) ---
    SpiConfig = XSpiPs_LookupConfig(SPI_DEVICE_ID);
    XSpiPs_CfgInitialize(&SpiInstance, SpiConfig, SpiConfig->BaseAddress);

    // Master 모드, 수동 CS(Force SSelect) 옵션
    XSpiPs_SetOptions(&SpiInstance, XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
    XSpiPs_SetClkPrescaler(&SpiInstance, XSPIPS_CLK_PRESCALE_64);
    // 중요: 하드웨어 SS는 안 쓰지만, 드라이버 동작을 위해 0번을 선택해둠 (실제 핀은 안 움직임)
    XSpiPs_SetSlaveSelect(&SpiInstance, 0x00);


    // --- 3. 인터럽트 컨트롤러 초기화 ---
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    XScuGic_CfgInitialize(&xInterruptController, IntcConfig, IntcConfig->CpuBaseAddress);

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
            (Xil_ExceptionHandler)XScuGic_InterruptHandler,
            &xInterruptController);
    Xil_ExceptionEnable();

    XScuGic_SetPriorityTriggerType(&xInterruptController, STM32_INTR_ID, 0xA0, 0x3); // Rising Edge
    Status = XScuGic_Connect(&xInterruptController, STM32_INTR_ID,
            (Xil_InterruptHandler)STM32_Intr_Handler, NULL);

    if (Status != XST_SUCCESS) return XST_FAILURE;

    XScuGic_Enable(&xInterruptController, STM32_INTR_ID);

    return XST_SUCCESS;
}

/* [함수 4] SPI 전송 테스트 (수정됨) */
void Test_SPI_Send(void) {
    u8 SendBuf[2];
    u8 RecvBuf[2];

    xil_printf("[TEST] SPI Sending... ");

    // 1. 데이터 준비
    SendBuf[0] = 0x80;
    SendBuf[1] = 0x55;

    // 2. [중요] GPIO로 CS 핀 내리기 (Low) -> 통신 시작
    XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 0);

    // (약간의 딜레이가 필요할 수 있음, STM32가 준비할 시간)
    for(volatile int i=0; i<100; i++);

    // 3. 데이터 전송 (Polling)
    XSpiPs_PolledTransfer(&SpiInstance, SendBuf, RecvBuf, 2);

    // (전송 끝날 때까지 기다려줌 - PolledTransfer가 해주지만 확실히 하기 위해)
    for(volatile int i=0; i<100; i++);

    // 4. [중요] GPIO로 CS 핀 올리기 (High) -> 통신 종료
    XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 1);

    xil_printf("Sent: 0x%02X, 0x%02X \r\n", SendBuf[0], SendBuf[1]);
}

/* Includes */
#include "xspips.h"
#include "xgpiops.h"
#include "FreeRTOS.h"
#include "task.h"

/* --- 설정 및 프로토콜 정의 --- */
#define MAX_IMAGE_SIZE  60000       // 최대 이미지 크기 (60KB, 메모리 상황에 맞춰 조절)
#define SPI_CHUNK_SIZE  256         // 한 번에 읽어올 데이터 크기 (STM32 버퍼 크기와 맞춰야 함)

// 명령어 (STM32와 약속된 값)
#define CMD_GET_SIZE    0xAA        // 이미지 크기 요청
#define CMD_READ_DATA   0xBB        // 이미지 데이터 요청

/* --- 전역 변수 (스택 오버플로우 방지를 위해 전역으로 선언) --- */
u8 Image_Buffer[MAX_IMAGE_SIZE];    // 최종 이미지가 저장될 곳
u32 Image_Len = 0;                  // 실제 수신된 이미지 크기

extern XSpiPs SpiInstance;          // main에 선언된 인스턴스 가져오기
extern XGpioPs xGpio;               // main에 선언된 GPIO 인스턴스

/* --- 내부 함수: 미세 딜레이 (us 단위) --- */
void delay_us(u32 count) {
    for (volatile u32 i = 0; i < count * 10; i++);
}

/* --- [핵심] 이미지 수신 함수 --- */
int Receive_Image_From_STM32(void) {
    u8 SendBuf[4];
    u8 RecvBuf[4];
    int Status;

    xil_printf("=== Start Receiving Image ===\r\n");

    // ---------------------------------------------------------
    // 1단계: 이미지 크기(Length) 알아내기
    // ---------------------------------------------------------

    // 1-1. CS Low (통신 시작)
    XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 0);
    delay_us(10); // STM32 인터럽트 진입 대기

    // 1-2. [명령 전송] 사이즈 알려줘! (0xAA)
    SendBuf[0] = CMD_GET_SIZE;
    XSpiPs_PolledTransfer(&SpiInstance, SendBuf, RecvBuf, 1);

    // STM32가 사이즈 계산해서 준비할 시간을 줌 (매우 중요!)
    delay_us(100);

    // 1-3. [데이터 수신] 4바이트 읽기 (uint32_t 크기)
    // Master는 읽을 때도 Dummy 데이터를 보내서 클럭을 만들어줘야 함
    u8 Dummy[4] = {0, 0, 0, 0};
    XSpiPs_PolledTransfer(&SpiInstance, Dummy, RecvBuf, 4);

    // 1-4. CS High (1단계 종료)
    XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 1);

    // 1-5. 수신된 바이트를 합쳐서 길이 계산 (Little Endian 가정)
    // STM32가 보내는 순서에 따라 수정 필요 (보통 하위 바이트부터 보냄)
    Image_Len = (RecvBuf[3] << 24) | (RecvBuf[2] << 16) | (RecvBuf[1] << 8) | RecvBuf[0];

    xil_printf("Image Size: %d bytes\r\n", Image_Len);

    if (Image_Len == 0 || Image_Len > MAX_IMAGE_SIZE) {
        xil_printf("[Error] Invalid Image Size!\r\n");
        return -1;
    }

    // ---------------------------------------------------------
    // 2단계: 데이터 청크(Chunk) 단위로 반복 수신
    // ---------------------------------------------------------
    u32 received_bytes = 0;
    u32 chunk_len;

    while (received_bytes < Image_Len) {
        // 남은 데이터가 CHUNK_SIZE보다 작으면 남은 만큼만 요청
        if ((Image_Len - received_bytes) >= SPI_CHUNK_SIZE) {
            chunk_len = SPI_CHUNK_SIZE;
        } else {
            chunk_len = Image_Len - received_bytes;
        }

        // 2-1. CS Low
        XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 0);
        delay_us(5);

        // 2-2. [명령 전송] 데이터 내놔! (0xBB)
        SendBuf[0] = CMD_READ_DATA;
        XSpiPs_PolledTransfer(&SpiInstance, SendBuf, RecvBuf, 1);

        // STM32가 버퍼 포인터 준비할 시간
        delay_us(20);

        // 2-3. [데이터 수신] 청크만큼 읽어서 버퍼에 저장
        // &Image_Buffer[received_bytes] 위치에 바로 저장
        XSpiPs_PolledTransfer(&SpiInstance, NULL, &Image_Buffer[received_bytes], chunk_len);

        // 2-4. CS High (한 덩어리 수신 끝)
        XGpioPs_WritePin(&xGpio, PIN_CAM_CS, 1);

        // 진행 상황 업데이트
        received_bytes += chunk_len;

        // [중요] STM32가 숨 돌릴 시간을 줘야 함 (다음 청크 준비)
        // 이 딜레이가 없으면 STM32가 데이터를 놓칠 수 있음
        delay_us(500);
    }

    xil_printf("=== Image Receive Complete (%d bytes) ===\r\n", received_bytes);

    // 여기서 Image_Buffer에 있는 데이터를 추론 엔진으로 넘기면 됨
    // Run_Inference(Image_Buffer, Image_Len);

    return 0;
}

int main(void)
{
    xil_printf("System Initializing...\r\n");

    if (Setup_Hardware() != XST_SUCCESS) {
        xil_printf("Hardware Setup Failed!\r\n");
        return -1;
    }

    xWakeUpSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(Camera_Task, "CamTask", 1024, NULL, 2, NULL);

    vTaskStartScheduler();

    while(1);
    return 0;
}
