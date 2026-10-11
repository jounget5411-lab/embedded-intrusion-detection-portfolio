# 코드 지도와 버전 관계

프로젝트의 통합 구조는 원보고서 부록 5.1(STM32)·5.2(Zynq)를 기준으로 설명합니다. 보관된 `.c` 원본은 카메라·SPI·RTOS 단계별 시험본이며, 보고서에서 복원한 통합 참조 코드와 구분합니다.

## 먼저 읽을 코드

| 경로 | 내용 | 출처·상태 |
|---|---|---|
| [`source/report-reference/stm32_main.c`](../source/report-reference/stm32_main.c) | 센서·카메라 FIFO·CMD/RESP·DMA·READY·main loop | P3 원보고서 물리 pp70–94 부록 5.1의 layout 추출. 원래 `.c` 파일은 아니며 미빌드 |
| [`source/report-reference/zynq_main.c`](../source/report-reference/zynq_main.c) | RxTask/AITask/SaveTask·semaphore·queue·C 추론·FatFs | P3 원보고서 물리 pp95–109 부록 5.2의 layout 추출. 원래 `.c` 파일은 아니며 미빌드 |
| [`source/report-reference/provenance.json`](../source/report-reference/provenance.json) | 보고서 hash와 추출 범위·복원 파일 hash | 원보고서 전체는 공개하지 않음 |
| [`model/train_cnn_3class.py`](../source/model/train_cnn_3class.py) | TinyCNN·학습·클래스별 validation | 보관 `3class/제출` 원본 |
| [`model/export_tinycnn3_to_header.py`](../source/model/export_tinycnn3_to_header.py) | PyTorch 가중치를 C float 배열로 출력 | 보관 `3class/제출` 원본 |
| [`model/tinycnn3_weights.h`](../source/model/tinycnn3_weights.h) | C 추론의 shape·클래스 순서·가중치 배열 | 보관 C 헤더, 132,451개 float 값 |

PDF에서 가져온 코드는 페이지 번호를 정리하고 페이지 위치 주석을 붙인 참조 파일입니다. 원래 `.c` 파일로 표시하지 않습니다. STM32 부록에는 동일 callback의 중복 정의와 PDF 줄바꿈이 남아 있어 독립 빌드 전 정리가 필요합니다. 원보고서 전체와 얼굴 이미지 데이터는 포함하지 않았습니다.

## 선별한 보관 원본

| 경로 | 실제 코드에서 확인한 내용 | 통합본과의 관계 |
|---|---|---|
| [`stm32-bringup/Core/Src/main.c`](../source/stm32-bringup/Core/Src/main.c) | OV5642 설정·ID/VSYNC/ArduChip 진단·320×240 UART 캡처, SPI2 시험 callback | 카메라 bring-up 단계. 0xC1 SPI frame state machine은 주석 처리되어 있고 최종 0x10/0x11/0x12·DMA·READY가 없음 |
| [`stm32-bringup/Z_project3.ioc`](../source/stm32-bringup/Z_project3.ioc) 및 Core header/MSP/IRQ·linker | SPI1 master·SPI2 slave·TIM/I2C/UART 설정 | 위 bring-up 프로젝트 설정. 최종 DMA 설정으로 대체하지 않음 |
| [`spi-diagnostic/main.c`](../source/spi-diagnostic/main.c) | SPI2 2바이트 IT 전송, callback/error count, main loop 재무장 | SPI 진단 단계. Mode 2이며 카메라·최종 프로토콜이 없음 |
| [`zynq-camera-experiment/freertos_hello_world.c`](../source/zynq-camera-experiment/freertos_hello_world.c) | ISR→binary semaphore→Camera_Task, 0xAA/0xBB·256바이트 수신, 시험 딜레이 | 초기 RTOS 수신 실험. 세 태스크·CNN·SD 통합본이 아님 |
| [`zynq-camera-experiment/system.mss`](../source/zynq-camera-experiment/system.mss) / [`FreeRTOSConfig.h`](../source/zynq-camera-experiment/FreeRTOSConfig.h) | freertos10_xilinx 1.3, PS SPI/GPIO/GIC, Idle Hook=0 | 초기 실험 BSP. 최종 WFI·FatFs에는 설정 추가 확인 필요 |
| [`hardware/design_1.bd`](../source/hardware/design_1.bd) / [`arducam.xdc`](../source/hardware/arducam.xdc) | Vivado 2025.2, Zynq PS SPI/I2C/GPIO·Alarm IRQ, EMIO GPIO 폭 2 | 초기 CS/STATUS용 설계. 최종 CS/STATUS/READY/BUSY 네 신호와 직접 같지 않음 |

CP949로 저장된 일부 시험 코드·설정은 읽을 수 있도록 UTF-8로 변환했습니다. 원문 코드·저작권 표기는 유지했고, 원본과 공개 파일의 SHA256·원본 Git 경로·변환 여부를 [출처 기록](../source/provenance.json)에 남겼습니다. 전체 vendor 드라이버·생성 빌드 파일은 선별 소스에 포함하지 않았습니다.

## 비교 후 제외한 사본

- Desktop의 `project3_stm32/Z_project3`는 초음파·LED 중심입니다. 같은 이름의 카카오톡 사본은 카메라 코드가 포함되어 서로 동일한 버전이 아닙니다.
- `project3_video`와 카카오톡 카메라 사본도 byte가 완전히 같지 않습니다. 이 저장소는 `project3_video` 한 묶음만 보존했습니다.
- `visitsssss/pj3/src/main.c`는 AXI SPI 4바이트 polling 시험입니다. 최종 PS SPI/RTOS 통합본으로 사용하지 않습니다.
- `3class`와 `3class/제출`의 학습·export·C 가중치 세 파일은 원본 Git blob ID가 각각 같아 byte 단위로 동일합니다. 제출 묶음 한 개만 포함했습니다.

## 모델 연결

C 헤더의 클래스 순서 `background=0, owner=1, stranger=2`, Conv 1→8→16, FC 4096→32→3과 배열 이름은 Zynq 부록의 참조 이름·shape와 맞습니다. 배열별 선언 길이와 실제 값 개수도 일치합니다.

학습 스크립트는 `./train`의 ImageFolder를 읽어 grayscale 64×64, Normalize(0.5, 0.5)를 적용하고 각 클래스의 20%를 validation으로 자동 분리합니다. 설정은 batch 32, Adam 1e-3, 20 epoch, seed 42입니다. 최고 validation 모델을 `tinycnn3_best.pth`로 저장하고, export는 이를 `tinycnn3_weights.h`에 소수점 여섯 자리의 C 배열로 씁니다. 이 작업에서 학습·export를 실행하지 않았습니다.

이 shape 일치만으로 현재 보관 가중치가 보고서의 93.33%를 낸 동일 checkpoint인지 확정할 수는 없습니다. 부록에는 `OWNER_TEMPLATE[32]` 상수가 있지만, 이를 생성한 이미지·절차와 보관 가중치의 대응은 별도 확인이 필요합니다.

## 최종 설명에 적용하는 기준

- 전송 설정: 보고서 부록은 320×240 RGB565, 153,600 bytes, 2,048 bytes ×75.
- 학습 결과: 3클래스 validation 정확도와 최종 background+embedding 판별은 다름.
- 실행 환경: STM32 bare-metal HAL / Zynq PS FreeRTOS 앱. HAL/BSP/kernel 자체 이식 근거는 없음.
- BUSY: 보고서 핀표와 코드의 양쪽 input 표기가 맞지 않음. 내부 `gBusy`와 실제 BUSY 배선을 구분.
- 시연 로그: 상태 전이와 저장 분기 증거. 최종 해상도 전체 전송 및 저장 무결성 시험 완료 근거는 아님.

원본 archive 기준 commit: `c8618b1caef9cdf128ed20f62692d63818d9dd61`. 저장된 교육자료·vendor 생성 코드를 프로젝트 자체 개발과 구분하고, 원문의 저작권·라이선스 표기를 유지합니다.
