# 시스템 구조

## 두 보드의 실행 환경

| 구성 | 실행 환경 | 기능 |
|---|---|---|
| STM32F103RB | STM32 HAL, bare-metal main loop와 IRQ/DMA | 거리 측정, 접근 ALARM, 카메라 초기화·촬영·FIFO 읽기, SPI slave 응답 |
| Zybo Z7-20 / Zynq PS | 기존 Xilinx BSP, FreeRTOS, C float32 추론 | SPI master 수신, 64×64 입력 전처리, CNN·유사도 판단, FatFs 저장 |
| OV5642 기반 ArduCam | I2C 센서 설정, SPI ArduChip 제어 | RGB565 사진 촬영과 외부 FIFO 보관 |

FreeRTOS 커널이나 HAL/BSP를 직접 이식한 프로젝트로 설명하지 않습니다. Zynq PL의 CNN 가속도 이 프로젝트의 최종 구조에 포함하지 않습니다.

## 사진 한 장의 흐름

1. HC-SR04의 ECHO 펄스를 TIM 입력 캡처로 읽어 접근을 감지합니다. 보고서 부록의 임계값은 30 cm입니다.
2. ALARM으로 Zynq 수신 태스크를 깨웁니다.
3. Zynq가 RESET 명령을 보내고 STM32 main loop가 카메라 캡처를 시작합니다.
4. 카메라의 외부 FIFO에 사진을 보관합니다. INFO로 준비 상태를 확인하고 CHUNK로 데이터를 읽습니다.
5. Zynq의 단일 사진 버퍼를 전처리·판별합니다.
6. 낯선 사람으로 판정하면 RGB565 원본을 SD 카드에 기록합니다. 배경·소유자는 저장을 생략합니다.
7. 버퍼 사용을 해제하고 다음 접근 이벤트를 기다립니다.

연속 영상 스트리밍이나 여러 사진을 동시에 처리하는 구조가 아닙니다.

## 명령과 응답의 준비 조건

카메라 FIFO 접근용 SPI1과 두 보드 사이의 SPI2는 역할이 다릅니다. 카메라 쪽에서는 STM32가 master이고, Zynq 연결에서는 STM32가 slave입니다. 보드 간 SPI 클록은 Zynq가 공급하므로 STM32의 응답 준비 시점을 별도로 알려야 합니다.

| 명령 | 값 | 기능 | 응답 |
|---|---|---|---|
| INFO | `0x10` | 해상도·전체/남은 바이트·준비 상태 | 16 bytes |
| CHUNK | `0x11` | 다음 FIFO 청크 읽기 | 2,048 bytes |
| RESET | `0x12` | 통신 상태 초기화·새 촬영 예약 | 4 bytes |

INFO는 little-endian `magic(uint32), w(uint16), h(uint16), total(uint32), remain(uint32)`입니다. `magic=0x1234ABCD`, 촬영 준비 전에는 `total=0`입니다.

부록 코드의 제어 신호는 다음과 같습니다. Zynq 숫자는 PS GPIO driver의 핀 번호이며 보드 커넥터 번호와 구분합니다.

| 신호 | STM32 부록 | Zynq 부록 | 상태 |
|---|---|---|---|
| ALARM | PA4 output | GIC IRQ 61 | 접근 알림 |
| READY | PB1 output | GPIO 56 input | 응답 DMA 준비 알림 |
| 수동 CS | SPI2 NSS PB12 input | GPIO 54 output | 명령/응답 transaction 선택 |
| STATUS | 해당 수신 용도는 버전별 확인 필요 | GPIO 55 output | 상태 표시 |
| BUSY | PB0 input | GPIO 57 input | 양쪽 input 표기가 남아 있어 최종 배선 확인 필요 |

초기 보관 `.bd`/`.xdc`의 GPIO 폭은 2이며 READY/BUSY까지 반영한 최종 export와 같지 않습니다. `gBusy`로 관리하는 내부 처리 상태와 미확정인 외부 BUSY 배선을 혼동하지 않습니다.

```text
Zynq: 명령 전송 ── READY 대기 ── 응답 길이만큼 SPI 읽기
STM32: READY Low ── 버퍼 준비 ── DMA 등록 성공 ── READY High
                                              전송 완료 ── READY Low
```

DMA 등록 성공은 응답을 받을 준비가 됐다는 조건입니다. 실제 사진 전송 완료는 DMA 완료 처리와 이후 프로토콜 진행으로 구분합니다. INFO/READY/캡처 대기에는 timeout이 있으며 오류 시 다음 명령 수신 상태로 돌아가도록 구성했습니다.

## 태스크와 버퍼 사용

| 단계 | 대기 대상 | 우선순위 | 전달 내용 |
|---|---|---:|---|
| ALARM ISR | 외부 인터럽트 | 해당 없음 | `xSemaphoreGiveFromISR`로 수신 알림 |
| RxTask | binary semaphore | 4 | 사진 버퍼 포인터·frame ID를 frame queue에 전달 |
| AITask | frame queue | 3 | frame ID·class·확률·유사도를 result queue에 전달 |
| SaveTask | result queue | 2 | 낯선 사람 사진 저장 또는 저장 생략 후 `gBusy=0` |

Frame/result queue의 깊이는 각각 1입니다. 사진 본문 전체를 queue로 복사하지 않고 동일 버퍼의 포인터를 전달합니다. 따라서 저장/생략이 끝나기 전에는 새 접근을 처리하지 않는 단일 버퍼 정책이 필요합니다.

`gBusy`는 mutex가 아닌 상태 플래그입니다. 처리 도중 들어온 접근 이벤트는 놓칠 수 있습니다. 보고서는 이를 mutex/event group으로 개선할 여지도 제시합니다. 태스크는 semaphore/queue에서 block되고, 실행할 작업이 없을 때 Idle Hook의 WFI로 대기합니다.

## 판별과 저장

전송된 big-endian RGB565를 Gray8로 바꾸고 bilinear resize로 64×64까지 줄인 뒤 `pixel/127.5-1`로 정규화합니다. TinyCNN은 Conv 1→8→16, FC 4096→32→3이며 Zynq PS에서 float32 C로 실행합니다.

최종 보고서 부록의 분기는 다음과 같습니다.

```text
Background softmax probability ≥ 0.60 → Background
그 외 FC1 32차원 embedding과 OWNER_TEMPLATE cosine ≥ 0.85 → Owner
나머지 → Stranger
```

Stranger만 `0:/intr_%06lu.bin`에 153,600바이트 RGB565 원본을 기록합니다. `.bin`은 JPEG/PNG 파일이 아닙니다. 보고서 본문에 있는 3클래스 softmax의 단순 argmax 설명과 이 최종 분기를 구분해야 합니다. Owner template 생성 과정과 보관 가중치의 최종 부록 대응은 현재 확인되지 않았습니다.

물리 핀표와 각 보관 버전의 연결은 [코드 지도](code-map.md), 실제 재구성 순서는 [재현 안내](reproduction.md)에 정리했습니다.
