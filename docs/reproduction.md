# 재현 안내

이 저장소는 보관 소스·설정과 원보고서의 통합 참조 코드를 정리한 포트폴리오입니다. 지금 그대로 clone한 뒤 한 명령으로 장치를 빌드·실행할 수 있는 패키지는 아닙니다. 이번 정리에서는 보드 실행이나 펌웨어 빌드를 하지 않았습니다.

## 필요한 환경

- STM32F103RB 보드, HC-SR04, OV5642 기반 ArduCam/ArduChip FIFO.
- Zybo Z7-20, SD 카드, SPI·ALARM·READY 연결과 공통 GND.
- 보관 STM32 프로젝트와 호환되는 STM32CubeIDE/HAL.
- 보관 Vivado/SDK 또는 호환 Vitis 환경, Xilinx PS 드라이버와 FreeRTOS BSP, FatFs.
- 학습을 다시 수행할 경우 Python·PyTorch·torchvision·Pillow·numpy 및 직접 확보한 학습/validation 이미지. 얼굴 이미지 데이터는 저장소에 포함하지 않습니다.

## 먼저 맞출 버전

1. [코드 지도](code-map.md)의 버전 표에서 최종 통합 참조 코드와 카메라/RTOS 시험본을 구분합니다.
2. STM32 `.ioc`와 Zynq `.bd`/`.xdc`를 비교해 SPI master/slave, ALARM·READY 핀과 IRQ 연결을 확인합니다. 원본 BUSY 배선은 표와 코드가 일치하지 않으므로 별도로 정해야 합니다.
3. 보고서 복원 STM32 코드의 callback 중복·PDF 줄바꿈을 해결하고, 대응하는 header·MSP·IRQ·DMA 설정을 확정합니다. 시험용 `.ioc`에는 최종 DMA 설정이 없으므로 자동 대체하지 않습니다.
4. Zynq의 CS/STATUS/READY/BUSY에 대응하는 하드웨어 export와 BSP를 구성합니다. 보관 `.bd`는 GPIO 폭 2이며, `FreeRTOSConfig.h`는 `configUSE_IDLE_HOOK=0`, `system.mss`에는 xilffs가 없습니다. 부록의 WFI·FatFs 경로에는 Idle Hook 활성화와 파일시스템 BSP 구성이 필요합니다.
5. 보관 C 가중치 헤더는 부록과 배열 이름·shape가 맞습니다. 부록에 포함된 Owner template 상수의 생성 절차와 가중치 대응을 확인합니다.

## 소프트웨어 확인 순서

| 순서 | 확인 항목 | 통과 기준 |
|---|---|---|
| 1 | 센서·카메라 연결 | 거리 입력, OV5642 ID 0x56/0x42, ArduChip register read-back·VSYNC |
| 2 | 단일 촬영·FIFO | 320×240 RGB565와 FIFO 길이를 확인; 부족한 길이를 조용히 정상 사진으로 취급하지 않음 |
| 3 | RESET/INFO/READY | 준비 전 total=0, DMA 등록 후 READY, timeout/오류 복구 |
| 4 | 사진 전송 | 2,048 ×75 byte 수신, 원본과 byte 비교·프레임 checksum 등 무결성 확인 |
| 5 | 전처리·추론 | Python과 C의 입력 배열·중간/최종 출력 대조; 같은 가중치와 클래스 순서 사용 |
| 6 | Owner 판별 | 별도 template 생성과 threshold 검증; validation 분리 |
| 7 | 저장 | Stranger만 저장, 반환값·실제 작성 길이 확인, 저장 파일 재읽기 |
| 8 | 이벤트 처리 | 처리 중 재접근, SD 오류·READY timeout 후 gBusy 해제와 다음 이벤트 복귀 |

이 표는 앞으로 재현할 절차입니다. 수행 완료한 시험 목록으로 읽으면 안 됩니다.

## 학습·내보내기

선별 학습 스크립트는 이미지 폴더를 읽어 64×64 grayscale TinyCNN을 학습하는 원본입니다. 얼굴 이미지와 원 `.pth` 체크포인트는 넣지 않고, 보관 C 가중치 배열을 소스 근거로 보존합니다. 프로젝트의 클래스 순서와 정규화, model key/shape가 일치하는지를 먼저 확인해야 합니다.

스크립트의 기본 경로·옵션과 실제 파일 배치는 [코드 지도](code-map.md)를 참고합니다. 보관 데이터 목록과 보고서의 100/20 분할은 달라 동일 validation 정확도를 재현했다고 주장할 수 없습니다. 학습 스크립트나 가중치 export를 이번 작업에서 실행하지 않았습니다.

## 결과 기록

재현할 때는 사용한 commit, bitstream/hardware export, BSP, 두 보드의 소스 hash, 가중치/template hash, 핀맵, 테스트 조건과 로그를 함께 기록해야 합니다. 성공 시연 한 번과 장시간 안정성·정량 정확도·전력 측정은 별도 결과로 보관합니다.
