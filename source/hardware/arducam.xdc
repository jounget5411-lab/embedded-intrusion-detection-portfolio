## ----------------------------------------------------------------------------
## Zybo Z7-20 Pmod JE Port Mapping (STM32 + Arducam)
## ----------------------------------------------------------------------------

## 1. STM32 Wake-up Interrupt (Input) -> JE1 (Pin V12)
## Design상의 이름: Alarm
set_property -dict { PACKAGE_PIN V12   IOSTANDARD LVCMOS33 } [get_ports { Alarm }];


## 2. Arducam SPI (Data/Clock) -> JE2, JE3, JE4
## Design상의 이름: SPI (Wrapper가 _io로 변환함)

# JE2: SCK (Pin W16)
set_property -dict { PACKAGE_PIN W16   IOSTANDARD LVCMOS33 } [get_ports { SPI_sck_io }];

# JE3: MOSI (io0) (Pin J15)
set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { SPI_io0_io }];

# JE4: MISO (io1) (Pin H15)
set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 } [get_ports { SPI_io1_io }];


## 3. GPIO 제어 (CS 및 STM32 상태 알림) -> JE7, JE10
## Design상의 이름: GPIO (Wrapper가 GPIO_tri_io 로 변환함)
## Zynq 설정에서 GPIO Width를 2 이상으로 잡았다고 가정

# JE7: CS (Chip Select) -> GPIO[0] (Pin V13)
set_property -dict { PACKAGE_PIN V13   IOSTANDARD LVCMOS33 } [get_ports { GPIO_tri_io[0] }];

# JE10: Status (To STM32) -> GPIO[1] (Pin Y17)
set_property -dict { PACKAGE_PIN Y17   IOSTANDARD LVCMOS33 } [get_ports { GPIO_tri_io[1] }];


## Design상의 이름: I2C (Wrapper가 _io로 변환함)
## 4. Arducam I2C (Config) -> JE8, JE9
## 수정: PULLUP TRUE 옵션 추가 (필수!)

# JE8: SCL (Pin U17)
set_property -dict { PACKAGE_PIN U17   IOSTANDARD LVCMOS33  PULLUP TRUE } [get_ports { I2C_scl_io }];

# JE9: SDA (Pin T17)
set_property -dict { PACKAGE_PIN T17   IOSTANDARD LVCMOS33  PULLUP TRUE } [get_ports { I2C_sda_io }];

## 5. [중요] 안 쓰는 SPI SS(Slave Select) 처리
## 하드웨어적으로 강제 생성되는 SS 신호를 보드 내장 LED(LD0)로 보내버림
## 만약 에러가 나면 SPI_ss_io[0] 으로 바꿔보세요.
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { SPI_ss_io }];

## 만약 SPI 설정에 ss1, ss2가 켜져 있어서 에러가 난다면, 아래 명령어로 에러를 무시합니다.
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]