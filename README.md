# TMC5160 SD Controller

ESP32 기반 TMC5160 스테퍼 모터 드라이버 제어 프로젝트입니다. SD 모드(STEP/DIR)와 SPI 모드를 동시에 사용하여 정밀한 모터 제어를 구현합니다.

## 프로젝트 개요

이 프로젝트는 TMC5160 스테퍼 모터 드라이버를 사용하여:
- **SD 모드**: STEP/DIR 신호로 모터 제어
- **SPI 모드**: SPI 통신으로 드라이버 설정 및 모니터링
- **마이크로스텝 설정**: 헤더 파일에서 마이크로스텝 해상도 조정 가능
- **실시간 속도 제어**: RPM 기반 속도 제어
- **포토 인터럽트**: 모터 회전 감지 및 피드백

## 하드웨어 구성

### 핀 연결
```
TMC5160 <-> ESP32
- REFL_STEP (STEP) -> GPIO_NUM_2
- REFR_DIR  (DIR)  -> GPIO_NUM_1
- SPI_SDO_CFG0     -> GPIO_NUM_42
- SPI_SDI_CFG1     -> GPIO_NUM_41
- SPI_SCK_CFG2     -> GPIO_NUM_40
- SPI_CS_CFG3      -> GPIO_NUM_39
- RELAY_BREAK_A    -> GPIO_NUM_47
- RELAY_BREAK_B    -> GPIO_NUM_48
- PHOTO_INTERRUPT  -> GPIO_NUM_7
```

### 주요 구성 요소
- **ESP32**: 메인 컨트롤러
- **TMC5160**: 스테퍼 모터 드라이버 (SD + SPI 모드)
- **스테퍼 모터**: 200 스텝/회전 (1.8도)
- **포토 인터럽트**: 모터 회전 감지
- **릴레이**: 모터 브레이크 제어

## 소프트웨어 구조

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c                    # 메인 애플리케이션
│   └── driver
│       ├── tmc5160_SD_driver.h   # TMC5160 드라이버 헤더
│       └── tmc5160_SD_driver.c   # TMC5160 드라이버 구현
└── README.md
```

### 주요 파일 설명

#### `main/driver/tmc5160_SD_driver.h`
- TMC5160 드라이버 설정 및 함수 선언
- 마이크로스텝 해상도 설정 (`MICROSTEPS`)
- GPIO 핀 정의
- 속도 제어 매개변수

#### `main/driver/tmc5160_SD_driver.c`
- TMC5160 초기화 및 설정
- SPI 통신 구현
- 스텝 신호 생성 (GPTimer 사용)
- 마이크로스텝 설정 및 제어
- 레지스터 모니터링

#### `main/main.c`
- 시스템 초기화
- 버튼 및 포토 인터럽트 처리
- 주기적 상태 모니터링

## 주요 기능

### 1. 마이크로스텝 설정
헤더 파일에서 마이크로스텝 해상도를 설정할 수 있습니다:

```c
#define MICROSTEPS 32  // 1, 2, 4, 8, 16, 32, 64, 128, 256 중 선택
```

### 2. 속도 제어
- **RPM 기반 제어**: 0.5 ~ 900 RPM 범위
- **실시간 속도 변경**: 버튼으로 RPM 조정
- **정밀한 스텝 주기 계산**: 마이크로스텝을 고려한 주기 계산

### 3. 드라이버 설정
- **StealthChop 모드**: 저소음, 저전력 모드
- **SpreadCycle 모드**: 고성능 모드
- **열 관리**: 자동 열 보호 기능
- **전류 제어**: IHOLD/IRUN 전류 설정

### 4. 모니터링
- **레지스터 상태 확인**: CHOPCONF, GCONF, TSTEP 등
- **실시간 상태 출력**: 마이크로스텝 설정 확인
- **포토 인터럽트 카운트**: 모터 회전 감지

## 빌드 및 실행

### 1. 환경 설정
```bash
# ESP-IDF 환경 설정
. $HOME/esp/esp-idf/export.sh
```

### 2. 빌드
```bash
# 프로젝트 빌드
idf.py build
```

### 3. 플래시 및 모니터링
```bash
# ESP32에 펌웨어 업로드
idf.py flash

# 시리얼 모니터링
idf.py monitor
```

## 설정 및 사용법

### 마이크로스텝 변경
1. `main/driver/tmc5160_SD_driver.h` 파일에서 `MICROSTEPS` 값 수정
2. 프로젝트 재빌드 및 플래시
3. 로그에서 설정 확인:
   ```
   SD mode microstep force set: MICROSTEPS=32, MRES=3, CHOPCONF=0x170100C0
   ✓ MRES setting is correct!
   ```

### 속도 제어
- **버튼 입력**: RPM 증가/감소
- **연속 회전**: 지정된 RPM으로 연속 회전
- **정지**: 모터 정지

### 상태 모니터링
시리얼 모니터에서 다음 정보 확인 가능:
- TMC5160 레지스터 상태
- 마이크로스텝 설정
- 스텝 주기 계산
- 포토 인터럽트 카운트

## 기술 사양

### TMC5160 설정
- **동작 모드**: SD 모드 + SPI 모드
- **마이크로스텝**: 1~256 단계 설정 가능
- **전류 제어**: 자동 전류 조정
- **보호 기능**: 과열, 과전류, 단락 보호

### ESP32 설정
- **타이머**: GPTimer 사용 (고정밀 스텝 신호 생성)
- **SPI**: SPI2_HOST 사용
- **GPIO**: 인터럽트 및 제어 신호

### 성능
- **최대 RPM**: 900 RPM
- **최소 RPM**: 0.5 RPM
- **스텝 정밀도**: 마이크로스텝 해상도에 따라 결정
- **응답 시간**: 실시간 속도 변경 가능

## 문제 해결

### ⚠️ 마이크로스텝 설정 문제 (미해결)
현재 SD 모드 + SPI 모드에서 마이크로스텝 설정이 완전히 해결되지 않은 상태입니다.

**현재 상황:**
- 헤더 파일에서 `MICROSTEPS` 값을 변경해도 실제 모터 속도에 반영되지 않음
- TMC5160 레지스터는 올바르게 설정되지만 실제 동작은 다름
- SD 모드에서 SPI 설정이 무시되거나 덮어써지는 문제

**임시 해결 방법:**
1. **완전한 클린 빌드**:
   ```bash
   idf.py fullclean
   idf.py build
   idf.py flash
   ```

2. **레지스터 상태 확인**:
   ```
   TMC5160 Status Check:
     CHOPCONF: 0x170100C0 (MRES:3, TOFF:8, HSTRT:2, HEND:2)
     ✓ MRES setting is correct!
   ```

3. **SD 모드 강제 설정**: 초기화 후 CHOPCONF 재설정

**진행 중인 디버깅:**
- TMC5160 데이터시트 재검토
- SD 모드에서의 마이크로스텝 설정 방법 연구
- 하드웨어 설정 확인

### 속도가 예상과 다른 경우
- 마이크로스텝 설정 확인
- 스텝 주기 계산 로그 확인
- TMC5160 레지스터 상태 확인

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 기여

버그 리포트, 기능 요청, 풀 리퀘스트를 환영합니다.

## 연락처

프로젝트 관련 문의사항이 있으시면 이슈를 생성해 주세요.
