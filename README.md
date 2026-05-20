# 🏎️ 4휠 스마트 RC카 고속 주행 및 탈선 방지 알고리즘 고도화 프로젝트

(https://img.youtube.com/vi/HuwVZSxAesw/mqdefault.jpg)](https://youtu.be/HuwVZSxAesw)
> **아두이노(Arduino) 기반 4인치 스마트 RC카의 주행 알고리즘을 개선하여, 기존 On/Off 제어의 한계를 극복하고 주행 안정성과 코스 완주율을 극대화한 프로젝트입니다.**

---

## 🛠️ 1. 하드웨어 구성 및 부품 정보 (Hardware Specification)

본 프로젝트는 **에듀이노 4휠 스마트 RC카 메이커 키트**의 하드웨어를 기반으로 설계되었습니다.

* **메인 컨트롤러 (MCU):** 아두이노 우노(Arduino UNO) R3 보드
* **센서 확장 인터페이스:** 아두이노 우노용 센서 확장 쉴드 V4.0 (Sensor Shield v4.0)
* **모터 드라이버:** L298N 듀얼 H-브릿지 모터 드라이버 모듈
* **구동부:** DC 기어드 모터 4개 + 타이어 4개 (4륜 구동 방식, 좌/우측 각각 2개씩 통합 제어)
* **센서부:** TCRT5000 적외선 반사형 센서 모듈 3개 (라인트레이서)
* **전원부:** 6칸 배터리 홀더 (AA 건전지 6개 사용, 약 9V 공급)

### 📌 하드웨어 핀 매핑 (Pin Mapping)

| 분류 | 하드웨어 명칭 | 아두이노 연결 핀 | 기능 및 역할 |
| :--- | :--- | :--- | :--- |
| **모터 제어** | RightMotor_E_pin | **D5** (PWM) | 우측 모터 속도 제어 (Enable A) |
| | LeftMotor_E_pin | **D6** (PWM) | 좌측 모터 속도 제어 (Enable B) |
| | RightMotor_1_pin | **D8** | 우측 모터 정/역회전 제어선 (IN1) |
| | RightMotor_2_pin | **D9** | 우측 모터 정/역회전 제어선 (IN2) |
| | LeftMotor_3_pin | **D10** | 좌측 모터 정/역회전 제어선 (IN3) |
| | LeftMotor_4_pin | **D11** | 좌측 모터 정/역회전 제어선 (IN4) |
| **센서 입력** | L_Line (좌측 센서) | **A5** (Digital) | 좌측 라인 감지 (적외선 TCRT5000) |
| | C_Line (중앙 센서) | **A4** (Digital) | 중앙 라인 감지 (적외선 TCRT5000) |
| | R_Line (우측 센서) | **A3** (Digital) | 우측 라인 감지 (적외선 TCRT5000) |

---

## 📈 2. 주요 개선 사항 및 성과 (Key Improvements)

### 1️⃣ 제어 정밀도 고도화 (On/Off 단조 제어 ➡️ 7단계 상태 분기 제어)
* **기본 예제의 한계:** 완만한 곡선 구간에서도 한쪽 모터를 완전히 꺼버려 로봇이 심하게 흔들리고 속도가 급감하는 지그재그(헌팅) 현상이 발생했습니다.
* **개선 및 성과:** 주행 모드를 완만한 코너(양륜 차동 구동 전진)와 급격한 코너(역회전 스핀 턴)로 세분화했습니다. 이를 통해 트랙 곡률에 맞춘 연동 조향이 가능해져 **평균 주행 속도가 대폭 상승하고 주행 안정성을 확보**했습니다.

### 2️⃣ 임베디드 예외 처리 시스템 구축 (Fail-Safe 자가 복귀 로직)
* **기본 예제의 한계:** 관성에 의해 센서가 일시적으로 라인을 완전히 벗어나면(`0 0 0`), 차량이 트랙 밖으로 직진하여 주행에 실패했습니다.
* **개선 및 성과:** `last_state` 메모리 알고리즘을 도입했습니다. 라인을 이탈하는 순간 **마지막에 라인을 감지했던 방향을 추적**하여 그 방향으로 즉시 복귀 회전을 수행함으로써 **코스 완주율을 획기적으로 향상**시켰습니다.

### 3️⃣ 소프트웨어 아키텍처 리팩토링 및 유지보수성 극대화
* **기본 예제의 한계:** `analogWrite`와 `digitalWrite`가 분산 호출되어 제어 로직 가독성이 떨어졌고 매직 넘버 사용으로 하드웨어 튜닝이 번거로웠습니다.
* **개선 및 성과:** 모터 제어 명령을 정수형 상태 데이터(`1`, `-1`, `0`)와 PWM 값으로 통합 결합한 고성능 `motor_role` 인터페이스 함수를 설계했습니다. 또한 모든 조향 속도 인자를 상단에 전역 상수로 배치하여, **배터리 잔량이나 트랙 상황 변화에 대응한 파라미터 튜닝 시간을 최소화**했습니다.

---

## 💻 3. 핵심 소스 코드 (Source Code)

```cpp
/**
 * @file LineTracer_Advanced.ino
 * @brief 4휠 스마트 RC카 고속 주행 및 탈선 방지 알고리즘 적용 코드
 */

const int RightMotor_E_pin = 5;  // 우측 모터 Enable & PWM
const int LeftMotor_E_pin = 6;   // 좌측 모터 Enable & PWM
const int RightMotor_1_pin = 8;  // 우측 모터 제어선 IN1
const int RightMotor_2_pin = 9;  // 우측 모터 제어선 IN2
const int LeftMotor_3_pin = 10; // 좌측 모터 제어선 IN3
const int LeftMotor_4_pin = 11; // 좌측 모터 제어선 IN4

const int L_Line = A5;           // 좌측 라인트레이서 센서
const int C_Line = A4;           // 중앙 라인트레이서 센서
const int R_Line = A3;           // 우측 라인트레이서 센서

// 주행 속도 파라미터 (상단 변수화로 유지보수성 최적화)
const int BaseSpeed = 250;       // 직진 속도
const int CurveHigh = 255;       // 곡선 바깥쪽 바퀴
const int CurveLow = 180;        // 곡선 안쪽 바퀴 (지그재그 방지 고성능 튜닝)
const int SpinSpeed = 250;       // 급회전 및 라인 복귀 속도

int last_state = 0;              // 라인 이탈 시 이전 상태 기억용 (0:직진, 1:좌, 2:우)
unsigned long startTime;         // 부스터 시간 측정용

void setup() {
  pinMode(RightMotor_E_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);

  Serial.begin(9600);
  Serial.println("System Ready: Smooth & Booster Mode");
  startTime = millis(); // 런타임 시작 시간 기록
}

void loop() {
  // [Time-Triggered] 딜레이 없는 비동기 제어로 초반 2초간 부스터 모드 활성화
  bool isBooster = (millis() - startTime < 2000);
  line_tracing(isBooster);
}

void line_tracing(bool booster) {
  int L = digitalRead(L_Line);
  int C = digitalRead(C_Line);
  int R = digitalRead(R_Line);

  // 부스터 플래그에 따른 동적 속도 할당
  int bBase = booster ? 250 : BaseSpeed;
  int bHigh = booster ? 255 : CurveHigh;
  int bLow  = booster ? 180 : CurveLow;   
  int bSpin = booster ? 255 : SpinSpeed;

  // 1. 직진 (0 1 0)
  if (L == LOW && C == HIGH && R == LOW) {
    motor_role(1, 1, bBase, bBase);
    last_state = 0;
  }
  // 2. 완만한 우회전 (0 1 1) -> 차동 구동
  else if (L == LOW && C == HIGH && R == HIGH) {
    motor_role(1, 1, bLow, bHigh); 
    last_state = 2;
  }
  // 3. 완만한 좌회전 (1 1 0) -> 차동 구동
  else if (L == HIGH && C == HIGH && R == LOW) {
    motor_role(1, 1, bHigh, bLow); 
    last_state = 1;
  }
  // 4. 급 우회전 / 90도 코너 (0 0 1) -> 제자리 회전
  else if (L == LOW && C == LOW && R == HIGH) {
    motor_role(-1, 1, bSpin, bSpin); 
    last_state = 2;
  }
  // 5. 급 좌회전 / 90도 코너 (1 0 0) -> 제자리 회전
  else if (L == HIGH && C == LOW && R == LOW) {
    motor_role(1, -1, bSpin, bSpin); 
    last_state = 1;
  }
  // 6. 라인 완전 이탈 예외 처리 (0 0 0) -> Fail-Safe 자가 복귀
  else if (L == LOW && C == LOW && R == LOW) {
    if (last_state == 1)      motor_role(1, -1, bSpin, bSpin);
    else if (last_state == 2) motor_role(-1, 1, bSpin, bSpin);
    else if (last_state == 0) motor_role(1, 1, bBase, bBase);
  }
  // 7. 정지 또는 교차로 (1 1 1)
  else if (L == HIGH && C == HIGH && R == HIGH) {
    motor_role(0, 0, 0, 0);
  }
}

void motor_role(int R_dir, int L_dir, int r_spd, int l_spd) {
  // 우측 모터 제어
  if (R_dir == 1) {
    digitalWrite(RightMotor_1_pin, HIGH);
    digitalWrite(RightMotor_2_pin, LOW);
  } else if (R_dir == -1) {
    digitalWrite(RightMotor_1_pin, LOW);
    digitalWrite(RightMotor_2_pin, HIGH);
  }
  analogWrite(RightMotor_E_pin, R_dir == 0 ? 0 : r_spd);

  // 좌측 모터 제어
  if (L_dir == 1) {
    digitalWrite(LeftMotor_3_pin, HIGH);
    digitalWrite(LeftMotor_4_pin, LOW);
  } else if (L_dir == -1) {
    digitalWrite(LeftMotor_3_pin, LOW);
    digitalWrite(LeftMotor_4_pin, HIGH);
  }
  analogWrite(LeftMotor_E_pin, L_dir == 0 ? 0 : l_spd);
}
