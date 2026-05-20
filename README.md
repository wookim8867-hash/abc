// 핀 설정
// 21s (배터리 완충시)
int RightMotor_E_pin = 5;
int LeftMotor_E_pin = 6;
int RightMotor_1_pin = 8;
int RightMotor_2_pin = 9;
int LeftMotor_3_pin = 10;
int LeftMotor_4_pin = 11;

int L_Line = A5;
int C_Line = A4;
int R_Line = A3;

// 일반 주행 속도 설정 (부드러운 곡선 주행 최적화)
int BaseSpeed = 250;     // 직진 속도
int CurveHigh = 255;     // 곡선 바깥쪽 바퀴
int CurveLow = 180;      // 곡선 안쪽 바퀴 (높게 설정하여 지그재그 방지)
int SpinSpeed = 250;     // 급회전 및 라인 복귀 속도

int last_state = 0;      // 라인 이탈 시 이전 상태 기억용
unsigned long startTime; // 부스터 시간 측정용

void setup() {
  pinMode(RightMotor_E_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);

  Serial.begin(9600);
  Serial.println("System Ready: Smooth & Booster Mode");
  startTime = millis(); // 전원이 켜진 시점 저장
}

void loop() {
  // 현재 부스터 모드인지 확인 (초반 2초간 true)
  bool isBooster = (millis() - startTime < 2000);
  
  // 라인 트레이싱 실행
  line_tracing(isBooster);
}

void line_tracing(bool booster) {
  int L = digitalRead(L_Line);
  int C = digitalRead(C_Line);
  int R = digitalRead(R_Line);

  // 부스터 여부에 따른 동적 속도 할당
  int bBase = booster ? 250 : BaseSpeed;
  int bHigh = booster ? 255 : CurveHigh;
  int bLow  = booster ? 180 : CurveLow;   // 부스터 시에도 안쪽 바퀴를 높게 유지해 탈선 방지
  int bSpin = booster ? 255 : SpinSpeed;

  // 1. 직진 (0 1 0)
  if (L == LOW && C == HIGH && R == LOW) {
    motor_role(1, 1, bBase, bBase);
    last_state = 0;
  }
  // 2. 완만한 우회전 (0 1 1)
  else if (L == LOW && C == HIGH && R == HIGH) {
    motor_role(1, 1, bLow, bHigh); 
    last_state = 2;
  }
  // 3. 완만한 좌회전 (1 1 0)
  else if (L == HIGH && C == HIGH && R == LOW) {
    motor_role(1, 1, bHigh, bLow); 
    last_state = 1;
  }
  // 4. 급 우회전 / 90도 (0 0 1)
  else if (L == LOW && C == LOW && R == HIGH) {
    motor_role(-1, 1, bSpin, bSpin); 
    last_state = 2;
  }
  // 5. 급 좌회전 / 90도 (1 0 0)
  else if (L == HIGH && C == LOW && R == LOW) {
    motor_role(1, -1, bSpin, bSpin); 
    last_state = 1;
  }
  // 6. 라인 완전 이탈 (0 0 0) - 마지막 방향으로 복귀 시도
  else if (L == LOW && C == LOW && R == LOW) {
    if (last_state == 1) motor_role(1, -1, bSpin, bSpin);
    else if (last_state == 2) motor_role(-1, 1, bSpin, bSpin);
    else if (last_state == 0) motor_role(1, 1, bBase, bBase);
  }
  // 7. 정지 또는 교차로 (1 1 1)
  else if (L == HIGH && C == HIGH && R == HIGH) {
    motor_role(0, 0, 0, 0);
  }
}

// 모터 제어 통합 함수
void motor_role(int R_dir, int L_dir, int r_spd, int l_spd) {
  // 오른쪽 모터 제어
  if (R_dir == 1) {
    digitalWrite(RightMotor_1_pin, HIGH);
    digitalWrite(RightMotor_2_pin, LOW);
  } else if (R_dir == -1) {
    digitalWrite(RightMotor_1_pin, LOW);
    digitalWrite(RightMotor_2_pin, HIGH);
  }
  analogWrite(RightMotor_E_pin, R_dir == 0 ? 0 : r_spd);

  // 왼쪽 모터 제어
  if (L_dir == 1) {
    digitalWrite(LeftMotor_3_pin, HIGH);
    digitalWrite(LeftMotor_4_pin, LOW);
  } else if (L_dir == -1) {
    digitalWrite(LeftMotor_3_pin, LOW);
    digitalWrite(LeftMotor_4_pin, HIGH);
  }
  analogWrite(LeftMotor_E_pin, L_dir == 0 ? 0 : l_spd);
}
