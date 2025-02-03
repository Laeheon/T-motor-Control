#include "CAN_Format.h"
#include <arduino.h>
#include <SoftwareSerial.h>
#include <math.h>

// CAN communication speed setting
CAN_Format CAN_Driver1(1000000);

uint8_t driver_id = 0x03; // driver_id
bool motor_on_flag = false; // 모터 활성화 상태
bool pd_control_on = false; // PD 제어 활성화 상태

double Kp_value = 1;
double Kd_value = 0.1;

unsigned long last_update_time = 0; // 타이머 변수
const int update_interval = 100; // 50ms 간격

const double frequency = 1.0; // Sine wave frequency (1Hz)
const double amplitude = 0.3; // Sine wave amplitude

double current_value = 0.0; // 현재 수신된 전류 값
double desired_current_value = 0.0; // Sine 파형에 따른 Desired Current
double pd_output = 0.0; // PD 출력 값

void processCommand(String command);
void PD_Control();
void Sin_Control();

void setup() {
  // Serial communication 시작
  Serial.begin(115200);
  delay(1000);

  CAN_Driver1.Show_information(driver_id);
  CAN_Driver1.Initial_state(driver_id);
  delay(1000);

  Serial.println("Setup complete");
  Serial.println("'start' to activate the motor");
  Serial.println("Type 'PD Control On' to enable PD control, or 'PD Control Off' to disable it.");
}

void loop() {
  // Serial 명령 처리
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }

  if (motor_on_flag) {
    if (pd_control_on) {
      PD_Control(); // PD 제어 활성화
    } else {
      Sin_Control(); // 기본 Sine Control
    }
  }
}

// Sin 파형으로 토크 명령 전송
void Sin_Control() {
  unsigned long current_time = millis();
  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;

    // Sine 함수로 Desired Current 계산
    desired_current_value = amplitude * sin(2 * M_PI * frequency * (current_time / 1000.0));

    // Sine 값으로 모터에 토크 명령 전송
    CAN_Driver1.Torque_Control(driver_id, desired_current_value);

    // Debugging output
    Serial.print("Sin Control - Desired Current: ");
    Serial.println(desired_current_value);
  }
}

// Sine 파형에 대한 PD 제어 수행
void PD_Control() {
  unsigned long current_time = millis();

  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;

    // 1️⃣ Sine 함수로 Desired Current 계산
    desired_current_value = amplitude * sin(2 * M_PI * frequency * (current_time / 1000.0));

    // 2️⃣ 현재 전류 값 수신 (송신-수신 동기화)
    double received_current = CAN_Driver1.Torque_Control_PD(driver_id, pd_output);
    if (received_current != 0.0) {
      current_value = received_current; // 유효 값이 수신된 경우만 업데이트
    }

    // 3️⃣ 현재 오차 계산
    static double previous_current_error = 0.0;
    double current_error = desired_current_value - current_value;
    double d_current_error = ((current_error - previous_current_error) *50) / (1000.0);
    previous_current_error = current_error;

    // 4️⃣ PD 출력 계산
    pd_output = Kp_value * current_error + Kd_value * d_current_error;

    // 출력 제한
    if (pd_output > 1.0) pd_output = 1.0;
    if (pd_output < -1.0) pd_output = -1.0;

    // Debugging 출력
    Serial.print("PD Control - Desired Current: ");
    Serial.println(desired_current_value);
    Serial.print("PD Control - Past Current: ");
    Serial.println(current_value);
    Serial.print("PD Control - Current Error: ");
    Serial.println(current_error);
    Serial.print("PD Control - d_Current Error: ");
    Serial.println(d_current_error);
    Serial.print("PD Control - PD Output: ");
    Serial.println(pd_output);
  }
}

// Serial 명령 처리
void processCommand(String command) {
  if (command == "start" && !motor_on_flag) {
    CAN_Driver1.MotorOn(driver_id);
    motor_on_flag = true;
    Serial.println("Motor is now ON.");
  } else if (command == "stop" && motor_on_flag) {
    CAN_Driver1.MotorOff(driver_id);
    motor_on_flag = false;
    pd_control_on = false;
    Serial.println("Motor is now OFF.");
  } else if (command == "PD On") {
    pd_control_on = true;
    Serial.println("PD Control is now ON.");
  } else if (command == "PD Off") {
    pd_control_on = false;
    Serial.println("Switched back to Sine Control.");
  } else {
    Serial.println("Invalid command.");
  }
}
