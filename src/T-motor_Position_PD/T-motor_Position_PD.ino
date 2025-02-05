#include "CAN_Format.h"
#include <arduino.h>
#include <SoftwareSerial.h>
#include <math.h>

// CAN communication speed setting
CAN_Format CAN_Driver1(1000000);

uint8_t driver_id = 0x03; // driver_id 
bool motor_on_flag = false; // 모터 활성화 상태
bool pd_control_on = false; // PD 제어 활성화 상태
double last_desired_position = 0.0; // 마지막 desired position 저장

int input_stage = 0; // Current input stage
double motor_position = 0.0;
double motor_speed = 0.0;
double current_value = 0.0;
int Kp_value = 5;
int Kd_value = 1;

unsigned long last_update_time = 0; // Last update time
const int update_interval = 1; // Update interval (10ms)
const int update_interval2 = 10;
unsigned long start_time = 0; // 초기 시작 시간을 기록
const double frequency = 1.0; // Frequency (1Hz for 1-second period)
const double amplitude = 1.0; // Amplitude of the sine wave

double pd_output = 0.0; // PD 출력 값


void processCommand(String command);
void PD_Control();
void Sin_Control();

void setup() {
  // Serial communication speed
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
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // 처리 명령어
    if (input == "stop") {
      processCommand(input);
    } else if (input == "start") {
      processCommand(input);
    } else if (input == "PD On") {
      pd_control_on = true;
      Serial.println("PD Control is now ON.");
    } else if (input == "PD Off") {
      pd_control_on = false;
      Serial.println("PD Control is now OFF.");
    }
  }

  if (motor_on_flag) {
    if (pd_control_on) {
      PD_Control(); // PD 제어 함수 호출
    } else {
      Sin_Control(); // Sin 함수로 토크 제어
    }
  }
}


// Torque order sending according to sine wave
void Sin_Control() {
  unsigned long current_time = millis();
  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;

    // Calculating desired current using sine function
    double desired_position_value = amplitude * sin(2 * M_PI * frequency * (current_time / 1000.0));
    double desired_speed_value = amplitude * sin(2 * M_PI * frequency * (current_time / 1000.0));
    // Sending torque order to T-motor using sine value 
    CAN_Driver1.Position_Control(driver_id, desired_position_value, desired_speed_value, Kp_value, Kd_value);

    // Debugging output
    Serial.print("Sin Control - Desired Position: ");
    Serial.println(desired_position_value);
  }
}


// Sine 파형에 대한 PD 제어 수행
void PD_Control() {
  unsigned long current_time = millis();

  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;

    // 상대 시간 계산
    double elapsed_time = (current_time - start_time) / 1000.0;

    // 새로운 desired_position_value 계산 (사인 곡선 기반)
    double desired_position_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);

    // CAN 통신으로 현재 위치 받기
    double received_position = CAN_Driver1.Position_Control_PD(driver_id, pd_output);

    // 오차 계산
    double position_error = desired_position_value - received_position;

    // PD 제어 출력 계산
    double kp_value = 0.15;
    pd_output = kp_value * position_error;

    // PD 출력 제한
    pd_output = constrain(pd_output, -0.5, 0.5);

    // 디버깅 출력
    Serial.print("PD Control - Desired Position: ");
    Serial.println(desired_position_value);
    Serial.print("PD Control - Current Position: ");
    Serial.println(received_position);
    Serial.print("PD Control - Position Error: ");
    Serial.println(position_error);
    Serial.print("PD Control - PD Output: ");
    Serial.println(pd_output);
  }
}


void handleInput(String input) {
  if (input_stage == 0) {
    motor_position = input.toFloat();
    Serial.print("Motor Position set to: ");
    Serial.println(motor_position);
    input_stage++;
    Serial.println("Enter Motor Speed:");
  } else if (input_stage == 1) {
    motor_speed = input.toFloat();
    Serial.print("Motor Speed set to: ");
    Serial.println(motor_speed);
    input_stage++;
    Serial.println("Enter Current Value:");
  } else if (input_stage == 2) {
    current_value = input.toFloat();
    Serial.print("Current Value set to: ");
    Serial.println(current_value);
    input_stage++;
    Serial.println("Enter Kp Value:");
  } else if (input_stage == 3) {
    Kp_value = input.toInt();
    Serial.print("Kp Value set to: ");
    Serial.println(Kp_value);
    input_stage++;
    Serial.println("Enter Kd Value:");
  } else if (input_stage == 4) {
    Kd_value = input.toInt();
    Serial.print("Kd Value set to: ");
    Serial.println(Kd_value);

    Serial.println("All values received!");

    // CAN message sending 
    CAN_Driver1.Send_CAN_message(driver_id, motor_position, motor_speed, Kp_value, Kd_value, current_value);
    Serial.println("CAN message sent!");

    CAN_Driver1.Receive_CAN_message();

    input_stage = 0; // input stage initializing
    Serial.println("Enter Motor Position to restart or type 'stop' to turn off the motor.");
  }
}

void processCommand(String command) {
  if (command == "start" && !motor_on_flag) {
    CAN_Driver1.MotorOn(driver_id); 
    motor_on_flag = true; 
    input_stage = 0;
    Serial.println("Motor is now ON.");
    Serial.println("Enter Motor Position:");
  } else if (command == "stop" && motor_on_flag) {
    CAN_Driver1.MotorOff(driver_id); 
    motor_on_flag = false;
    pd_control_on = false; // PD 제어도 비활성화
    Serial.println("Motor is now OFF.");
  } else if (command == "start" && motor_on_flag) {
    Serial.println("Motor is already ON.");
  } else {
    Serial.println("Invalid command. Type 'start' to turn on the motor.");
  }
}
