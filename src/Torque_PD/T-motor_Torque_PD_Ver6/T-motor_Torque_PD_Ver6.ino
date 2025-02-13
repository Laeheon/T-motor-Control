#include "CAN_Format.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <IntervalTimer.h>

// CAN communication speed setting (e.g., 1Mbps)
CAN_Format CAN_Driver1(1000000);

uint8_t driver_id = 0x03;         // Driver ID
bool motor_on_flag = false;       // Motor activation state
bool pd_control_on = false;       // PD control activation state

double Kp_value = 1.5;
double Kd_value = 0.3;

const double frequency = 1.0;     // Sine wave frequency (1Hz)
const double amplitude = 0.3;     // Sine wave amplitude

double current_value = 0.0;       // Current received value
double desired_current_value = 0.0; // Desired current according to sine wave
double pd_output = 0.0;           // PD output
double current_error = 0.0;
double d_current_error = 0.0;
// IntervalTimer for 1ms control period
IntervalTimer myTimer;
volatile bool loopFlag = false;   // Flag set by timer every 1ms

// Function declarations
void processCommand(String command);
void PD_Control();
void Sin_Control();

// Timer callback: sets the loopFlag every 1ms (1000µs)
void timerCallback() {
  loopFlag = true;
}

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  delay(1000);

  // Initialize CAN communication and display information
  CAN_Driver1.Show_information(driver_id);
  CAN_Driver1.Initial_state(driver_id);
  delay(1000);

  // Start the IntervalTimer to call timerCallback() every 1ms
  myTimer.begin(timerCallback, 1000);

  Serial.println("Setup complete");
  Serial.println("Type 'start' to activate the motor");
  Serial.println("Type 'stop' to deactivate the motor");
  Serial.println("Type 'PD On' to enable PD control, or 'PD Off' to disable it.");
}

void loop() {
  // Process serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }

  // Call the appropriate control function every 1ms as indicated by the timer flag
  if (loopFlag) {
    loopFlag = false;
    if (motor_on_flag) {
      if (pd_control_on) {
        PD_Control();
      } else {
        Sin_Control();
      }
    }
  }
}

// Sine Control: sends a torque command based on a sine wave
// Also measures and prints the function call interval
void Sin_Control() {
  static unsigned long last_sin_time = micros();
  unsigned long current_sin_time = micros();
  unsigned long sin_interval = current_sin_time - last_sin_time;
  last_sin_time = current_sin_time;
  Serial.print("Sin_Control interval (us): ");
  Serial.println(sin_interval);

  unsigned long current_time = millis();
  double desired_current_value = amplitude * sin(2 * M_PI * frequency * (current_time / 1000.0));

  // Send torque command to the T-motor using the sine wave value
  CAN_Driver1.Torque_Control(driver_id, desired_current_value);

  // Debug output
  Serial.print("Sin Control - Desired Current: ");
  Serial.println(desired_current_value);
}

// PD Control: performs PD control based on the sine wave reference
// Also measures and prints the function call interval
void PD_Control() {
  static unsigned long last_pd_time = micros();
  unsigned long current_pd_time = micros();
  unsigned long pd_interval = current_pd_time - last_pd_time;
  last_pd_time = current_pd_time;
  //Serial.print("PD_Control interval (us): ");
  //Serial.println(pd_interval);

  unsigned long current_time = millis();
  static unsigned long start_time = current_time; // Initialize start time once
  double elapsed_time = (current_time - start_time) / 1000.0; // Elapsed time in seconds

  double desired_current_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
  // Receive the current value from the motor via CAN

  // PD control computation
  static double previous_current_error = 0.0;
  current_error = desired_current_value - current_value;
  d_current_error = ((current_error - previous_current_error) * 50) / 1000.0;
  previous_current_error = current_error;
  double pd_output = Kp_value * current_error + Kd_value * d_current_error;
  pd_output = constrain(pd_output, -1.0, 1.0);

  // Send PD torque command to the T-motor
  CAN_Driver1.Torque_Control_PD(driver_id, pd_output);

  // Debug output
  Serial.print("PD Control - PD Output: ");
  Serial.println(pd_output);

  double received_current = CAN_Driver1.Receive_current_torque();
  if (received_current != 0.0) {
    current_value = received_current;
  } else {
    Serial.println("Warning: Received current value is 0.0, keeping previous value.");
  }

}

// Serial command processing
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
