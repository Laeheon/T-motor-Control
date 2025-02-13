#include "CAN_Format.h"       // Custom CAN communication library
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <IntervalTimer.h>

// Set CAN communication speed (e.g., 1Mbps)
CAN_Format CAN_Driver1(1000000);

uint8_t driver_id = 0x03;      // Driver ID
bool motor_on_flag = false;    // Motor activation state
bool pd_control_on = false;    // PD control activation state

IntervalTimer myTimer;         // Hardware timer object
volatile bool loopFlag = false; // Flag set every 1ms by the interrupt

// Control related variables
int input_stage = 0;         
double motor_position = 0.0;
double motor_speed = 0.0;
double current_value = 0.0;
int Kp_value = 5;
int Kd_value = 1;

// Motor start time (millis)
unsigned long start_time = 0;       

// Parameters for sine wave control
const double frequency = 1.0;       // Frequency (Hz)
const double amplitude = 1.0;       // Amplitude

// Variables for PD control
double pd_output = 0.0;             
double received_position = 0.0;
double received_speed = 0.0;

// Function declarations
void processCommand(String command);
void PD_Control();
void Sin_Control();

// ===== Interrupt Service Routine (ISR) =====
// Called every 1ms to set loopFlag to true
void timerCallback() {
  loopFlag = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Delay for Serial monitor stabilization

  // Use IntervalTimer to call timerCallback() every 1ms (1000µs)
  myTimer.begin(timerCallback, 1000);

  // CAN initialization and information display
  CAN_Driver1.Show_information(driver_id);
  CAN_Driver1.Initial_state(driver_id);
  delay(1000);

  Serial.println("Setup complete");
  Serial.println("Enter 'start' to activate the motor.");
  Serial.println("Enter 'PD On' or 'PD Off' to enable or disable PD control.");
}

void loop() {
  // ===== Serial command processing =====
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "stop") {
      processCommand(input);
    } else if (input == "start") {
      processCommand(input);
    } else if (input == "PD On") {
      pd_control_on = true;
      Serial.println("PD Control has been activated.");
    } else if (input == "PD Off") {
      pd_control_on = false;
      Serial.println("PD Control has been deactivated.");
    }
  }

  // ===== Call control functions every 1ms =====
  // Executed when loopFlag is set by the IntervalTimer interrupt
  if (loopFlag) {
    loopFlag = false;  // Reset flag

    if (motor_on_flag) {
      if (pd_control_on) {
        PD_Control();   // PD control mode
      } else {
        Sin_Control();  // Sine wave control mode
      }
    }
  }
}

// ===== Sine Wave Control Function =====
// Called every 1ms to calculate sine-based position and speed values
// and send a CAN message. Measures and prints the call interval.
void Sin_Control() {
  static unsigned long last_sin_time = micros();
  unsigned long current_sin_time = micros();
  unsigned long sin_interval = current_sin_time - last_sin_time;
  last_sin_time = current_sin_time;
  //Serial.print("Sin_Control interval (us): ");
  //Serial.println(sin_interval);

  unsigned long current_time = millis();
  double elapsed_time = (current_time - start_time) / 1000.0; // Elapsed time (seconds)

  // Calculate desired position and speed using sine function
  double desired_position_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
  double desired_speed_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
  
  // Send position control command via CAN message
  CAN_Driver1.Position_Control(driver_id, desired_position_value, desired_speed_value, Kp_value, Kd_value);
}

// ===== PD Control Function =====
// Called every 1ms to calculate the error between the desired and current values,
// perform PD control, and send a CAN message. Measures and prints the call interval.
void PD_Control() {
  static unsigned long last_pd_time = micros();
  unsigned long current_pd_time = micros();
  unsigned long pd_interval = current_pd_time - last_pd_time;
  last_pd_time = current_pd_time;
  //Serial.print("PD_Control interval (us): ");
  //Serial.println(pd_interval);

  unsigned long current_time = millis();
  double elapsed_time = (current_time - start_time) / 1000.0; // Elapsed time (seconds)

  // Calculate desired position and speed (using sine and cosine functions)
  double desired_position_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
  double desired_speed_value = amplitude * cos(2 * M_PI * frequency * elapsed_time);
  
  // Calculate errors
  double position_error = desired_position_value - received_position;
  double speed_error = desired_speed_value - received_speed;
  
  // PD control computation (adjust gains as needed)
  double kp = 0.25;
  double kd = 0.05;
  pd_output = kp * position_error + kd * speed_error;
  pd_output = constrain(pd_output, -1.0, 1.0);
  
  // Send PD control command via CAN message
  CAN_Driver1.Position_Control_PD(driver_id, pd_output);
  
  // Receive sensor data (current position, speed) via CAN
  received_position = CAN_Driver1.Receive_current_position();
  received_speed = CAN_Driver1.received_speed_position_PD;
}

// ===== Serial Command Processing Function =====
// "start": activates the motor and begins control,
// "stop": deactivates the motor.
void processCommand(String command) {
  if (command == "start" && !motor_on_flag) {
    start_time = millis();                // Initialize motor start time
    CAN_Driver1.MotorOn(driver_id);         // Send motor on command
    motor_on_flag = true;
    input_stage = 0;
    Serial.println("Motor has been turned on.");
    Serial.println("Please enter the Motor Position value:");
  } else if (command == "stop" && motor_on_flag) {
    CAN_Driver1.MotorOff(driver_id);        // Send motor off command
    motor_on_flag = false;
    pd_control_on = false;
    Serial.println("Motor has been turned off.");
  } else if (command == "start" && motor_on_flag) {
    Serial.println("Motor is already on.");
  } else {
    Serial.println("Invalid command. Please enter 'start' to activate the motor.");
  }
}
