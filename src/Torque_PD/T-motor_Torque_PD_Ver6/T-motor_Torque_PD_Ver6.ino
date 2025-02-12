#include "CAN_Format.h"
#include <arduino.h>
#include <SoftwareSerial.h>
#include <math.h>

// CAN communication speed setting
CAN_Format CAN_Driver1(1000000);

uint8_t driver_id = 0x03; // driver_id
bool motor_on_flag = false; // Motor activation state
bool pd_control_on = false; // PD control activation state

double Kp_value = 1.0;
double Kd_value = 0.2;

unsigned long last_update_time = 0; // Timer variable
const int update_interval = 1; // 1ms interval

const double frequency = 1.0; // Sine wave frequency (1Hz)
const double amplitude = 0.3; // Sine wave amplitude

double current_value = 0.0; // current received current value
double desired_current_value = 0.0; // Desired Current according to sine wave
double pd_output = 0.0; // PD Output 

void processCommand(String command);
void PD_Control();
void Sin_Control();

void setup() {
  // Serial communication start
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
  // Serial order processing
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }

  if (motor_on_flag) {
    if (pd_control_on) {
      PD_Control(); // PD control acivation
    } else {
      Sin_Control(); // Basic Sine wave current activation
    }
  }
}

// Torque order sending according to sine wave
void Sin_Control() {
  unsigned long current_time = millis();
  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;

    // Calculating desired current using sine function
    desired_current_value = amplitude * sin(2 * M_PI * frequency * (current_time / 1000.0));

    // Sending torque order to T-motor using sine value 
    CAN_Driver1.Torque_Control(driver_id, desired_current_value);

    // Debugging output
    Serial.print("Sin Control - Desired Current: ");
    Serial.println(desired_current_value);
  }
}

// PD control to Sine wave
void PD_Control() {
  unsigned long current_time = millis();

  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;

    // Relative elapsed time calculation
    static unsigned long start_time = 0;
    if (start_time == 0) start_time = current_time; // Initialize start time
    double elapsed_time = (current_time - start_time) / 1000.0; // Time in seconds

    // Calculating desired current using sine function
    double desired_current_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);

    // Receiving present current value
    double received_current = CAN_Driver1.received_torque_PD;
    if (received_current != 0.0) {
      current_value = received_current; // Updating value when received valid current value
    } else {
      Serial.println("Warning: Received current value is 0.0, keeping previous value.");
    }

    // Calculating current error
    static double previous_current_error = 0.0;
    double current_error = desired_current_value - current_value;
    double d_current_error = ((current_error - previous_current_error) * 50) / 1000.0; // Derivative of error
    previous_current_error = current_error;
    // Calculating PD output
    double pd_output = Kp_value * current_error + Kd_value * d_current_error;

    // Limitation of PD output
    pd_output = constrain(pd_output, -1.0, 1.0);
    CAN_Driver1.Torque_Control_PD(driver_id, pd_output);
    // Debugging printing
    Serial.println(pd_output);
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
