#include "CAN_Format.h"
#include <arduino.h>
#include <SoftwareSerial.h>
#include <math.h>

// CAN communication speed setting
CAN_Format CAN_Driver1(1000000);

uint8_t driver_id = 0x03;      // driver_id 
bool motor_on_flag = false;    // Motor activation state
bool pd_control_on = false;    // PD Control activation state
double last_desired_position = 0.0; // Saving the last desired position 

int input_stage = 0;         // Current input stage
double motor_position = 0.0;
double motor_speed = 0.0;
double current_value = 0.0;
int Kp_value = 5;
int Kd_value = 1;

unsigned long last_update_time = 0; // Last update time
const int update_interval = 1;      // Update interval (ms)
unsigned long start_time = 0;       // Motor starting time 
const double frequency = 1.0;       // Sine wave frequency (Hz)
const double amplitude = 1.0;       // Sine wave amplitude

double pd_output = 0.0;             // PD output value 

void processCommand(String command);
void PD_Control();
void Sin_Control();

void setup() {
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
      PD_Control();
    } else {
      Sin_Control();
    }
  }
}

// Sin Control funciton 
void Sin_Control() {
  unsigned long current_time = millis();
  if (current_time - last_update_time >= update_interval) {
    last_update_time = current_time;
    
    // Calculation of elasped time after start tinme  
    double elapsed_time = (current_time - start_time) / 1000.0;
    double desired_position_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
    double desired_speed_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
    
    //Sending position control command to T-motor using sine wave 
    CAN_Driver1.Position_Control(driver_id, desired_position_value, desired_speed_value, Kp_value, Kd_value);

    Serial.print("Sin Control - Desired Position: ");
    Serial.println(desired_position_value);
  }
}

//Position PD Contorl function 
void PD_Control() {
    unsigned long current_time = millis();
    if (current_time - last_update_time >= update_interval) {
        last_update_time = current_time;
        
        //  Calculation of elapsed time 
        double elapsed_time = (current_time - start_time) / 1000.0;
        // Calculation of desired position and speed 
        double desired_position_value = amplitude * sin(2 * M_PI * frequency * elapsed_time);
        double desired_speed_value = amplitude * cos(2 * M_PI * frequency * elapsed_time);
        

        // Reception of current position and speed using CAN message format
        double received_position = CAN_Driver1.Receive_current_position();
        double received_speed = CAN_Driver1.received_speed_position_PD;
        
        // Calculation of error
        double position_error = desired_position_value - received_position;
        double speed_error = desired_speed_value - received_speed;
        
        // PD Control calculation 
        double kp = 0.25;   // Kp value 
        double kd = 0.05;   // Kd value
        pd_output = kp * position_error + kd * speed_error;
        pd_output = constrain(pd_output, -1.0, 1.0);
        
        
        // Sending position command to T-motor 
        CAN_Driver1.Position_Control_PD(driver_id, pd_output);
        
        Serial.print("PD Output: ");
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

    // Sending CAN message to T-motor
    CAN_Driver1.Send_CAN_message(driver_id, motor_position, motor_speed, Kp_value, Kd_value, current_value);
    Serial.println("CAN message sent!");

    CAN_Driver1.Receive_CAN_message();

    input_stage = 0;
    Serial.println("Enter Motor Position to restart or type 'stop' to turn off the motor.");
  }
}

void processCommand(String command) {
  if (command == "start" && !motor_on_flag) {
    // Start time initialization when motor start 
    start_time = millis();
    CAN_Driver1.MotorOn(driver_id); 
    motor_on_flag = true;
    input_stage = 0;
    Serial.println("Motor is now ON.");
    Serial.println("Enter Motor Position:");
  } else if (command == "stop" && motor_on_flag) {
    CAN_Driver1.MotorOff(driver_id); 
    motor_on_flag = false;
    pd_control_on = false;
    Serial.println("Motor is now OFF.");
  } else if (command == "start" && motor_on_flag) {
    Serial.println("Motor is already ON.");
  } else {
    Serial.println("Invalid command. Type 'start' to turn on the motor.");
  }
}
