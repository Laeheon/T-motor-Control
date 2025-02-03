#ifndef CAN_Format_H 
#define CAN_Format_H

#include "Arduino.h"
#include <FlexCAN_T4.h> 



struct MIT_CAN_Frame {
    uint32_t canID;             
    byte frame_byte[8];
    double initial_position;  
    double initial_speed;      
    double initial_current;    
    int initial_temperature;   
    int initial_error_flag;
};


class CAN_Format
{
public:

CAN_Format(int Baudrate);
void Show_information(uint8_t drivers_id);
void Receive_CAN_message();
double Receiving_scaling_value(int value, double max_value, double min_value, int bit_resolution);
void Send_frame(MIT_CAN_Frame Sending_Frame, int length);
void Read_frame(MIT_CAN_Frame Reading_frame, int length);
void Send_CAN_message(uint8_t driver_id, double Motor_position, double Motor_speed, int Kp_value, int Kd_value, double Current_value);
int Sending_scaling_value(double value, double max_value, double min_value, int bit_resolution);
void Initial_state(uint8_t driver_id);
void Open_CAN();
void Reset_CAN();
void MotorOn(uint8_t driver_id);
void MotorOff(uint8_t driver_id);
void Position_Control(uint8_t driver_id, double Motor_position, double Motor_speed, int Kp_value, int Kd_value);
void Speed_Control(uint8_t driver_id, double Motor_speed, int Kd_value);
void Torque_Control(uint8_t driver_id, double Current_value); 
double Receive_current_torque();
double Torque_Control_PD(uint8_t driver_id, double Current_value);
double initial_position;

private:
  int CAN_BaudRate;
  uint8_t driver_id;
};
#endif