#include "CAN_Format.h"
#include "arduino.h"
#include <FlexCAN_T4.h>
#include <cmath>
#include <iomanip>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>can1;


double Reduction_ratio =10; 
double Rated_voltage = 24; // (V)
double Rated_torque = 8.3; // (Nm)
double Rated_speed = 148;  // (rpm)
double Rated_current = 7.2; // (ADC)
double Peak_torque = 24.8; // (Nm)
double Peak_current = 23.2; // (ADC)
double Speed_constant = 100; // (rpm/V)
double Torque_constant = 0.123; // (Nm/A)
double Back_EMF_constant = 11.2; // (V/krpm) 
double Motor_constant = 0.24; // (Nm/sqrt(W))
double M_time_constant = 0.74; //(ms)
double E_time_constant = 0.42; //(ms)
double Peak_position = 12.5; //(rad)
double Peak_speed = 50;  //(rad/s)


// CAN open according to given baudrate
CAN_Format::CAN_Format(int BaudRate){
    this->CAN_BaudRate = BaudRate;
    Open_CAN();
}


//AK70-10 Motor information
void CAN_Format::Show_information(uint8_t drivers_id){
    Serial.println("Motor reduction ratio: " + String(Reduction_ratio));
    Serial.println("Motor torque constant: " + String(Torque_constant) + " Nm/A");
    Serial.println("Motor rated torque: " + String(Rated_torque) + " Nm");
    Serial.println("Motor peak torque: " + String(Peak_torque) + " Nm");
    Serial.println("Motor rated current: " + String(Rated_current) + " ADC");
    Serial.println("Motor peak current: " + String(Peak_current) + " ADC");
    Serial.println("Motor peak speed: " + String(Peak_speed) + " rad/s");
    Serial.println("Motor peak position: " + String(Peak_position) + " rad");

}

// CAN Open function
void CAN_Format::Open_CAN(){
    Serial.println("CAN Opening...");
    can1.begin();
    can1.setBaudRate(this->CAN_BaudRate);    
    delay(1000);
    Serial.println("CAN Activated!");
}

// CAN Reset function
void CAN_Format::Reset_CAN(){
    Serial.println("CAN Reseting...");
    can1.reset();
    Open_CAN();
    Serial.println("CAN Reset Finished!");
}

// CAN message sending frame function
void CAN_Format::Send_frame(MIT_CAN_Frame Sending_Frame, int length){
    CAN_message_t frame;
    frame.id = Sending_Frame.canID;
    frame.len = length;
    for (uint8_t i = 0; i < 8; i++) {
        frame.buf[i] = Sending_Frame.frame_byte[i];
    }

    can1.write(frame);
}

//AK70-10 Motor On
void CAN_Format::MotorOn(uint8_t driver_id){

    MIT_CAN_Frame enter_control_mode_frame;

    enter_control_mode_frame.canID= driver_id;
    enter_control_mode_frame.frame_byte[0]= 0xFF;
    enter_control_mode_frame.frame_byte[1]= 0xFF;
    enter_control_mode_frame.frame_byte[2]= 0xFF;
    enter_control_mode_frame.frame_byte[3]= 0xFF;
    enter_control_mode_frame.frame_byte[4]= 0xFF;
    enter_control_mode_frame.frame_byte[5]= 0xFF;
    enter_control_mode_frame.frame_byte[6]= 0xFF;
    enter_control_mode_frame.frame_byte[7]= 0xFC;

    Send_frame(enter_control_mode_frame,8);

}

//AK70-10 Motor Off
void CAN_Format::MotorOff(uint8_t driver_id){
    
    MIT_CAN_Frame enter_control_mode_frame;

    enter_control_mode_frame.canID= driver_id;
    enter_control_mode_frame.frame_byte[0]= 0xFF;
    enter_control_mode_frame.frame_byte[1]= 0xFF;
    enter_control_mode_frame.frame_byte[2]= 0xFF;
    enter_control_mode_frame.frame_byte[3]= 0xFF;
    enter_control_mode_frame.frame_byte[4]= 0xFF;
    enter_control_mode_frame.frame_byte[5]= 0xFF;
    enter_control_mode_frame.frame_byte[6]= 0xFF;
    enter_control_mode_frame.frame_byte[7]= 0xFD;

    Send_frame(enter_control_mode_frame,8);
}


// AK70-10 Motor initial state handling
void CAN_Format::Initial_state(uint8_t driver_id) {
    Serial.println("Setting Initial State...");

    MIT_CAN_Frame Initial_frame;
    int length;
    CAN_message_t received_frame;

    length = received_frame.len;

    Initial_frame.canID = received_frame.id;
    for (int i = 0; i < length; i++) {
        Initial_frame.frame_byte[i] = received_frame.buf[i];
    }

    Serial.println("CAN message successfully read!");
    Serial.print("CAN ID: ");
    Serial.println(Initial_frame.canID);

    Serial.print("Frame Data: ");
    for (int i = 0; i < length; i++) {
        Serial.print(Initial_frame.frame_byte[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    int driver_id_initial = Initial_frame.frame_byte[0];

    int motor_position = (Initial_frame.frame_byte[1] << 8) | Initial_frame.frame_byte[2];

    double scaled_position = Receiving_scaling_value(motor_position, Peak_position, -Peak_position, 16);

       
    int motor_speed = ((Initial_frame.frame_byte[3] << 4) | (Initial_frame.frame_byte[4] >> 4)) & 0xFFF;


    double scaled_speed = Receiving_scaling_value(motor_speed, Peak_speed, -Peak_speed, 12);

  
    int current_value = ((Initial_frame.frame_byte[4] & 0xF) << 8) | Initial_frame.frame_byte[5];

    double scaled_current = Receiving_scaling_value(current_value, Peak_torque, -Peak_torque, 12); 

        
    int motor_temperature = Initial_frame.frame_byte[6];

    
    int motor_error_flag = Initial_frame.frame_byte[7];

    Initial_frame.initial_position = scaled_position;
    Initial_frame.initial_speed = scaled_speed;
    Initial_frame.initial_current = scaled_current;
    Initial_frame.initial_temperature = motor_temperature;
    Initial_frame.initial_error_flag = motor_error_flag;

    this->initial_position = scaled_position;

    // Printing initial data
    Serial.println("Initial Data:");
    Serial.println("Motor Id: ");
    Serial.println(driver_id_initial);
    
    Serial.print("Motor Position: ");
    Serial.println(scaled_position);

    Serial.print("Motor Speed: ");
    Serial.println(scaled_speed);

    Serial.print("Current Value: ");
    Serial.println(scaled_current);

    Serial.print("Motor Temperature: ");
    Serial.println(motor_temperature);

    Serial.print("Motor Error Flag: ");
    Serial.println(motor_error_flag);
}

// Position control function
void CAN_Format::Position_Control(uint8_t driver_id, double Motor_position, double Motor_speed, int Kp_value, int Kd_value){
    CAN_message_t frame;
    frame.id = driver_id;  
    frame.len = 8;        

    int pos_hex = Sending_scaling_value(Motor_position, Peak_position, -Peak_position, 16);
    int spd_hex = Sending_scaling_value(Motor_speed, Peak_speed, -Peak_speed, 12);
    int Kp_hex = Sending_scaling_value(Kp_value, 500, 0, 12);
    int Kd_hex = Sending_scaling_value(Kd_value, 5, 0, 12);
    int cur_hex = Sending_scaling_value(0, Peak_torque, -Peak_torque, 12);
   
    frame.buf[0] = (pos_hex >> 8) & 0xFF;         // pos_hex High 8 bits
    frame.buf[1] = pos_hex & 0xFF;                // pos_hex Low 8 bits
    frame.buf[2] = (spd_hex >> 4) & 0xFF;         // spd_hex High 8 bits
    frame.buf[3] = ((spd_hex & 0xF) << 4) | ((Kp_hex >> 8) & 0xF); // spd_hex Low bits + Kp_hex High 4 bits
    frame.buf[4] = Kp_hex & 0xFF;                 // Kp_hex Low 8 bitrs
    frame.buf[5] = (Kd_hex >> 4) & 0xFF;          // Kd_hex High 8 bits
    frame.buf[6] = ((Kd_hex & 0xF) << 4) | ((cur_hex >> 8) & 0xF); // Kd Low 4 bits + cur_hex High 4 bits
    frame.buf[7] = cur_hex & 0xFF;                // cur_hex Low 8 bits

    can1.write(frame);  

    Serial.println("Position control");    
}

// Speed Control function
void CAN_Format::Speed_Control(uint8_t driver_id, double Motor_speed, int Kd_value) 
{
    
    CAN_message_t frame;
    frame.id = driver_id;  
    frame.len = 8;        

    if(Kd_value > 5)
    {
        Kd_value = 5;
    }
    else if(Kd_value < 0)
    {
        Kd_value = 0;
    }

    int pos_hex = Sending_scaling_value(0, Peak_position, -Peak_position, 16);
    int spd_hex = Sending_scaling_value(Motor_speed, Peak_speed, -Peak_speed, 12);
    int Kp_hex = Sending_scaling_value(0, 500, 0, 12);
    int Kd_hex = Sending_scaling_value(Kd_value, 5, 0, 12);
    int cur_hex = Sending_scaling_value(0, Peak_torque, -Peak_torque, 12);

    frame.buf[0] = (pos_hex >> 8) & 0xFF;         // pos_hex High 8 bits
    frame.buf[1] = pos_hex & 0xFF;                // pos_hex Low 8 bits
    frame.buf[2] = (spd_hex >> 4) & 0xFF;         // spd_hex High 8 bits
    frame.buf[3] = ((spd_hex & 0xF) << 4) | ((Kp_hex >> 8) & 0xF); // spd_hex Low bits + Kp_hex High 4 bits
    frame.buf[4] = Kp_hex & 0xFF;                 // Kp_hex Low 8 bits
    frame.buf[5] = (Kd_hex >> 4) & 0xFF;          // Kd_hex High 8 bits
    frame.buf[6] = ((Kd_hex & 0xF) << 4) | ((cur_hex >> 8) & 0xF); // Kd Low 4 bits + cur_hex High 4 bits
    frame.buf[7] = cur_hex & 0xFF;                // cur_hex Low 8 bits
    can1.write(frame);  

    Serial.println("Speed Control");
}

void CAN_Format::Torque_Control(uint8_t driver_id, int Kp_value, int Kd_value, double Current_value) 
{
    
    CAN_message_t frame;
    frame.id = driver_id;  
    frame.len = 8;        
    if(Kp_value > 500)
    {
        Kp_value  = 500;
    }
    else if(Kp_value < 0)
    {
        Kp_value = 0;
    }

    if(Kd_value > 5)
    {
        Kd_value = 5;
    }
    else if(Kd_value < 0)
    {
        Kd_value = 0;
    }

    int pos_hex = Sending_scaling_value(0, Peak_position, -Peak_position, 16);
    int spd_hex = Sending_scaling_value(0, Peak_speed, -Peak_speed, 12);
    int Kp_hex = Sending_scaling_value(Kp_value, 500, 0, 12);
    int Kd_hex = Sending_scaling_value(Kd_value, 5, 0, 12);
    int cur_hex = Sending_scaling_value(Current_value, Peak_torque, -Peak_torque, 12);

    frame.buf[0] = (pos_hex >> 8) & 0xFF;         // pos_hex High 8 bits
    frame.buf[1] = pos_hex & 0xFF;                // pos_hex Low 8 bits
    frame.buf[2] = (spd_hex >> 4) & 0xFF;         // spd_hex High 8 bits
    frame.buf[3] = ((spd_hex & 0xF) << 4) | ((Kp_hex >> 8) & 0xF); // spd_hex Low bits + Kp_hex High 4 bits
    frame.buf[4] = Kp_hex & 0xFF;                 // Kp_hex Low 8 bits
    frame.buf[5] = (Kd_hex >> 4) & 0xFF;          // Kd_hex High 8 bits
    frame.buf[6] = ((Kd_hex & 0xF) << 4) | ((cur_hex >> 8) & 0xF); // Kd Low 4 bits + cur_hex High 4 bits
    frame.buf[7] = cur_hex & 0xFF;                // cur_hex Low 8 bits

    can1.write(frame);  

    Serial.println("Torque Control");
}

// Sending CAN message according to the AK70-10 format
void CAN_Format::Send_CAN_message(uint8_t driver_id, double Motor_position,  double Motor_speed, int Kp_value, int Kd_value, double Current_value) 
{
    
    CAN_message_t frame;
    frame.id = driver_id;  
    frame.len = 8;        
    if(Kp_value > 500)
    {
        Kp_value  = 500;
    }
    else if(Kp_value < 0)
    {
        Kp_value = 0;
    }

    if(Kd_value > 5)
    {
        Kd_value = 5;
    }
    else if(Kd_value < 0)
    {
        Kd_value = 0;
    }


    int pos_hex = Sending_scaling_value(Motor_position, Peak_position, -Peak_position, 16);
    int spd_hex = Sending_scaling_value(Motor_speed, Peak_speed, -Peak_speed, 12);
    int Kp_hex = Sending_scaling_value(Kp_value, 500, 0, 12);
    int Kd_hex = Sending_scaling_value(Kd_value, 5, 0, 12);
    int cur_hex = Sending_scaling_value(Current_value, Peak_torque, -Peak_torque, 12);

    frame.buf[0] = (pos_hex >> 8) & 0xFF;         // pos_hex High 8 bits
    frame.buf[1] = pos_hex & 0xFF;                // pos_hex Low 8 bits
    frame.buf[2] = (spd_hex >> 4) & 0xFF;         // spd_hex High 8 bits
    frame.buf[3] = ((spd_hex & 0xF) << 4) | ((Kp_hex >> 8) & 0xF); // spd_hex Low bits + Kp_hex High 4 bits
    frame.buf[4] = Kp_hex & 0xFF;                 // Kp_hex Low 8 bits
    frame.buf[5] = (Kd_hex >> 4) & 0xFF;          // Kd_hex High 8 bits
    frame.buf[6] = ((Kd_hex & 0xF) << 4) | ((cur_hex >> 8) & 0xF); // Kd Low 4 bits + cur_hex High 4 bits
    frame.buf[7] = cur_hex & 0xFF;                // cur_hex Low 8 bits

    can1.write(frame);  

    Serial.println("CAN message sent successfully!");
}

// Scaling value from hexadecimal to decimal when receiving data from AK70-10 Motor
double CAN_Format::Receiving_scaling_value(int value, double max_value, double min_value, int bit_resolution) {
    double range = max_value - min_value;
    double int_max = pow(2, bit_resolution) - 1;
    return min_value + ((value * range) / int_max) ;
}

// Scaling value from decimal to hexadecimal when sending data to AK70-10 Motor
int CAN_Format::Sending_scaling_value(double value, double max_value, double min_value, int bit_resolution) {
   
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }
    double range = max_value - min_value;
    int int_range = pow(2, bit_resolution) - 1;
    int scaled_value = (value - min_value) * int_range / range;

    return scaled_value;
}

// Receiving and interpreting data 
void CAN_Format::Receive_CAN_message() {
    CAN_message_t received_frame;

    if (can1.read(received_frame)) {
        Serial.println("CAN message received!");

        MIT_CAN_Frame receiving_frame;
        for (int i = 0; i < 8; i++) {
            receiving_frame.frame_byte[i] = received_frame.buf[i];
        }

     
        int driver_id = receiving_frame.frame_byte[0];

        int motor_position = (received_frame.buf[1] << 8) | received_frame.buf[2];

        double scaled_position = Receiving_scaling_value(motor_position, Peak_position, -Peak_position, 16);

       
        int motor_speed = ((received_frame.buf[3] << 4) | (received_frame.buf[4] >> 4)) & 0xFFF;


        double scaled_speed = Receiving_scaling_value(motor_speed, Peak_speed, -Peak_speed, 12);

  
        int current_value = ((received_frame.buf[4] & 0xF) << 8) | received_frame.buf[5];

        double scaled_current = Receiving_scaling_value(current_value, Peak_torque, -Peak_torque, 12); 

        
        int motor_temperature = receiving_frame.frame_byte[6];

    
        int motor_error_flag = receiving_frame.frame_byte[7];

        Serial.println("Received Data:");
        Serial.print("Driver ID: ");
        Serial.println(driver_id);

        Serial.print("Motor Position (HEX): 0x");        
        Serial.println(motor_position, HEX);

        Serial.print("Motor Position: ");
        Serial.println(scaled_position);

        Serial.print("Motor Speed: ");
        Serial.println(scaled_speed);

        Serial.print("Current Value: ");
        Serial.println(scaled_current);

        Serial.print("Motor Temperature: ");
        Serial.println(motor_temperature);

        Serial.print("Motor Error Flag: ");
        Serial.println(motor_error_flag);

        if (motor_error_flag == 1) {
            MotorOff(driver_id);
            Serial.println("Error flag is 1, motor is turned off.");
        }
        else {
        Serial.println("No error in CAN message .");
    }
    }
}
// Receiving and interpreting data 
double CAN_Format::Receive_current_torque(){
    CAN_message_t received_frame;

    if (can1.read(received_frame)) {
        Serial.println("CAN message received!");

        MIT_CAN_Frame receiving_frame;
        for (int i = 0; i < 8; i++) {
            receiving_frame.frame_byte[i] = received_frame.buf[i];
        }

  
        int current_value = ((received_frame.buf[4] & 0xF) << 8) | received_frame.buf[5];

        double scaled_current = Receiving_scaling_value(current_value, Peak_torque, -Peak_torque, 12); 
    
        int motor_error_flag = receiving_frame.frame_byte[7];


        Serial.print("Current Value: ");
        Serial.println(scaled_current);


        if (motor_error_flag == 1) {
            MotorOff(driver_id);
            Serial.println("Error flag is 1, motor is turned off.");
            return 0.0;
        }
        else {
        Serial.println("No error in CAN message .");
        return scaled_current;
        }
    }
    return 0.0;
}
double CAN_Format::Receive_current_position(){
    CAN_message_t received_frame;

    if (can1.read(received_frame)) {
        Serial.println("CAN message received!");

        MIT_CAN_Frame receiving_frame;
        for (int i = 0; i < 8; i++) {
            receiving_frame.frame_byte[i] = received_frame.buf[i];
        }
        int motor_position = (received_frame.buf[1] << 8) | received_frame.buf[2];

        double scaled_position = Receiving_scaling_value(motor_position, Peak_position, -Peak_position, 16);
    
        int motor_error_flag = receiving_frame.frame_byte[7];


        Serial.print("Current Position Value: ");
        Serial.println(scaled_position);


        if (motor_error_flag == 1) {
            MotorOff(driver_id);
            Serial.println("Error flag is 1, motor is turned off.");
            return 0.0;
        }
        else {
        Serial.println("No error in CAN message .");
        return scaled_position;
        }
    }
    return 0.0;
}

double CAN_Format::Receive_current_speed(){
    CAN_message_t received_frame;

    if (can1.read(received_frame)) {
        Serial.println("CAN message received!");

        MIT_CAN_Frame receiving_frame;
        for (int i = 0; i < 8; i++) {
            receiving_frame.frame_byte[i] = received_frame.buf[i];
        }

        int motor_speed = ((received_frame.buf[3] << 4) | (received_frame.buf[4] >> 4)) & 0xFFF;

        double scaled_speed = Receiving_scaling_value(motor_speed, Peak_speed, -Peak_speed, 12);
    
        int motor_error_flag = receiving_frame.frame_byte[7];


        Serial.print("Current Speed Value: ");
        Serial.println(scaled_speed);


        if (motor_error_flag == 1) {
            MotorOff(driver_id);
            Serial.println("Error flag is 1, motor is turned off.");
            return 0.0;
        }
        else {
        Serial.println("No error in CAN message .");
        return scaled_speed;
        }
    }
    return 0.0;
}

double CAN_Format::Position_Control_PD(uint8_t driver_id, double Current_value) {
    CAN_message_t frame;
    frame.id = driver_id;  
    frame.len = 8;        

    // 송신 데이터 생성
    int pos_hex = Sending_scaling_value(0, Peak_position, -Peak_position, 16);
    int spd_hex = Sending_scaling_value(0, Peak_speed, -Peak_speed, 12);
    int Kp_hex = Sending_scaling_value(0, 500, 0, 12);
    int Kd_hex = Sending_scaling_value(0, 5, 0, 12);
    int cur_hex = Sending_scaling_value(Current_value, Peak_torque, -Peak_torque, 12);

    frame.buf[0] = (pos_hex >> 8) & 0xFF;         // pos_hex High 8 bits
    frame.buf[1] = pos_hex & 0xFF;                // pos_hex Low 8 bits
    frame.buf[2] = (spd_hex >> 4) & 0xFF;         // spd_hex High 8 bits
    frame.buf[3] = ((spd_hex & 0xF) << 4) | ((Kp_hex >> 8) & 0xF); // spd_hex Low bits + Kp_hex High 4 bits
    frame.buf[4] = Kp_hex & 0xFF;                 // Kp_hex Low 8 bits
    frame.buf[5] = (Kd_hex >> 4) & 0xFF;          // Kd_hex High 8 bits
    frame.buf[6] = ((Kd_hex & 0xF) << 4) | ((cur_hex >> 8) & 0xF); // Kd Low 4 bits + cur_hex High 4 bits
    frame.buf[7] = cur_hex & 0xFF;                // cur_hex Low 8 bits

    // CAN 메시지 송신
    if (can1.write(frame)) {
        Serial.println("Torque PD Control - Command sent successfully.");
    } else {
        Serial.println("Torque PD Control - Command send failed.");
        return 0.0; // 송신 실패 시 기본값 반환
    }

    // 송신 후 수신 대기
    CAN_message_t received_frame;
    unsigned long start_time = millis();  // 수신 대기 시작 시간
    const unsigned long timeout = 100;    // 최대 대기 시간 (ms)
    int retry_count = 0;                 // 재시도 횟수
    const int max_retries = 100;           // 최대 재시도 횟수

    while (retry_count < max_retries) {
        while (millis() - start_time < timeout) {
            if (can1.read(received_frame)) {
                Serial.println("Position PD Control - Response received!");

        int motor_position = (received_frame.buf[1] << 8) | received_frame.buf[2];

        double scaled_position = Receiving_scaling_value(motor_position, Peak_position, -Peak_position, 16);

                int motor_error_flag = received_frame.buf[7];
                if (motor_error_flag == 1) {
                    MotorOff(driver_id);
                    Serial.println("Error flag is 1, motor is turned off.");
                    return 0.0; // 오류 시 기본값 반환
                } else {
                    Serial.println("No error in CAN message.");
                    return scaled_position; // 수신된 값 반환
                }
            }
        }

        // 타임아웃 발생
        Serial.println("Position PD Control - Response timeout. Retrying...");
        retry_count++;
        start_time = millis(); // 타임아웃 발생 시 재시작
    }

    // 최대 재시도 후 실패 처리
    Serial.println("Position PD Control - Max retries reached. Returning default value.");
    return 0.0; // 재시도 후 실패 시 기본값 반환
}


