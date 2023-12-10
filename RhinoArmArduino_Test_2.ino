#include <Wire.h>
#define SLAVE_ADDRESS 0x08
//############################################

// Dec 10, 2023   Merging Functions and communications
// On Git


//=================================================================================================
// Hardware constants
const int IF_A_DIR = 2;           // D02 - Interface A - Direction Control
const int IF_A_ENABLE_PWM = 3;    // D03 - Interface A - H-Bridge Enable Control
const int IF_A_LIMIT = 4;         // D04 - Interface A - Limit Switch
const int IF_A_OPT_A = 5;         // D05 - Interface A - Shaft Encoder - A
const int IF_A_OPT_B = 6;         // D06 - Interface A - Shaft Encoder - B

const int IF_B_OPT_A = 7;         // D07 - Interface B - Shaft Encoder - A
const int IF_B_OPT_B = 8;         // D08 - Interface B - Shaft Encoder - A
const int IF_B_ENABLE_PWM =  9;   // D09 - Interface B - H-Bridge Enable Control
const int IF_B_DIR = 10;          // D10 - Interface B - Direction Control
const int IF_B_LIMIT = 16;        // D16 - Interface B - Limit Switch

const int IF_C_ENABLE_PWM = 11;   // D11 - Interface C - H-Bridge Enable Control
const int IF_C_LIMIT = 12;        // D12 - Interface C - Limit Switch
const int IF_C_DIR = 13;          // D13 - Interface C - Direction Control  (D13 cannot be used for input)
const int IF_C_OPT_A = 14;        // D14 - Interface C - Shaft Encoder - A
const int IF_C_OPT_B = 15;        // D15 - Interface C - Shaft Encoder - A

int IF_A__limit_sw_status = 0;
int IF_A_opt_A_status     = 0;
int IF_A_opt_B_status     = 0;

int IF_B__limit_sw_status = 0;
int IF_B_opt_A_status     = 0;
int IF_B_opt_B_status     = 0;

int IF_C__limit_sw_status = 0;
int IF_C_opt_A_status     = 0;
int IF_C_opt_B_status     = 0;

int IF_A_opt_A_previous_status = 0;
int IF_A_opt_B_previous_status = 0;
int IF_B_opt_A_previous_status = 0;
int IF_B_opt_B_previous_status = 0;
int IF_C_opt_A_previous_status = 0;
int IF_C_opt_B_previous_status = 0;

int IF_A_rotation_counter = 0;
int IF_B_rotation_counter = 0;
int IF_C_rotation_counter = 0;

int IF_A_range_full_count = 1300;
int IF_B_range_full_count = 1300;
int IF_C_range_full_count = 1300;

int IF_A_home_direction = 1;     
int IF_B_home_direction = 1;
int IF_C_home_direction = 1;

int IF_A_slow_speed = 150;     
int IF_B_slow_speed = 150;
int IF_C_slow_speed = 150;

int IF_A_max_speed = 100;     
int IF_B_max_speed = 100;
int IF_C_max_speed = 100;

bool IF_A_motor_not_timedout = true;
bool IF_B_motor_not_timedout = true;
bool IF_C_motor_not_timedout = true;

int  IF_A_motor_timeout_milliseconds = 20000;
int  IF_B_motor_timeout_milliseconds = 20000;
int  IF_C_motor_timeout_milliseconds = 20000;


int  IF_X_range_full_count = 0;
int  IF_X_home_direction = 0;
int  IF_X_slow_speed = 0;
int  IF_X_max_speed = 0;
int  IF_X_motor_timeout_milliseconds = 0;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
int analog_input_pin_number_to_control_motor = A7;
int analog_input_to_control_motor = 0;

int command = 32;

bool action_successful = false;
int debug_control = 10;

long previousMillis = 0; 
long interval = 1000; 
unsigned long currentMillis;

int loop_state = 0;

int IF_A_target_count = 0;
int IF_B_target_count = 0;
int IF_C_target_count = 0;

bool IF_A_home_achieved = false;
bool IF_B_home_achieved = false;
bool IF_C_home_achieved = false;

//=================================================================================================
// Communication variables

int data_to_echo = 0;
int received_command = 0;
int command_response = 0;

const int array_size = 33;
int received_data_array[array_size];
int send_data_array[array_size];
int length_of_send_data_array;
int last_command_received = 0;

int IF_A_status = 0;   // Current  status of joint moving to target = Unknown = 0,  In Process = 1, Complete = 2
int IF_B_status = 0;
int IF_C_status = 0;
//int IF_D_status = 0;
//int IF_E_status = 0;
//int IF_F_status = 0;

int IF_A_count_status = 0;
int IF_B_count_status = 0;
int IF_C_count_status = 0;
//int IF_D_count_status = 0;
//int IF_E_count_status = 0;
//int IF_F_count_status = 0;

int IF_A_count_target = 0;
int IF_B_count_target = 0;
int IF_C_count_target = 0;
//int IF_D_count_target = 0;
//int IF_E_count_target = 0;
//int IF_F_count_target = 0;

int IF_A_count_difference = 0;
int IF_B_count_difference = 0;
int IF_C_count_difference = 0;
//int IF_D_count_difference = 0;
//int IF_E_count_difference = 0;
//int IF_F_count_difference = 0;

int IF_A_angle_target = 0;
int IF_B_angle_target = 0;
int IF_C_angle_target = 0;
//int IF_D_angle_target = 0;
//int IF_E_angle_target = 0;
//int IF_F_angle_target = 0;


//=================================================================================================
/*

Ideas for RPI controlling arduinos

RPI Commanding Arduino
Send an index number followed by a one byte command
The arm will respond with command returning the index number, command and a status byte

RPI polling Arduino for status (a one byte command)
The arduino will respond with the status of the last 10 commands

RPI polling Arduino for joint status (a one byte command)
The arduino will respond with the status of the arm joints. The status is the joint counters, joint angle in
degrees and if any outstanding commands are not complete

The RPI can send a command to stop all motors to stop. This is a high priority command


*/


//=================================================================================================

//==[ Rhino Arm Joint Specific Information ]==

//Joint A - Gripper
//home_direction = 0
//Home procedure:  Move forward at home speed to home, Move back 50% to open gripper to full, Move back 100% to close gripper
// Range Count = TBD
//
//Joint B - Wrist Rotation
//No Limit on rotation
//Range count is 1300
//
//Joint C - Wrist Flex
//home_direction = 1
//Move upward to home
////Range count is 1300
//
//Joint D - Elbow
//Move down to home
//home_direction = 0
//Range count is 700
//
//
//Joint E - Shoulder
//Move backward to home
//home_direction = 0
//Range count is 740
//
//
//Joint F - Waist
//Move Clockwise (top down) to home
//home_direction = 0
//Range count is 740


//Parameters to configure for each joint
//
//IF_X_range_full_count
//IF_X_home_direction
//IF_X_slow_speed
//IF_X_max_speed
//IF_C_motor_timeout_milliseconds


//=================================================================================================
void setup() {
  
Wire.begin(SLAVE_ADDRESS);
Wire.onReceive(receiveEvent);
Wire.onRequest(sendDataEvent);

Serial.begin(9600);
Serial.println(F(""));
Serial.println(F(">> Restarting "));

setup_monitor_output();
setup_pin_modes();

// Identify which Robot Arm Joint the Arduino interface is controlling
set_interface_X_parameters_to_Joint('F'); copy_interface_X_parameters_to_interface_('A');
set_interface_X_parameters_to_Joint('F'); copy_interface_X_parameters_to_interface_('B');
set_interface_X_parameters_to_Joint('F'); copy_interface_X_parameters_to_interface_('C');
print_interface_parameters();

// Temporary test commands
command = 30;    // Home
command = 31;    // Out to max range
command = 12; // Out then home
//command = 10;
}


//=================================================================================================
//  Python commands    (Note:  Arduino only handles 3 interfaces )
//    Home One   - 10,      20,      30,      40,      50,      60
//    Set Angle  - 11-xx,   21-xx,   31-xx,   41-xx,   51-xx,   61-xx
//    Set Count  - 12-xxxx, 22-xxxx, 32-xxxx, 42-xxxx, 52-xxxx, 62-xxxx
//
//  NEED TO PREVENT NEW COMMAND FROM STARTING PRIOR COMPLETING CURRENT COMMAND
//=================================================================================================
void loop() {

  switch (command)
  {
  
  case 10:    // Home Joint A   
    command = 0;     // Clear the command so that it does not run again 
        if (debug_control > 2) Serial.println (F("Command 10 - Home Joint A"));
    IF_A_go_to_home();
    break;

  case 11:    // Move Joint A to specific angle  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 13 - Move Joint A to specific angle"));
    break;

  case 12:    // Move Joint A from home out to full range then back to home  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 14 - Move Joint A to specific count"));
    IF_A_move_full_range(IF_A_target_count);  ///  << Limits movement
    break;

//   Need  to  make sure home is achieved before using count  IF_A_home_achieved


  case 13:    // Move Joint A out to full range
    command = 0;
    if (debug_control > 2) Serial.println (F("Command 11 - Move Joint A over full range"));
    IF_A_move_full_range(IF_A_range_full_count);  ///  << Limits movement
    break;

  case 14:    // Move Joint A from home out to full range then back to home  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 12 - Move Joint A from home out to full range then back to home"));
    IF_A_go_to_home();    
    IF_A_move_full_range(IF_A_range_full_count);  ///  << Limits movement
    IF_A_go_to_home();    
    break;

  case 20:    // Home Joint B   
    command = 0;     // Clear the command so that it does not run again 
        if (debug_control > 2) Serial.println(F("Command 20 - Home Joint B"));
    IF_B_go_to_home();
    break;

  case 21:    // Move Joint B out to full range
    command = 0;
    if (debug_control > 2) Serial.println (F("Command 21 - Move Joint B over full range"));
    IF_B_move_full_range(IF_B_range_full_count);  ///  << Limits movement
    break;

  case 22:    // Move Joint B from home out to full range then back to home  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 22 - Move Joint B from home out to full range then back to home"));
    IF_B_go_to_home();    
    IF_B_move_full_range(IF_B_range_full_count);  ///  << Limits movement
    IF_B_go_to_home();    
    break;

  case 23:    // Move Joint C to specific angle  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 23 - Move Joint B to specific angle"));
    break;
 
  case 30:    // Home Joint C   
    command = 0;     // Clear the command so that it does not run again 
        if (debug_control > 2) Serial.println (F("Command 30 - Home Joint C"));
    IF_C_go_to_home();
    break;

  case 31:    // Move Joint C out to full range
    command = 0;
    if (debug_control > 2) Serial.println(F("Command 31 - Move Joint C over full range"));
    IF_C_move_full_range(IF_C_range_full_count);  ///  << Limits movement
    break;

  case 32:    // Move Joint C from home out to full range then back to home  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 32 - Move Joint C from home out to full range then back to home"));
    IF_C_go_to_home();    
    IF_C_move_full_range(IF_C_range_full_count);  ///  << Limits movement
    IF_C_go_to_home();    
    break;

  case 33:    // Move Joint C to specific angle  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 33 - Move Joint C to specific angle"));
    break;

  case 1:    // Stop All Motors
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 0 - STOP ALL MOTORS"));
    analogWrite(IF_A_ENABLE_PWM, 0);
    analogWrite(IF_B_ENABLE_PWM, 0);
    analogWrite(IF_C_ENABLE_PWM, 0);

    break;

  default:
    break;
  }
}

//=================================================================================================
//------------------------------------------
void receiveEvent(int rx_byte_count)    //  Raspberry Pie sending to Arduino 
{

  for (int i = 0; i < array_size; i++) {    // Clear out old data
    received_data_array[i] = 0;
  }

  for (int i = 0; i < rx_byte_count; i++)  {  // Receive the data and place in array
    received_data_array[i] = Wire.read();
  }
  // Get the byte count for the response
  if (rx_byte_count > 1) {
    length_of_send_data_array = received_data_array[rx_byte_count - 1]; //  The last byte is the response size
//    Serial.print(" Exit RX   Response size: ");
//    Serial.println(length_of_send_data_array);
  }

  Serial.print ("R");
  for (int i = 0; i < rx_byte_count; i++)  {
    Serial.print(received_data_array[i]);   //check what you are receiving against an Intel-Hex frame
//    Serial.print ("R");
  }
  Serial.println ("|");

  if (received_data_array[1] != 0) {
    last_command_received = received_data_array[1];
    command = last_command_received;
    Serial.print ("Last CMD: ");
    Serial.println (last_command_received);
  }

  // -----------------------------------------------------------------------
  if (received_data_array[1] == 11) {
    int IF_A_angle_target = received_data_array[2];
    Serial.print ("Ang A: "); Serial.println (IF_A_angle_target);
  };

// -----------------------------------------------------------------------
if (received_data_array[1] == 21) {
  int IF_B_angle_target = received_data_array[2];
//  Serial.print ("Ang B: "); Serial.println (IF_A_angle_target);
};

// -----------------------------------------------------------------------
if (received_data_array[1] == 31) {
  int IF_C_angle_target = received_data_array[2];
//  Serial.print ("Ang C: "); Serial.println (IF_A_angle_target);
};

// -----------------------------------------------------------------------
if (received_data_array[1] == 12) {
  IF_A_count_target = (received_data_array[2] * 256) +  received_data_array[3];
  Serial.print ("IF_A_count_target: "); Serial.println (IF_A_count_target);
}


if (received_data_array[1] == 22) {
  IF_B_count_target = (received_data_array[2] * 256) +  received_data_array[3];
  Serial.print ("IF_B_count_target: "); Serial.println (IF_B_count_target);
}

if (received_data_array[1] == 32) {
  IF_C_count_target = (received_data_array[2] * 256) +  received_data_array[3];
  Serial.print ("IF_C_count_target: "); Serial.println (IF_C_count_target);
}

// -----------------------------------------------------------------------
if (last_command_received == 40) {
  Serial.println ("Req stat");
}

if (received_data_array[1] == 50) {
  //    Serial.println ("Requesting all counts");
}


}

//
//------------------------------------------
void sendDataEvent()
{
  for (int i = 0; i < array_size; i++) {    // Clear out old data
    send_data_array[i] = 0;
  }

  send_data_array[0] = 0;
  send_data_array[1] = 1;
  send_data_array[2] = 2;
  send_data_array[3] = 3;
  send_data_array[4] = 4;
  send_data_array[5] = 5;
  send_data_array[6] = 6;
  //  send_data_array[7] = 7;
  //  send_data_array[8] = 8;
  //  send_data_array[9] = 9;
  //  send_data_array[10] = 10;
  //  send_data_array[11] = 11;
  //  send_data_array[12] = 12;
  //  send_data_array[13] = 13;
  //  send_data_array[14] = 14;
  //  send_data_array[15] = 15;
  //  send_data_array[16] = 16;
  //  send_data_array[17] = 17;
  //  send_data_array[18] = 18;
  //  send_data_array[19] = 19;
  //  send_data_array[20] = 20;

  if (last_command_received == 11) {
    send_data_array[length_of_send_data_array - 1] = IF_A_status;
  }

  if (last_command_received == 12) {
    send_data_array[length_of_send_data_array - 1] = IF_B_status;
  }

  if (last_command_received == 13) {
    send_data_array[length_of_send_data_array - 1] = IF_C_status;
  }



  if (last_command_received == 40) {
    Serial.print(IF_A_status);
    Serial.print(IF_B_status);
    Serial.print(IF_C_status);
    Serial.println("Stat");

    send_data_array[0] = IF_A_status;
    send_data_array[1] = IF_B_status;
    send_data_array[2] = IF_C_status;

  }

  if (last_command_received == 50) {
    send_data_array[0] = (IF_A_count_status >> 8) & 0xff;  //  IF_A_count_status_Low_byte
    send_data_array[1] = IF_A_count_status % 256;          //  IF_A_count_status_low_byte

    send_data_array[2] = (IF_B_count_status >> 8) & 0xff;  //  IF_A_count_status_Low_byte
    send_data_array[3] = IF_B_count_status % 256;   //  IF_A_count_status_low_byte

    send_data_array[4] = (IF_C_count_status >> 8) & 0xff;  //  IF_A_count_status_Low_byte
    send_data_array[5] = IF_C_count_status % 256;   //  IF_A_count_status_low_byte
  }

  for (int i = 0; i < length_of_send_data_array; i++)
  {
    Wire.write(send_data_array[i]);
  }

}  // end of Send Data Event
//------------------------------------------

//=================================================================================================
//=================================================================================================
//  IF_A_home_achieved
//=================================================================================================
bool IF_A_go_to_home(){
    if (debug_control > 2) Serial.println(F("IF_A_go_to_home ")); 
    
    action_successful = IF_A_drive_motor(IF_A_home_direction, IF_A_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    bool limit_switch_status = limit_switch_triggered(IF_A_LIMIT);   // Returns True when limit hit
        
    while (! limit_switch_status && IF_A_motor_not_timedout ){
      IF_A_monitor_encoder(-1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_A_motor_timeout_milliseconds) 
        {
        IF_A_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch A not triggered before timeout"));
        }
        
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) { Serial.print(F("Encoder A: ")); Serial.println(IF_A_rotation_counter);}
        }

      limit_switch_status = limit_switch_triggered(IF_A_LIMIT);

      if (limit_switch_status){
          IF_A_home_achieved = true;
      }
        
      if (debug_control > 2) 
        if (limit_switch_status) Serial.println(F("Homing Limit Switch A Detected"));
    }

  action_successful = IF_A_drive_motor(IF_A_home_direction, 0);  // Stop the motor
  
  if (debug_control > 2) { Serial.print(F("IF_A_rotation_counter: "));  Serial.println(IF_A_rotation_counter);}
  IF_A_rotation_counter = 0;
  if (debug_control > 2) Serial.print(F("IF_A_rotation_counter reset to zero: "));
  return IF_A_home_achieved;
}

//=================================================================================================
void IF_B_go_to_home(){
    if (debug_control > 2) Serial.println(F("IF_B_go_to_home ")); 
    
    action_successful = IF_B_drive_motor(  IF_B_home_direction, IF_B_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    bool limit_switch_status = limit_switch_triggered(IF_B_LIMIT);   // Returns True when limit hit
        
    while (! limit_switch_status && IF_B_motor_not_timedout ){
      IF_B_monitor_encoder(-1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_B_motor_timeout_milliseconds) 
        {
        IF_B_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch B not triggered before timeout"));
        }
        
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) { Serial.print(F("Encoder B: ")); Serial.println(IF_B_rotation_counter);}
        }

      if (limit_switch_status){
          IF_B_home_achieved = true;
      }
        
      limit_switch_status = limit_switch_triggered(IF_B_LIMIT);
      if (debug_control > 2) 
        if (limit_switch_status) Serial.println(F("Homing Limit Switch B Detected"));
    }

  action_successful = IF_B_drive_motor(IF_B_home_direction, 0);  // Stop the motor
  
  if (debug_control > 2) { Serial.print(F("IF_B_rotation_counter: "));  Serial.println(IF_B_rotation_counter);}
  IF_B_rotation_counter = 0;
  if (debug_control > 2) Serial.print(F("IF_B_rotation_counter reset to zero "));
}

//=================================================================================================
void IF_C_go_to_home(){
    if (debug_control > 2) Serial.println(F("IF_C_go_to_home ")); 
    
    action_successful = IF_C_drive_motor(  IF_C_home_direction, IF_C_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    bool limit_switch_status = limit_switch_triggered(IF_C_LIMIT);   // Returns True when limit hit
        
    while (! limit_switch_status && IF_C_motor_not_timedout ){
      IF_C_monitor_encoder(-1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_C_motor_timeout_milliseconds) 
        {
        IF_C_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch C not triggered before timeout"));
        }
        
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) { Serial.print(F("Encoder C: ")); Serial.println(IF_C_rotation_counter);}
        }

      if (limit_switch_status){
          IF_C_home_achieved = true;
      }
        
      limit_switch_status = limit_switch_triggered(IF_C_LIMIT);
      if (debug_control > 2) 
        if (limit_switch_status) Serial.println(F("Homing Limit Switch C Detected"));
    }

  action_successful = IF_C_drive_motor(IF_C_home_direction, 0);  // Stop the motor
  
  if (debug_control > 2) { Serial.print(F("IF_C_rotation_counter: "));  Serial.println(IF_C_rotation_counter);}
  IF_C_rotation_counter = 0;
  if (debug_control > 2) Serial.print(F("IF_C_rotation_counter reset to zero "));
}

//=================================================================================================

bool IF_A_drive_motor(int _direction, int _speed){

  digitalWrite(IF_A_DIR,   _direction);  // 
  analogWrite(IF_A_ENABLE_PWM, _speed); // where dutyCycle is a value from 0 to 255
  if (debug_control > 2) {
    Serial.print(F("Motor A dir: ")); 
    Serial.print(_direction);
    Serial.print(F("  speed: "));
    Serial.println(_speed);  
  }
  return true;
}
//=================================================================================================

bool IF_B_drive_motor(int _direction, int _speed){

  digitalWrite(IF_B_DIR,   _direction);  // 
  analogWrite(IF_B_ENABLE_PWM, _speed); // where dutyCycle is a value from 0 to 255
  if (debug_control > 2) {
    Serial.print(F("Motor B dir: ")); 
    Serial.print(_direction);
    Serial.print(F("  speed: "));
    Serial.println(_speed);  
  }
  return true;
}
//=================================================================================================

bool IF_C_drive_motor(int _direction, int _speed){

  digitalWrite(IF_C_DIR,   _direction);  // 
  analogWrite(IF_C_ENABLE_PWM, _speed); // where dutyCycle is a value from 0 to 255
  if (debug_control > 2) {
    Serial.print(F("Motor dir C: ")); 
    Serial.print(_direction);
    Serial.print(F("  speed: "));
    Serial.println(_speed);  
  }
  return true;
}
//=================================================================================================

bool limit_switch_triggered(int _pin){
  int return_value = 1;                     // High [1] when switch is not selected
  int return_value_1 = digitalRead(_pin);   // Seem to be getting false LOW Signals to trying to combine 5 readings
  int return_value_2 = digitalRead(_pin);   // All three readings must be low to get a limit switch trip 
  int return_value_3 = digitalRead(_pin);
  int return_value_4 = digitalRead(_pin);
  int return_value_5 = digitalRead(_pin);
  return_value = return_value_1 || return_value_2 || return_value_3 || return_value_4 || return_value_5;
//  Serial.println (return_value);
  return ! return_value;  // Invert the sense of the pin
}

//=================================================================================================
//  Keep this routine short to reduce the chance  of missing an edge transition
//  This function updates the  interface counter:  IF_X_rotation_counter

void IF_A_monitor_encoder(int increment_direction){
  
  IF_A_opt_A_status = digitalRead(IF_A_OPT_A);
  if (IF_A_opt_A_previous_status != IF_A_opt_A_status){
    if (IF_A_opt_A_status == 1){
      IF_A_rotation_counter = IF_A_rotation_counter + increment_direction;
    }
  }
  IF_A_opt_A_previous_status = IF_A_opt_A_status;
}

//=================================================================================================
void IF_B_monitor_encoder(int increment_direction){
  
  IF_B_opt_A_status = digitalRead(IF_B_OPT_A);
  if (IF_B_opt_A_previous_status != IF_B_opt_A_status){
    if (IF_B_opt_A_status == 1){
      IF_B_rotation_counter = IF_B_rotation_counter + increment_direction;
    }
  }
  IF_B_opt_A_previous_status = IF_B_opt_A_status;
}

//=================================================================================================
void IF_C_monitor_encoder(int increment_direction){
  
  IF_C_opt_A_status = digitalRead(IF_C_OPT_A);
  if (IF_C_opt_A_previous_status != IF_C_opt_A_status){
    if (IF_C_opt_A_status == 1){
      IF_C_rotation_counter = IF_C_rotation_counter + increment_direction;
    }
  }
  IF_C_opt_A_previous_status = IF_C_opt_A_status;
}

//=================================================================================================
bool IF_A_move_to_target_Count(int target_count ){
    bool command_successful = false;

    // Add some range checking of the target value here

    // Log  the command and target  count
    if (debug_control > 2) {
      Serial.print(F("IF_A moving to target count: ")); 
      Serial.println (target_count);
    }

    // Verify that the joint has been "homed" to make sure counter is  available
    
    if (!IF_A_home_achieved){
      Serial.println(F("Fail: Cannot move to target count because IF_A Not Homed"));
      return false;
    }
  
    // Determine the direction to move based on current position and target position
    // Moving toward home makes the position counter get smaller (negative)
    // Moving away from home (larger number) is positive
    
    int delta_count = target_count - IF_A_rotation_counter;  // Target - current count

    // Need to take into consideration that joints have different home directions
    // Home direction (IF_A_home_direction) is either 1 or 0 
    
    int  motion_direction = 0;  //  1 = moving outward , -1 = toward home 
    if (delta_count > 0){
        Serial.print (F("Joint to move away from home"));
        if (IF_A_home_direction == 1){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
      } else{
        Serial.print (F("Joint to move toward home"));
        if (IF_A_home_direction == 0){
          motion_direction = 1;
        } else {
          motion_direction = 0;}
    }

    // Start the motor moving
    action_successful = IF_A_drive_motor( motion_direction, IF_A_slow_speed);   // 

    // Monitor for achieving target count, WHILE monitoring timeout or hitting the home limit (unexpectedly)

    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
  
    unsigned long startMilliseconds = millis();      // Timers for timeout monitoring
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_A_motor_not_timedout ){
      IF_A_monitor_encoder(1);  // Increment the counter while going away from home.

      if ( IF_A_rotation_counter >= IF_A_range_full_count) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder A count limit hit"));
      }
      
//      encoder_limit_not_hit = !encoder_limit_hit;   // ?? WHY not set to T or F
      encoder_limit_not_hit = false;   // ?? WHY not set to T or F
            
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_A_motor_timeout_milliseconds) 
        {
        IF_A_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch A not triggered before timeout"));
        }

      // Print the current encoder count every 500 milliseconds  
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) {Serial.print(F("Encoder A: ")); Serial.println(IF_A_rotation_counter); }
        }
    }

  //  Stop the motor
  action_successful = IF_A_drive_motor( ! IF_A_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_A_rotation_counter: "));  Serial.println(IF_A_rotation_counter); }

  command_successful = encoder_limit_not_hit && IF_A_motor_not_timedout;

  return command_successful;
}
//=================================================================================================
//=================================================================================================
void IF_A_move_full_range(int encoder_counts_to_move ){
    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
  
    if (debug_control > 2) Serial.println(F("IF_A_move_full_range ")); 
    
    action_successful = IF_A_drive_motor( ! IF_A_home_direction, IF_A_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_A_motor_not_timedout ){
      IF_A_monitor_encoder(1);  // Increment the counter while going away from home.

      if ( IF_A_rotation_counter >= encoder_counts_to_move) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder A count limit hit"));
      }
      encoder_limit_not_hit = !encoder_limit_hit;
      
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_A_motor_timeout_milliseconds) 
        {
        IF_A_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch A not triggered before timeout"));
        }
        
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) {Serial.print(F("Encoder A: ")); Serial.println(IF_A_rotation_counter); }
        }
    }

  action_successful = IF_A_drive_motor( ! IF_A_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_A_rotation_counter: "));  Serial.println(IF_A_rotation_counter); }
  
}
//=================================================================================================
void IF_B_move_full_range(int encoder_counts_to_move ){
    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
  
    if (debug_control > 2) Serial.println(F("IF_B_move_full_range ")); 
    
    action_successful = IF_B_drive_motor( ! IF_B_home_direction, IF_B_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_B_motor_not_timedout ){
      IF_B_monitor_encoder(1);  // Increment the counter while going away from home.

      if ( IF_B_rotation_counter >= encoder_counts_to_move) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder B count limit hit"));
      }
      encoder_limit_not_hit = !encoder_limit_hit;
      
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_B_motor_timeout_milliseconds) 
        {
        IF_B_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch B not triggered before timeout"));
        }
        
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) {Serial.print(F("Encoder B: ")); Serial.println(IF_B_rotation_counter); }
        }
    }

  action_successful = IF_B_drive_motor( ! IF_B_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_B_rotation_counter: "));  Serial.println(IF_B_rotation_counter); }
  
}
//=================================================================================================
void IF_C_move_full_range(int encoder_counts_to_move ){
    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
  
    if (debug_control > 2) Serial.println(F("IF_C_move_full_range ")); 
    
    action_successful = IF_C_drive_motor( ! IF_C_home_direction, IF_C_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_C_motor_not_timedout ){
      IF_C_monitor_encoder(1);  // Increment the counter while going away from home.

      if ( IF_C_rotation_counter >= encoder_counts_to_move) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder count limit hit"));
      }
      encoder_limit_not_hit = !encoder_limit_hit;
      
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_C_motor_timeout_milliseconds) 
        {
        IF_C_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch not triggered before timeout"));
        }
        
      if (currentMillis - previousMillidisplay > 500) 
        {
          previousMillidisplay = currentMillis;
          if (debug_control > 2) {Serial.print(F("Encoder: ")); Serial.println(IF_C_rotation_counter); }
        }
    }

  action_successful = IF_C_drive_motor( ! IF_C_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_C_rotation_counter: "));  Serial.println(IF_C_rotation_counter); }
  
}

//=================================================================================================
void set_interface_X_parameters_to_Joint(char joint){

//Parameters to configure for each joint
//
  if (debug_control > 2) {
    Serial.println(F("---------------------------------------------"));
    Serial.print (F("Joint: "));  Serial.println(joint);
    }
    
  switch (joint)
  {
  case 'A':    // Joint C   
    IF_X_range_full_count           = 80;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 80;    // PWM value
    IF_X_max_speed                  = 200;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    break;

  case 'B':    // Joint C   
    IF_X_range_full_count           = 880;   // Counts
    IF_X_home_direction             = 1;      //
    IF_X_slow_speed                 = 200;    // PWM value
    IF_X_max_speed                  = 200;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    break;
  
  case 'C':    // Joint C   
    IF_X_range_full_count           = 1300;   // Counts
    IF_X_home_direction             = 1;      //
    IF_X_slow_speed                 = 150;    // PWM value
    IF_X_max_speed                  = 200;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    break;

  case 'D':    // Joint C   
    IF_X_range_full_count           = 700;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 150;    // PWM value
    IF_X_max_speed                  = 200;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    break;
    
  case 'E':    // Joint C   
    IF_X_range_full_count           = 740;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 150;    // PWM value
    IF_X_max_speed                  = 200;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    break;
    
  case 'F':    // Joint C   
    IF_X_range_full_count           = 720;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 200;    // PWM value
    IF_X_max_speed                  = 200;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    break;
  }

 
}

//=================================================================================================
void copy_interface_X_parameters_to_interface_(char interface){

  if (debug_control > 2) {Serial.print(F("Interface: "));  Serial.println(interface);}

  switch (interface)
  {
  case 'A':    // Joint A   
    IF_A_range_full_count           = IF_X_range_full_count;
    IF_A_home_direction             = IF_X_home_direction;             
    IF_A_slow_speed                 = IF_X_slow_speed;                 
    IF_A_max_speed                  = IF_X_max_speed;                  
    IF_A_motor_timeout_milliseconds = IF_A_motor_timeout_milliseconds; 
    break;

  case 'B':    // Joint B   
    IF_B_range_full_count           = IF_X_range_full_count;
    IF_B_home_direction             = IF_X_home_direction;             
    IF_B_slow_speed                 = IF_X_slow_speed;                 
    IF_B_max_speed                  = IF_X_max_speed;                  
    IF_B_motor_timeout_milliseconds = IF_A_motor_timeout_milliseconds; 
    break;
  
  case 'C':    // Joint C   
    IF_C_range_full_count           = IF_X_range_full_count;
    IF_C_home_direction             = IF_X_home_direction;             
    IF_C_slow_speed                 = IF_X_slow_speed;                 
    IF_C_max_speed                  = IF_X_max_speed;                  
    IF_C_motor_timeout_milliseconds = IF_A_motor_timeout_milliseconds; 
    break;
  }
}

//=================================================================================================
//=================================================================================================
void print_interface_parameters(){

if (debug_control > 2) {
    Serial.println(F(" "));
    Serial.print(F("IF_A_range_full_count: "));           Serial.println(IF_A_range_full_count);
    Serial.print(F("IF_A_home_direction: "));             Serial.println(IF_A_home_direction);
    Serial.print(F("IF_A_slow_speed: "));                 Serial.println(IF_A_slow_speed);
    Serial.print(F("IF_A_max_speed: "));                  Serial.println(IF_A_max_speed);
    Serial.print(F("IF_A_motor_timeout_milliseconds: ")); Serial.println(IF_A_motor_timeout_milliseconds);
    Serial.println(F(" "));

    Serial.print(F("IF_B_range_full_count: "));           Serial.println(IF_B_range_full_count);
    Serial.print(F("IF_B_home_direction: "));             Serial.println(IF_B_home_direction);
    Serial.print(F("IF_B_slow_speed: "));                 Serial.println(IF_B_slow_speed);
    Serial.print(F("IF_B_max_speed: "));                  Serial.println(IF_B_max_speed);
    Serial.print(F("IF_B_motor_timeout_milliseconds: ")); Serial.println(IF_B_motor_timeout_milliseconds);
    
    Serial.println(F(" "));
    Serial.print(F("IF_C_range_full_count: "));           Serial.println(IF_C_range_full_count);
    Serial.print(F("IF_C_home_direction: "));             Serial.println(IF_C_home_direction);
    Serial.print(F("IF_C_slow_speed: "));                 Serial.println(IF_C_slow_speed);
    Serial.print(F("IF_C_max_speed: "));                  Serial.println(IF_C_max_speed);
    Serial.print(F("IF_C_motor_timeout_milliseconds: ")); Serial.println(IF_C_motor_timeout_milliseconds);
    Serial.println(F(" "));
  }
}

//==(Functions)=======================================================
void initialize_output_variables(){
  
}

//=================================================================================================

void setup_monitor_output()
{
  Serial.begin(9600);
  Serial.println(F("")); 
  Serial.println(F(">> Restarting ")); 
}
//=================================================================================================
void setup_pin_modes(){
  
  pinMode(IF_A_DIR, OUTPUT);
  pinMode(IF_A_ENABLE_PWM, OUTPUT);
  pinMode(IF_A_LIMIT, INPUT_PULLUP);
  pinMode(IF_A_OPT_A, INPUT);
  pinMode(IF_A_OPT_B, INPUT);
  
  pinMode(IF_B_OPT_A, INPUT);
  pinMode(IF_B_OPT_B, INPUT);
  pinMode(IF_B_ENABLE_PWM, OUTPUT);
  pinMode(IF_B_DIR, OUTPUT);
  pinMode(IF_B_LIMIT, INPUT_PULLUP);
  
  pinMode(IF_C_ENABLE_PWM, OUTPUT);
  pinMode(IF_C_DIR, OUTPUT);
  pinMode(IF_C_LIMIT, INPUT_PULLUP);
  pinMode(IF_C_OPT_A, INPUT);
  pinMode(IF_C_OPT_B, INPUT);

  pinMode(analog_input_pin_number_to_control_motor, INPUT);
}

//=================================================================================================
void read_digital_inputs(){

//  Save of previous pin state
  IF_A_opt_A_previous_status = IF_A_opt_A_status;
  IF_A_opt_B_previous_status = IF_A_opt_B_status;
  IF_B_opt_A_previous_status = IF_B_opt_A_status;
  IF_B_opt_B_previous_status = IF_B_opt_B_status;
  IF_C_opt_A_previous_status = IF_C_opt_A_status;
  IF_C_opt_B_previous_status = IF_C_opt_B_status;

  
  IF_A__limit_sw_status = digitalRead(IF_A_LIMIT);
  IF_A_opt_A_status     = digitalRead(IF_A_OPT_A);
  IF_A_opt_B_status     = digitalRead(IF_A_OPT_B);

  IF_B__limit_sw_status = digitalRead(IF_B_LIMIT);
  IF_B_opt_A_status     = digitalRead(IF_B_OPT_A);
  IF_B_opt_B_status     = digitalRead(IF_B_OPT_B);

  IF_C__limit_sw_status = digitalRead(IF_C_LIMIT);
  IF_C_opt_A_status     = digitalRead(IF_C_OPT_A);
  IF_C_opt_B_status     = digitalRead(IF_C_OPT_B);

  if (IF_C_opt_A_previous_status != IF_C_opt_A_status){
    if (IF_C_opt_A_status == 1){
      IF_C_rotation_counter = IF_C_rotation_counter +1;
    }
  }
}
//
//=================================================================================================
//=================================================================================================