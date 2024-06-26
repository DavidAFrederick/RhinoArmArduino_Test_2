#include <Wire.h>
//#d e f ine SLAVE_ADDRESS 0x08
//############################################

// Dec 27, 2023   
// On Git      || Near line 212 for interface to joint definition


//=================================================================================================
// Communication Contants
int slave_address = 8;

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

int IF_A_slow_speed = 150;     ///  Need to find out if smaller is faster or slower??
int IF_B_slow_speed = 150;
int IF_C_slow_speed = 150;

int IF_A_max_speed = 100;     
int IF_B_max_speed = 100;
int IF_C_max_speed = 60;   // Line 1709

bool IF_A_motor_not_timedout = true;
bool IF_B_motor_not_timedout = true;
bool IF_C_motor_not_timedout = true;

int  IF_A_motor_timeout_milliseconds = 10000;
int  IF_B_motor_timeout_milliseconds = 10000;
int  IF_C_motor_timeout_milliseconds = 10000;

int  IF_A_JointMovementThreshold = 10;
int  IF_B_JointMovementThreshold = 10;
int  IF_C_JointMovementThreshold = 10;
int  IF_D_JointMovementThreshold = 10;
int  IF_E_JointMovementThreshold = 10;
int  IF_F_JointMovementThreshold = 10;


int  IF_A_typical_encoder_cnt_in_sec = 0;
int  IF_B_typical_encoder_cnt_in_sec = 0;
int  IF_C_typical_encoder_cnt_in_sec = 0;


int  IF_X_range_full_count = 0;
int  IF_X_home_direction = 0;
int  IF_X_slow_speed = 0;
int  IF_X_max_speed = 0;
int  IF_X_motor_timeout_milliseconds = 0;
int  IF_X_typical_encoder_cnt_in_sec = 0;
int  IF_X_JointMovementThreshold = 0;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
int slave_address_sensor_pin = A3;       // High = Address = 8, low = Address = 9
int slave_address_sensor = 0;

int command = 0;
bool last_command_complete = false;
bool action_successful = false;
int debug_control = 10;  // Was 10

long previousMillis = 0; 
long interval = 1000; 
unsigned long currentMillis;

int loop_state = 0;

int starting_encoder_count = 0;
int ending_encoder_count = 0;
int typical_encoder_speed = 0;
int IF_A_delta_encoder_Count = 0;
int IF_B_delta_encoder_Count = 0;
int IF_C_delta_encoder_Count = 0;



//int IF_A_target_count = 0;
//int IF_B_target_count = 0;
//int IF_C_target_count = 0;

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

int IF_A_count_target = 0;
int IF_B_count_target = 0;
int IF_C_count_target = 0;

int IF_A_count_difference = 0;
int IF_B_count_difference = 0;
int IF_C_count_difference = 0;

int IF_A_angle_target = 0;
int IF_B_angle_target = 0;
int IF_C_angle_target = 0;

unsigned long startMilliseconds = 0;

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
setup_monitor_output();
setup_pin_modes();

slave_address_sensor = digitalRead(slave_address_sensor_pin);
if (slave_address_sensor) slave_address = 8;
if (!slave_address_sensor) slave_address = 9;
  
Wire.begin(slave_address);
Wire.onReceive(receiveEvent);
Wire.onRequest(sendDataEvent);

Serial.begin(9600);
Serial.println(F(""));
Serial.println(F(">> Restarting "));

Serial.print(F(">> slave_address_sensor: "));
Serial.println(slave_address_sensor);

Serial.print(F(">> slave_address: "));
Serial.println(slave_address);


// Identify which Robot Arm Joint the Arduino interface is controlling

if (slave_address == 8){   // Arduino #1
  set_interface_X_parameters_to_Joint('A'); copy_interface_X_parameters_to_interface_('A');
  set_interface_X_parameters_to_Joint('B'); copy_interface_X_parameters_to_interface_('B');
  set_interface_X_parameters_to_Joint('C'); copy_interface_X_parameters_to_interface_('C');
  }

if (slave_address == 9){   // Arduino #2
  set_interface_X_parameters_to_Joint('D'); copy_interface_X_parameters_to_interface_('A');
  set_interface_X_parameters_to_Joint('E'); copy_interface_X_parameters_to_interface_('B');
  set_interface_X_parameters_to_Joint('F'); copy_interface_X_parameters_to_interface_('C');
  }
print_interface_parameters();

//Joint A - Gripper
//Joint B - Wrist Rotation
//Joint C - Wrist Flex
//Joint D - Elbow
//Joint E - Shoulder
//Joint F - Waist


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

  case 1:    // Stop All Motors
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 0 - STOP ALL MOTORS"));
    analogWrite(IF_A_ENABLE_PWM, 0);
    analogWrite(IF_B_ENABLE_PWM, 0);
    analogWrite(IF_C_ENABLE_PWM, 0);
    break;

  case 10:    // Home Joint A   
    command = 0;     // Clear the command so that it does not run again 
//    last_command_complete = false;
        if (debug_control > 2) Serial.println (F("Command 10 - Home Joint A"));
    IF_A_go_to_home();
    break;

  case 11:    // Move Joint A to specific angle  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 13 - Move Joint A to specific angle"));
    break;

  case 12:    // Move Joint A from to a specific count
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 12 - Move Joint A to specific count"));
    IF_A_move_to_target_Count(IF_A_count_target);  ///  << Limits movement
    break;

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

// - IF-B
  case 20:    // Home Joint B   
    command = 0;     // Clear the command so that it does not run again 
        if (debug_control > 2) Serial.println (F("Command 20 - Home Joint B"));
    IF_B_go_to_home();
    break;

  case 21:    // Move Joint B to specific angle  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 21 - Move Joint B to specific angle"));
    break;

  case 22:    // Move Joint B from to a specific count
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 22 - Move Joint B to specific count"));
    IF_B_move_to_target_Count(IF_B_count_target);  ///  << Limits movement
    break;

  case 23:    // Move Joint B out to full range
    command = 0;
    if (debug_control > 2) Serial.println (F("Command 23 - Move Joint B over full range"));
    IF_B_move_full_range(IF_B_range_full_count);  ///  << Limits movement
    break;

  case 24:    // Move Joint B from home out to full range then back to home  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 24 - Move Joint B from home out to full range then back to home"));
    IF_B_go_to_home();    
    IF_B_move_full_range(IF_B_range_full_count);  ///  << Limits movement
    IF_B_go_to_home();    
    break;


// - IF-C
  case 30:    // Home Joint C   
    command = 0;     // Clear the command so that it does not run again 
        if (debug_control > 2) Serial.println (F("Command 30 - Home Joint C"));
    IF_C_go_to_home();
    break;

  case 31:    // Move Joint C to specific angle  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 31 - Move Joint C to specific angle"));
    break;

  case 32:    // Move Joint C from to a specific count
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 32 - Move Joint C to specific count"));
    IF_C_move_to_target_Count(IF_C_count_target);  ///  << Limits movement
    break;

  case 33:    // Move Joint C out to full range
    command = 0;
    if (debug_control > 2) Serial.println (F("Command 33 - Move Joint C over full range"));
    IF_C_move_full_range(IF_C_range_full_count);  ///  << Limits movement
    break;

  case 34:    // Move Joint C from home out to full range then back to home  
    command = 0; 
    if (debug_control > 2) Serial.println(F("Command 34 - Move Joint C from home out to full range then back to home"));
    IF_C_go_to_home();    
    IF_C_move_full_range(IF_C_range_full_count);  ///  << Limits movement
    IF_C_go_to_home();    
    break;

  case 90:  // Get Status flags
    command = 0;
    if (debug_control > 200) Serial.println(F("Cmd 90-Gt stat"));
    break;
    
  case 91:  // Get Interface Encoder Counts
    command = 0;
    if (debug_control > 200) Serial.println(F("Cmd 91-Gt Cnts"));
    break;

  case 15: 
    Serial.println (F(">> Special - Move interface A away from home then to home"));
    IF_A_go_away_from_home_until_limited();
    break;

  case 25: 
    Serial.println (F(">> Special - Move interface B away from home then to home"));
    IF_B_go_away_from_home_until_limited();
    break;

  case 35: 
    Serial.println (F(">> Special - Move interface C away from home then to home"));
    IF_C_go_away_from_home_until_limited();
    break;

  case 18: 
    command = 0;
    Serial.println (F(">> Special - Move interface A toward home for 1 second"));
    IF_A_Move_toward_home_for_one_Second();
    break;
  case 28: 
    command = 0;
    Serial.println (F(">> Special - Move interface B toward home for 1 second"));
    IF_B_Move_toward_home_for_one_Second();
    break;

  case 38: 
    command = 0;
    Serial.println (F(">> Special - Move interface C toward home for 1 second"));
    IF_C_Move_toward_home_for_one_Second();
    break;

 
  case 19: 
    command = 0;
    Serial.println (F(">> Special - Move interface A away from home for 1 second"));
    startMilliseconds = millis();
    IF_A_motor_not_timedout = true;  
    
    starting_encoder_count = IF_A_rotation_counter;
    while (IF_A_motor_not_timedout ){
      action_successful = IF_A_drive_motor(! IF_A_home_direction, IF_A_slow_speed);  // Stop the motor
      IF_A_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > 1000) 
        {
        IF_A_motor_not_timedout = false;   
        }
    }
    action_successful = IF_A_drive_motor(IF_A_home_direction, 0);  // Stop the motor

    ending_encoder_count = IF_A_rotation_counter;
    IF_A_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("Interface A - Counts in 1 second: ");
    Serial.println (IF_A_delta_encoder_Count);
    break;

  case 29: 
    command = 0;
    Serial.println (">> Special - Move interface B away from home for 1 second");
    startMilliseconds = millis();
    IF_B_motor_not_timedout = true;  
    
    starting_encoder_count = IF_B_rotation_counter;
    while (IF_B_motor_not_timedout ){
      action_successful = IF_B_drive_motor(! IF_B_home_direction, IF_B_slow_speed);  // Start the motor
      IF_B_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > 1000) //TEMP
        {
        IF_B_motor_not_timedout = false;   
        }
    }
    action_successful = IF_B_drive_motor(IF_B_home_direction, 0);  // Stop the motor

    ending_encoder_count = IF_B_rotation_counter;
    IF_B_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("Interface B - Counts in 1 second: ");
    Serial.println (IF_B_delta_encoder_Count);
    break;


  case 39: 
    command = 0;
    Serial.println (">> Special - Move interface C away from home for 1 second");
    startMilliseconds = millis();
    IF_C_motor_not_timedout = true;  
    
    starting_encoder_count = IF_C_rotation_counter;
    while (IF_C_motor_not_timedout ){
      action_successful = IF_C_drive_motor(! IF_C_home_direction, IF_C_slow_speed);  // Star the motor
      IF_C_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > 1000) //TEMP 
        {
        IF_C_motor_not_timedout = false;   
        }
    }
    action_successful = IF_C_drive_motor(IF_C_home_direction, 0);  // Stop the motor

    ending_encoder_count = IF_C_rotation_counter;
    IF_C_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("Interface C - Counts in 1 second: ");
    Serial.println (IF_C_delta_encoder_Count);
    break;

//  case 1:    // Stop All Motors
//    command = 0; 
//    if (debug_control > 2) Serial.println(F("Command 0 - STOP ALL MOTORS"));
//    analogWrite(IF_A_ENABLE_PWM, 0);
//    analogWrite(IF_B_ENABLE_PWM, 0);
//    analogWrite(IF_C_ENABLE_PWM, 0);
//    break;

  default:
    break;
  }  // Case Statement
}

//=================================================================================================

void IF_A_Move_toward_home_for_one_Second(){
    startMilliseconds = millis();
    IF_A_motor_not_timedout = true;  

    starting_encoder_count = IF_A_rotation_counter;
    
    while (IF_A_motor_not_timedout ){
      action_successful = IF_A_drive_motor(IF_A_home_direction, IF_A_slow_speed);  // Stop the motor
      IF_A_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > 1000) 
        {
        IF_A_motor_not_timedout = false;   
        }
    }
    action_successful = IF_A_drive_motor(IF_A_home_direction, 0);  // Stop the motor
    ending_encoder_count = IF_A_rotation_counter;
    IF_A_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("Interface A - Counts in 1 second: ");
    Serial.println (IF_A_delta_encoder_Count);
}
//=========================================================

void IF_B_Move_toward_home_for_one_Second(){
    startMilliseconds = millis();
    IF_B_motor_not_timedout = true;  
    
    starting_encoder_count = IF_B_rotation_counter;
    while (IF_B_motor_not_timedout ){
      action_successful = IF_B_drive_motor( IF_B_home_direction, IF_B_slow_speed);  // Start the motor
      IF_B_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      if (currentMillis - startMilliseconds > 1000)  //TEMP
        {
        IF_B_motor_not_timedout = false;   
        }
    }
    action_successful = IF_B_drive_motor(IF_B_home_direction, 0);  // Stop the motor
    
    ending_encoder_count = IF_B_rotation_counter;
    IF_B_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("Interface B - Counts in 1 second: ");
    Serial.println (IF_B_delta_encoder_Count);
}
//=========================================================

void     IF_C_Move_toward_home_for_one_Second(){
      startMilliseconds = millis();
      IF_C_motor_not_timedout = true;  
      
      starting_encoder_count = IF_C_rotation_counter;
      while (IF_C_motor_not_timedout ){
        action_successful = IF_C_drive_motor( IF_C_home_direction, IF_C_slow_speed);  // Star the motor
        IF_C_monitor_encoder(1);  // Decrement the counter while homing
        currentMillis = millis();
        if (currentMillis - startMilliseconds > 1000) //TEMP
          {
          IF_C_motor_not_timedout = false;   
          }
      }
      action_successful = IF_C_drive_motor(IF_C_home_direction, 0);  // Stop the motor
  
      ending_encoder_count = IF_C_rotation_counter;
      IF_C_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
      Serial.print ("Interface C - Counts in 1 second: ");
      Serial.println (IF_C_delta_encoder_Count);
  }
 
//=================================================================================================

//------------------------------------------

//=================================================================================================

void receiveEvent(int rx_byte_count)    //  Raspberry Pi sending to Arduino 
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
    }

  if (debug_control > 200){
    Serial.print ("R");
    for (int i = 0; i < rx_byte_count; i++)  {
      Serial.print(received_data_array[i]);   //check what you are receiving 
    }
    Serial.println ("|");
    }
  

  if (received_data_array[1] != 0) {
    command = received_data_array[1];

    if (slave_address == 9) {
      if ((command >=40) && (command <= 70)){
        command = command-30;  
      }
      if (debug_control > 100) {Serial.print ("  Last CMD: ");  Serial.println (command);}
    }

    last_command_received = command;  // Used to determine what data to send back in the send data
    
//    Serial.print ("slave_address: ");
//    Serial.print (slave_address);
//    Serial.print ("    Command: ");
//    Serial.println (command);
  }

  // -----------------------------------------------------------------------
  if (command == 11) {
    int IF_A_angle_target = received_data_array[2];
    Serial.print ("Ang A: "); Serial.println (IF_A_angle_target);
  };

// -----------------------------------------------------------------------
  if (command == 21) {
    int IF_B_angle_target = received_data_array[2];
    //  Serial.print ("Ang B: "); Serial.println (IF_A_angle_target);
  };

// -----------------------------------------------------------------------
  if (command == 31) {
    int IF_C_angle_target = received_data_array[2];
    //  Serial.print ("Ang C: "); Serial.println (IF_A_angle_target);
  };

// -----------------------------------------------------------------------
  if (command == 12) {
    IF_A_count_target = (received_data_array[2] * 256) +  received_data_array[3];
    Serial.print ("IF_A_count_target: "); Serial.println (IF_A_count_target);
    };

  if (command == 22) {
    IF_B_count_target = (received_data_array[2] * 256) +  received_data_array[3];
    Serial.print ("IF_B_count_target: "); Serial.println (IF_B_count_target);
    };

  if (command == 32) {
    IF_C_count_target = (received_data_array[2] * 256) +  received_data_array[3];
    Serial.print ("IF_C_count_target: "); Serial.println (IF_C_count_target);
  }
  
  
       

// -----------------------------------------------------------------------
  if (command == 90) {
//    Serial.println ("Rec cmd 90 stat");
  }

  if (command == 91) {
//    Serial.println ("Rec cmd 91 counts");
  }
}

//
//------------------------------------------
void sendDataEvent()     // Send data from Arduino to RPI (Need to be very, very quick)
{

  command = last_command_received;
  if ((command != 90) && (command != 91)){   ////  April 17, 2024 - Stop printing status commmands
    Serial.print ("c:");
    Serial.println (command);
  }

  for (int i = 0; i < array_size; i++) {    // Clear out old data
    send_data_array[i] = 0;
    }

  send_data_array[0] = 0;
  send_data_array[1] = 0;
  send_data_array[2] = 0;
  send_data_array[3] = 0;
  send_data_array[4] = 0;
  send_data_array[5] = 0;
  send_data_array[6] = 0;

//  Need to update this for waiting last command to complete
//  if (command == 10) {
//    send_data_array[length_of_send_data_array - 1] = last_command_complete;
//  }


//  if (command == 11) {
//    send_data_array[length_of_send_data_array - 1] = IF_A_status;
//  }
//
//  if (command == 12) {
//    send_data_array[length_of_send_data_array - 1] = IF_B_status;
//  }
//
//  if (command == 13) {
//    send_data_array[length_of_send_data_array - 1] = IF_C_status;
//  }

//=========================================

  if (command == 3) {

    send_data_array[0] = digitalRead(IF_A_LIMIT);
    send_data_array[1] = digitalRead(IF_B_LIMIT);
    send_data_array[2] = digitalRead(IF_C_LIMIT);

    // Returns zero when limit switch engaged

//    Serial.print ("L");
//    Serial.print (send_data_array[0]);
//    Serial.print (send_data_array[1]);
//    Serial.println (send_data_array[2]);


//    
//    IF_A__limit_sw_status = digitalRead(IF_A_LIMIT);
//    IF_B__limit_sw_status = digitalRead(IF_B_LIMIT);
//    IF_C__limit_sw_status = digitalRead(IF_C_LIMIT);
//
//    send_data_array[0] = IF_A__limit_sw_status;
//    send_data_array[1] = IF_B__limit_sw_status;
//    send_data_array[2] = IF_C__limit_sw_status;

    length_of_send_data_array = 3;
  }

//=========================================

  if (command == 17) {
    send_data_array[0] = IF_A_delta_encoder_Count;
    length_of_send_data_array = 1;
  }

  if (command == 27) {
    send_data_array[0] = IF_B_delta_encoder_Count;
    length_of_send_data_array = 1;
    }

  if (command == 37) {
    send_data_array[0] = IF_C_delta_encoder_Count;
    length_of_send_data_array = 1;
    }

  if (command == 90) {
    send_data_array[0] = IF_A_status;
    send_data_array[1] = IF_B_status;
    send_data_array[2] = IF_C_status;
    length_of_send_data_array = 3;
    }

  if (command == 91) {
    send_data_array[0] = (IF_A_rotation_counter >> 8) & 0xff;  
    send_data_array[1] = IF_A_rotation_counter % 256;          

    send_data_array[2] = (IF_B_rotation_counter >> 8) & 0xff;  
    send_data_array[3] = IF_B_rotation_counter % 256;   

    send_data_array[4] = (IF_C_rotation_counter >> 8) & 0xff;  
    send_data_array[5] = IF_C_rotation_counter % 256;   
    send_data_array[6] = last_command_complete;
    length_of_send_data_array = 7;
    }

  for (int i = 0; i < length_of_send_data_array; i++)
  {
    Wire.write(send_data_array[i]);
//    Serial.print(send_data_array[i]);
  }
//  Serial.println ("|");

  if (command == 90) {
    if (debug_control > 200){
        Serial.print("90st A");
      Serial.print(IF_A_status);
      Serial.print("B");
      Serial.print(IF_B_status);
      Serial.print("C");
      Serial.print(IF_C_status);
      Serial.println("|");
      }
  }

  if (command == 91) {
    if (debug_control > 200){
      Serial.print("91cts");
      Serial.print(IF_A_rotation_counter);
      Serial.print("|");
      Serial.print(IF_B_rotation_counter);
      Serial.print("|");
      Serial.print(IF_C_rotation_counter);
      Serial.println("|");
      }
  }

}  // end of Send Data Event
//------------------------------------------

//=================================================================================================
//=================================================================================================
//  
//=================================================================================================
bool IF_A_go_to_home(){
    last_command_complete = false;
//    Serial.print(F("LCC:"));  Serial.println(last_command_complete);
    if (debug_control > 2) Serial.println(F("IF_A_go_to_home ")); 
    
    action_successful = IF_A_drive_motor(IF_A_home_direction, IF_A_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    bool limit_switch_status = limit_switch_triggered(IF_A_LIMIT);   // Returns True when limit hit
    IF_A_motor_not_timedout = true;
        
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
          IF_A_status = 2;
          Serial.println ("HOME ACHIEVED");
      }
        
      if (debug_control > 2) 
        if (limit_switch_status) Serial.println(F("Homing Limit Switch A Detected"));
       
    }  // (end of WHILE)

    if (limit_switch_status){                //  Not sure why this code is used twice...
        IF_A_home_achieved = true;
        IF_A_status = 2;
        Serial.println ("HOME ACHIEVED");
    }

  action_successful = IF_A_drive_motor(IF_A_home_direction, 0);  // Stop the motor
  
  if (debug_control > 2) { Serial.print(F("IF_A_rotation_counter: "));  Serial.println(IF_A_rotation_counter);}
  IF_A_rotation_counter = 0;
  if (debug_control > 2) Serial.println(F("IF_A_rotation_counter reset to zero: "));

  Serial.println(F("GOTO HOME IF-A COMPLETE "));
  last_command_complete = true;
  Serial.print(F("LCC:"));  Serial.println(last_command_complete);

  return IF_A_home_achieved;
}

//=================================================================================================

bool IF_B_go_to_home(){
      last_command_complete = false;
//    Serial.print(F("LCC:"));  Serial.println(last_command_complete);

    if (debug_control > 2) Serial.println(F("IF_B_go_to_home ")); 
    
    action_successful = IF_B_drive_motor(IF_B_home_direction, IF_B_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    bool limit_switch_status = limit_switch_triggered(IF_B_LIMIT);   // Returns True when limit hit
    IF_B_motor_not_timedout = true;
        
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

      limit_switch_status = limit_switch_triggered(IF_B_LIMIT);

      if (limit_switch_status){
          IF_B_home_achieved = true;
          IF_B_status = 2;
          Serial.println ("HOME ACHIEVED");
      }
        
      if (debug_control > 2) 
        if (limit_switch_status) Serial.println(F("Homing Limit Switch B Detected"));
        
    }  // (end of WHILE)

    if (limit_switch_status){
        IF_B_home_achieved = true;
        IF_B_status = 2;
        Serial.println ("HOME ACHIEVED");
    }

  action_successful = IF_B_drive_motor(IF_B_home_direction, 0);  // Stop the motor
  
  if (debug_control > 2) { Serial.print(F("IF_B_rotation_counter: "));  Serial.println(IF_B_rotation_counter);}
  IF_B_rotation_counter = 0;
  if (debug_control > 2) Serial.print(F("IF_B_rotation_counter reset to zero: "));

  Serial.print(F("GOTO HOME IF-B COMPLETE "));
  last_command_complete = true;
  Serial.print(F("LCC:"));  Serial.println(last_command_complete);

  return IF_B_home_achieved;
}

//=================================================================================================

bool IF_C_go_to_home(){
      last_command_complete = false;
//    Serial.print(F("LCC:"));  Serial.println(last_command_complete);

    if (debug_control > 2) Serial.println(F("IF_C_go_to_home ")); 
    
    action_successful = IF_C_drive_motor(IF_C_home_direction, IF_C_slow_speed);

    unsigned long startMilliseconds = millis();
    unsigned long previousMillidisplay = millis();

    bool limit_switch_status = limit_switch_triggered(IF_C_LIMIT);   // Returns True when limit hit
    IF_C_motor_not_timedout = true;
        
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

      limit_switch_status = limit_switch_triggered(IF_C_LIMIT);

      if (limit_switch_status){
          IF_C_home_achieved = true;
          IF_C_status = 2;
          Serial.println ("HOME ACHIEVED");
      }
        
      if (debug_control > 2) 
        if (limit_switch_status) Serial.println(F("Homing Limit Switch C Detected"));
        
    }  // (end of WHILE)

    if (limit_switch_status){
        IF_C_home_achieved = true;
        IF_C_status = 2;
        Serial.println ("HOME ACHIEVED");
    }

  action_successful = IF_C_drive_motor(IF_C_home_direction, 0);  // Stop the motor
  
  if (debug_control > 2) { Serial.print(F("IF_C_rotation_counter: "));  Serial.println(IF_C_rotation_counter);}
  IF_C_rotation_counter = 0;
  if (debug_control > 2) Serial.print(F("IF_C_rotation_counter reset to zero: "));

  Serial.print(F("GOTO HOME IF-C COMPLETE "));
  last_command_complete = true;
  Serial.print(F("LCC:"));  Serial.println(last_command_complete);
  
  return IF_C_home_achieved;
}



//=================================================================================================

bool IF_A_drive_motor(int _direction, int _speed){

  digitalWrite(IF_A_DIR,   _direction);  // 
  analogWrite(IF_A_ENABLE_PWM, _speed); // where dutyCycle is a value from 0 to 255
  if (debug_control > 200) {
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
  if (debug_control > 200) {
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
  if (debug_control > 200) {
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
//bool IF_A_test_encoder(int direction){
//  
//  // get current encoder count 
//  // start motor
//  // start timer at zero
//  // while timer not at limit (0.5)
//  // Monitor and count encoder triggers
//  // stop motor
//  // Collect results
//  // Evaluate results
//  // 
//}

//=================================================================================================
//
// Safe Home
// Drive motor away from home at slow speed until stalled then home inwards


// Lookup range of typical encoder speeds for specific joint
// Save current encoder value
// Move joint away from home at slow speed for 0.5 second
// stop motor
// Save finish encoder value
// Calculate difference
// Save current encoder speed for 0.5 second
// Compare to typical value for specific joint
// Notify if out of tolerance
// Wait 0.25 second

// Repeat until encoder speed is 50% of typical speed (meaning hit away from home limit)

// Drive motor toward home at slow speed
// Monitor home position switch for trigger
// Stop Motor

//-------------------------------------------------------------------------------
void IF_A_go_away_from_home_until_limited(){

  last_command_complete = false;
  
  Serial.println (F("STARTING:  IF_A_Move_toward_home_for_one_Second()"));
  IF_A_Move_toward_home_for_one_Second();
   
  bool done = false;
  int check_counter = 0;

    // Lookup range of typical encoder speeds for specific joint
  typical_encoder_speed = IF_A_typical_encoder_cnt_in_sec;

  while (!done)
  {
    check_counter = check_counter + 1;
    
    // Save current encoder value
    starting_encoder_count = IF_A_rotation_counter;
    
    // Move joint away from home at slow speed for 0.5 second
    action_successful = IF_A_drive_motor(! IF_A_home_direction, IF_A_slow_speed);

    // Delay for 500 millisecond and monitor counts.

    unsigned long startMilliseconds = millis();
    unsigned long currentMillis = millis();
       
    while ( (currentMillis-startMilliseconds) < 500 ){
      IF_A_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      }  // (end of WHILE)
    
    // stop motor
    action_successful = IF_A_drive_motor(! IF_A_home_direction, 0);
    
    // Save finish encoder value
    ending_encoder_count = IF_A_rotation_counter;
  
    // Calculate difference
    IF_A_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("IF_A_delta_encoder_count: "); Serial.println (IF_A_delta_encoder_Count);
  
    // Save current encoder speed for 1 second
  
    Serial.print ("check_counter: "); Serial.println (check_counter);

    if (check_counter > 16){
      Serial.println ("Check Counter exceeded");
      done = true;
      check_counter = 0;
    }
    
    // Compare to typical value for specific joint
    int alarm_threshold;
    if (check_counter == 1){
      alarm_threshold = IF_A_delta_encoder_Count * 0.3;  
    }
    
    
    if (abs(IF_A_delta_encoder_Count) < alarm_threshold ){
      Serial.println ("Slow Encoder");
      done = true;
      check_counter = 0;
    }
 
    delay(1000);
    
    // Repeat until encoder speed is 30% of typical speed (meaning hit away from home limit)
  }  

    //  "last_command_complete" not updated here. Updating within GO_to_Home function

    Serial.println (F("Within Find home: - Before last IF-A goto home"));
    IF_A_go_to_home();
    Serial.println (F("Within Find home: - After last IF-A goto home"));
    
    // Drive motor toward home at slow speed
    // Monitor home position switch for trigger
    // Stop Motor
  
} // End of go_away_from_home_until_limited()
  
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

//-------------------------------------------------------------------------------
void IF_B_go_away_from_home_until_limited(){
  last_command_complete = false;
  Serial.println (F("STARTING:  IF_B_Move_toward_home_for_one_Second()"));
  IF_B_Move_toward_home_for_one_Second();
//  Serial.println (F("ENDING:  IF_B_Move_toward_home_for_one_Second()"));
   
  bool done = false;
  int check_counter = 0;
//  Serial.println ("Starting to move away: ");

    // Lookup range of typical encoder speeds for specific joint
  typical_encoder_speed = IF_B_typical_encoder_cnt_in_sec;
//  Serial.print ("typical_encoder_speed: "); Serial.println (typical_encoder_speed);

  while (!done)
  {
    check_counter = check_counter + 1;
    
    // Save current encoder value
    starting_encoder_count = IF_B_rotation_counter;
//    Serial.print ("starting_encoder_count: "); Serial.println (starting_encoder_count);
    
    // Move joint away from home at slow speed for 0.5 second
    action_successful = IF_B_drive_motor(! IF_B_home_direction, IF_B_slow_speed);
//    Serial.println ("Starting Motor "); 

    // Delay for 500 millisecond and monitor counts.

    unsigned long startMilliseconds = millis();
    unsigned long currentMillis = millis();
       
    while ( (currentMillis-startMilliseconds) < 500 ){
      IF_B_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      }  // (end of WHILE)
    
    // stop motor
    action_successful = IF_B_drive_motor(! IF_B_home_direction, 0);
    
    // Save finish encoder value
    ending_encoder_count = IF_B_rotation_counter;
//    Serial.print ("ending_encoder_count: "); Serial.println (ending_encoder_count);
  
    // Calculate difference
    IF_B_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("IF_B_delta_encoder_count: "); Serial.println (IF_B_delta_encoder_Count);
  
    // Save current encoder speed for 1 second
    
  
    Serial.print ("check_counter: "); Serial.println (check_counter);

    if (check_counter > 16){
      Serial.println ("Check Counter exceeded");
      done = true;
      check_counter = 0;
    }
    
    // Compare to typical value for specific joint
    int alarm_threshold;
    if (check_counter == 1){
      alarm_threshold = IF_B_delta_encoder_Count * 0.3;  
    }
    
//    Serial.print ("alarm_threshold: "); Serial.println (alarm_threshold);
    
//    if (abs( (2 * IF_B_delta_encoder_Count - typical_encoder_speed)) > alarm_threshold ){
    if (abs(IF_B_delta_encoder_Count) < alarm_threshold ){
      Serial.println ("Slow Encoder");
      done = true;
      check_counter = 0;
    }
 
    delay(1000);
    
    // Repeat until encoder speed is 30% of typical speed (meaning hit away from home limit)
  }  
    
    IF_B_go_to_home();
    
    // Drive motor toward home at slow speed
    // Monitor home position switch for trigger
    // Stop Motor
  
} // End of go_away_from_home_until_limited()
  
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

//-------------------------------------------------------------------------------
void IF_C_go_away_from_home_until_limited(){
  last_command_complete = false;
  Serial.println (F("STARTING:  IF_C_Move_toward_home_for_one_Second()"));
  IF_C_Move_toward_home_for_one_Second();
//  Serial.println (F("ENDING:  IF_C_Move_toward_home_for_one_Second()"));
   
  bool done = false;
  int check_counter = 0;
//  Serial.println ("Starting to move away: ");

    // Lookup range of typical encoder speeds for specific joint
  typical_encoder_speed = IF_C_typical_encoder_cnt_in_sec;
//  Serial.print ("typical_encoder_speed: "); Serial.println (typical_encoder_speed);

  while (!done)
  {
    check_counter = check_counter + 1;
    
    // Save current encoder value
    starting_encoder_count = IF_C_rotation_counter;
//    Serial.print ("starting_encoder_count: "); Serial.println (starting_encoder_count);
    
    // Move joint away from home at slow speed for 0.5 second
    action_successful = IF_C_drive_motor(! IF_C_home_direction, IF_C_slow_speed);
//    Serial.println ("Starting Motor "); 

    // Delay for 500 millisecond and monitor counts.

    unsigned long startMilliseconds = millis();
    unsigned long currentMillis = millis();
       
    while ( (currentMillis-startMilliseconds) < 500 ){
      IF_C_monitor_encoder(1);  // Decrement the counter while homing
      currentMillis = millis();
      }  // (end of WHILE)
    
    // stop motor
    action_successful = IF_C_drive_motor(! IF_C_home_direction, 0);
    
    // Save finish encoder value
    ending_encoder_count = IF_C_rotation_counter;
//    Serial.print ("ending_encoder_count: "); Serial.println (ending_encoder_count);
  
    // Calculate difference
    IF_C_delta_encoder_Count = abs(ending_encoder_count-starting_encoder_count);
    Serial.print ("IF_C_delta_encoder_count: "); Serial.println (IF_C_delta_encoder_Count);
  
    // Save current encoder speed for 1 second
    
  
    Serial.print ("check_counter: "); Serial.println (check_counter);

    if (check_counter > 16){
      Serial.println ("Check Counter exceeded");
      done = true;
      check_counter = 0;
    }
    
    // Compare to typical value for specific joint
    int alarm_threshold;
    if (check_counter == 1){
      alarm_threshold = IF_C_delta_encoder_Count * 0.3;  
    }
    
//    Serial.print ("alarm_threshold: "); Serial.println (alarm_threshold);
    
//    if (abs( (2 * IF_C_delta_encoder_Count - typical_encoder_speed)) > alarm_threshold ){
    if (abs(IF_C_delta_encoder_Count) < alarm_threshold ){
      Serial.println ("Slow Encoder");
      done = true;
      check_counter = 0;
    }
 
    delay(1000);
    
    // Repeat until encoder speed is 30% of typical speed (meaning hit away from home limit)
  }  
    
    IF_C_go_to_home();
    
    // Drive motor toward home at slow speed
    // Monitor home position switch for trigger
    // Stop Motor
  
} // End of go_away_from_home_until_limited()
  
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

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
//      Serial.print (".");
    }
  }
  IF_C_opt_A_previous_status = IF_C_opt_A_status;
}

//=================================================================================================
bool IF_A_move_to_target_Count(int target_count ){
    bool command_successful = false;
    last_command_complete = false;

    // Add some range checking of the target value here

    // Log  the command and target  count
    if (debug_control > 2) {
      Serial.print(F("IF_A moving to target count: ")); 
      Serial.print (target_count);             ////  April 17, 2024
      Serial.print ("/ Ang A: ");              ////  Angle to Count Conversion done in Python
      Serial.println (target_count / 12.3);    ////  Just reversing for debugging
      Serial.print ("  / Ang D: ");            ////  
      Serial.println (target_count / 14.286);  ////
      Serial.print(F("IF_A IF_A_home_achieved: ")); 
      Serial.println (IF_A_home_achieved);
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
    int  encoder_direction = 0; //  Used to determine if encoder should increment or decrement
    
    if (delta_count > 0){
        Serial.println (F("Joint to move away from home"));
        encoder_direction = 1;
        if (IF_A_home_direction == 1){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
      } else{
        Serial.println (F("Joint to move toward home"));
        encoder_direction = -1;
        if (IF_A_home_direction == 0){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
    }

    // Start the motor moving
    action_successful = IF_A_drive_motor( motion_direction, IF_A_max_speed);   // 

    // Monitor for achieving target count, WHILE monitoring timeout or hitting the home limit (unexpectedly)

    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
    bool IF_A_motor_not_at_target_count = true;
    int  target_count_window_size = 3;
    int  previousIF_A_rotation_counter = IF_A_rotation_counter;   ////  April 17, 2024
//    int  IF_A_JointMovementThreshold = 5;                              ////
  
    unsigned long startMilliseconds = millis();      // Timers for timeout monitoring
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_A_motor_not_timedout & IF_A_motor_not_at_target_count ){
      IF_A_monitor_encoder(encoder_direction);  // Increment the counter while going away from home.
                              //  This function updates the  interface counter:  IF_X_rotation_counter

      // Check to see if we have reached the target
      delta_count = target_count - IF_A_rotation_counter;  // Target - current count
      if ( abs(delta_count) < target_count_window_size ){
        IF_A_motor_not_at_target_count = false;
      }
        
      // Check to see if we have reached the counter limit for this specific Joint
      if ( IF_A_rotation_counter >= IF_A_range_full_count) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder A count limit hit"));
      }
      
      encoder_limit_not_hit = !encoder_limit_hit;   // ?? WHY not set to T or F

      // Check to see if we have reached the time limit for this specific Joint
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_A_motor_timeout_milliseconds) 
        {
        IF_A_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch A not triggered before timeout"));
        }

      // Print the current encoder count every 500 milliseconds  
      if (currentMillis - previousMillidisplay > 500)             ////  Apr 17, 2024 update change back to 500 msec 
        {
          // Determine if the encoder is moving by minimal amount          ////  Not Jammed
          if (abs (previousIF_A_rotation_counter-IF_A_rotation_counter) < IF_A_JointMovementThreshold){   ////
            if (debug_control > 2) {Serial.print(F("Encoder A TIMEOUT == ")); Serial.println(IF_A_rotation_counter); }
              IF_A_motor_not_timedout = false;                             //// Counter (Motor) not moving enough 
          }                                                                ////
          previousMillidisplay = currentMillis;
          previousIF_A_rotation_counter = IF_A_rotation_counter;           ////
          if (debug_control > 2) {Serial.print(F("Encoder A: ")); Serial.println(IF_A_rotation_counter); }
        }
    }

  //  Stop the motor
  action_successful = IF_A_drive_motor( ! IF_A_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_A_rotation_counter: "));  Serial.println(IF_A_rotation_counter); }

  command_successful = encoder_limit_not_hit && IF_A_motor_not_timedout;

  last_command_complete = true;

  return command_successful;
}

//=================================================================================================
bool IF_B_move_to_target_Count(int target_count ){
    bool command_successful = false;
    last_command_complete = false;
    
    // Add some range checking of the target value here

    // Log  the command and target  count
    if (debug_control > 2) {
      Serial.print(F("IF_B moving to target count: ")); 
      Serial.print (target_count);             ////  April 17, 2024
      Serial.print ("/ Ang B: ");              ////  Conversion done in python code
      Serial.println (target_count / 2.44);    ////
      Serial.print ("  / Ang E: ");            ////  
      Serial.println (target_count / 8.22);  ////
      Serial.print(F("IF_B IF_B_home_achieved: ")); 
      Serial.println (IF_B_home_achieved);
    }

    // Verify that the joint has been "homed" to make sure counter is  available
    
    if (!IF_B_home_achieved){
      Serial.println(F("Fail: Cannot move to target count because IF_B Not Homed"));
      return false;
    }
  
    // Determine the direction to move based on current position and target position
    // Moving toward home makes the position counter get smaller (negative)
    // Moving away from home (larger number) is positive
    
    int delta_count = target_count - IF_B_rotation_counter;  // Target - current count

    // Need to take into consideration that joints have different home directions
    // Home direction (IF_B_home_direction) is either 1 or 0 
    
    int  motion_direction = 0;  //  1 = moving outward , -1 = toward home 
    int  encoder_direction = 0; //  Used to determine if encoder should increment or decrement
    
    if (delta_count > 0){
        Serial.println (F("Joint to move away from home"));
        encoder_direction = 1;
        if (IF_B_home_direction == 1){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
      } else{
        Serial.println (F("Joint to move toward home"));
        encoder_direction = -1;
        if (IF_B_home_direction == 0){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
    }

    // Start the motor moving
    action_successful = IF_B_drive_motor( motion_direction, IF_B_max_speed);   // 

    // Monitor for achieving target count, WHILE monitoring timeout or hitting the home limit (unexpectedly)

    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
    bool IF_B_motor_not_at_target_count = true;
    int  target_count_window_size = 3;
    int  previousIF_B_rotation_counter = IF_B_rotation_counter;   ////  April 17, 2024
    int  JointMovementThreshold = 5;                              ////

  
    unsigned long startMilliseconds = millis();      // Timers for timeout monitoring
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_B_motor_not_timedout & IF_B_motor_not_at_target_count ){
      IF_B_monitor_encoder(encoder_direction);  // Increment the counter while going away from home.
                              //  This function updates the  interface counter:  IF_X_rotation_counter

      // Check to see if we have reached the target
      delta_count = target_count - IF_B_rotation_counter;  // Target - current count
      if ( abs(delta_count) < target_count_window_size ){
        IF_B_motor_not_at_target_count = false;
      }
        
      // Check to see if we have reached the counter limit for this specific Joint
      if ( IF_B_rotation_counter >= IF_B_range_full_count) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder B count limit hit"));
      }
      
      encoder_limit_not_hit = !encoder_limit_hit;   // ?? WHY not set to T or F

      // Check to see if we have reached the time limit for this specific Joint
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_B_motor_timeout_milliseconds) 
        {
        IF_B_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch B not triggered before timeout"));
        }

      // Print the current encoder count every 500 milliseconds  
      if (currentMillis - previousMillidisplay > 500) 
        {
          // Determine if the encoder is moving by minimal amount          ////  Not Jammed
          if (abs (previousIF_B_rotation_counter-IF_B_rotation_counter) < JointMovementThreshold){   ////
            if (debug_control > 2) {Serial.print(F("Encoder B TIMEOUT == ")); Serial.println(IF_B_rotation_counter); }
              IF_B_motor_not_timedout = false;                             //// Counter (Motor) not moving enough 
          }                                                                ////
          previousIF_B_rotation_counter = IF_B_rotation_counter;           ////
          previousMillidisplay = currentMillis;
          if (debug_control > 2) {Serial.print(F("Encoder B: ")); Serial.println(IF_B_rotation_counter); }
        }
    }

  //  Stop the motor
  action_successful = IF_B_drive_motor( ! IF_B_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_B_rotation_counter: "));  Serial.println(IF_B_rotation_counter); }

  command_successful = encoder_limit_not_hit && IF_B_motor_not_timedout;

  last_command_complete = true;

  return command_successful;
}

//=================================================================================================
bool IF_C_move_to_target_Count(int target_count ){
    bool command_successful = false;
    last_command_complete = false;
    
    // Add some range checking of the target value here

    // Log  the command and target  count
    if (debug_control > 2) {
      Serial.print(F("IF_C CURRENT count: ")); 
      Serial.println (IF_C_rotation_counter);
      Serial.print(F("IF_C moving to target count: ")); 
      Serial.print (target_count);              ////  April 17, 2024
      Serial.print ("/ Ang C: ");               ////  Conversion done in Python codde
      Serial.println (target_count / 9.283);    ////
      Serial.print ("  / Ang F: ");             ////  
      Serial.println (target_count / 4.235);    ////
      Serial.print(F("IF_C IF_C_home_achieved: ")); 
      Serial.println (IF_C_home_achieved);
    }

    // Verify that the joint has been "homed" to make sure counter is  available
    
    if (!IF_C_home_achieved){
      Serial.println(F("Fail: Cannot move to target count because IF_C Not Homed"));
      return false;
    }
  
    // Determine the direction to move based on current position and target position
    // Moving toward home makes the position counter get smaller (negative)
    // Moving away from home (larger number) is positive
    
    int delta_count = target_count - IF_C_rotation_counter;  // Target - current count

    // Need to take into consideration that joints have different home directions
    // Home direction (IF_C_home_direction) is either 1 or 0 
    
    int  motion_direction = 0;  //  1 = moving outward , -1 = toward home 
    int  encoder_direction = 0; //  Used to determine if encoder should increment or decrement
    
    if (delta_count > 0){
        Serial.println (F("Joint to move away from home"));
        encoder_direction = 1;
        if (IF_C_home_direction == 1){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
      } else{
        Serial.println (F("Joint to move toward home"));
        encoder_direction = -1;
        if (IF_C_home_direction == 0){
          motion_direction = 0;
        } else {
          motion_direction = 1;}
    }

    // Start the motor moving
////    action_successful = IF_C_drive_motor( motion_direction, IF_C_slow_speed);   //  //// April 17, 2024 speedup
    action_successful = IF_C_drive_motor( motion_direction, IF_C_max_speed);   // 
    Serial.print (F("FAST SPEED  "));
    Serial.println (IF_C_max_speed);

            

    // Monitor for achieving target count, WHILE monitoring timeout or hitting the home limit (unexpectedly)

    bool encoder_limit_not_hit = true;
    bool encoder_limit_hit = false;
    bool IF_C_motor_not_at_target_count = true;
    int  target_count_window_size = 10;
    int  previousIF_C_rotation_counter = IF_C_rotation_counter;   ////  April 17, 2024
    int  JointMovementThreshold = 5;                              ////

  
    unsigned long startMilliseconds = millis();      // Timers for timeout monitoring
    unsigned long previousMillidisplay = millis();

    while ( encoder_limit_not_hit && IF_C_motor_not_timedout & IF_C_motor_not_at_target_count ){
      IF_C_monitor_encoder(encoder_direction);  // Increment the counter while going away from home.
                              //  This function updates the  interface counter:  IF_X_rotation_counter

      // Check to see if we have reached the target
      delta_count = target_count - IF_C_rotation_counter;  // Target - current count
      if ( abs(delta_count) < target_count_window_size ){
        IF_C_motor_not_at_target_count = false;
      }
        
      // Check to see if we have reached the counter limit for this specific Joint
      if ( IF_C_rotation_counter >= IF_C_range_full_count) {
        encoder_limit_hit = true;
        if (debug_control > 2) Serial.println(F("Encoder C count limit hit"));
      }
      
      encoder_limit_not_hit = !encoder_limit_hit;   // 

      // Check to see if we have reached the time limit for this specific Joint
      currentMillis = millis();
      if (currentMillis - startMilliseconds > IF_C_motor_timeout_milliseconds) 
        {
        IF_C_motor_not_timedout = false;   
        if (debug_control > 2) Serial.println(F("Limit Switch C not triggered before timeout"));
        }

      // Print the current encoder count every 500 milliseconds  
      if (currentMillis - previousMillidisplay > 500) 
        {
          // Determine if the encoder is moving by minimal amount          ////  Not Jammed
          if (abs (previousIF_C_rotation_counter-IF_C_rotation_counter) < JointMovementThreshold){   ////
            if (debug_control > 2) {Serial.print(F("Encoder C TIMEOUT == ")); Serial.println(IF_C_rotation_counter); }
              IF_C_motor_not_timedout = false;                             //// Counter (Motor) not moving enough 
          }                                                                ////
          previousIF_C_rotation_counter = IF_C_rotation_counter;           ////
          previousMillidisplay = currentMillis;
          if (debug_control > 2) {Serial.print(F("Encoder C: ")); Serial.println(IF_C_rotation_counter); }
        }
    }

  //  Stop the motor
  action_successful = IF_C_drive_motor( ! IF_C_home_direction, 0);
  if (debug_control > 2) { Serial.print(F("IF_C_rotation_counter: "));  Serial.println(IF_C_rotation_counter); }

  command_successful = encoder_limit_not_hit && IF_C_motor_not_timedout;
  last_command_complete = true;

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
  case 'A':    // Joint A  
    IF_X_range_full_count           = 80;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 120;    // PWM value //// Was 80
    IF_X_max_speed                  = 255;    // PWM value
    IF_X_motor_timeout_milliseconds = 10000;  // Milliseconds
    IF_X_typical_encoder_cnt_in_sec = 20;    // Typical Counts in 1 second
    IF_X_JointMovementThreshold     = 5;     //  One third of the one second count
    break;

  case 'B':    // Joint B   
    IF_X_range_full_count           = 880;   // Counts
    IF_X_home_direction             = 1;      //
    IF_X_slow_speed                 = 220;    // PWM value
    IF_X_max_speed                  = 250;    // PWM value
    IF_X_motor_timeout_milliseconds = 10000;  // Milliseconds
    IF_X_typical_encoder_cnt_in_sec = 68;    // Typical Counts in 1 second
    IF_X_JointMovementThreshold     = 12;
    break;
  
  case 'C':    // Joint C   
    IF_X_range_full_count           = 1300;   // Counts
    IF_X_home_direction             = 1;      //
    IF_X_slow_speed                 = 200;    // PWM value
    IF_X_max_speed                  = 250;    // PWM value
    IF_X_motor_timeout_milliseconds = 15000;  // Milliseconds
    IF_X_typical_encoder_cnt_in_sec = 94;    // Typical Counts in 1 second
    IF_X_JointMovementThreshold     = 33;
    break;

  case 'D':    // Joint D   
    IF_X_range_full_count           = 700;   // Counts
    IF_X_home_direction             = 1;      //   WAS 0 prior to 12/26/2023
    IF_X_slow_speed                 = 200;    // PWM value
    IF_X_max_speed                  = 255;    // PWM value
    IF_X_motor_timeout_milliseconds = 10000;  // Milliseconds
    IF_X_typical_encoder_cnt_in_sec = 100;    // Typical Counts in 1 second
    IF_X_JointMovementThreshold     = 15;
    break;
    
  case 'E':    // Joint E   
    IF_X_range_full_count           = 740;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 200;    // PWM value
    IF_X_max_speed                  = 255;    // PWM value
    IF_X_motor_timeout_milliseconds = 10000;  // Milliseconds
    IF_X_typical_encoder_cnt_in_sec = 68;    // Typical Counts in 1 second
    IF_X_JointMovementThreshold     = 35;
    break;
    
  case 'F':    // Joint C   
    IF_X_range_full_count           = 720;   // Counts
    IF_X_home_direction             = 0;      //
    IF_X_slow_speed                 = 200;    // PWM value
    IF_X_max_speed                  = 255;    // PWM value    ////  April 17, 2024
    IF_X_motor_timeout_milliseconds = 10000;  // Milliseconds
    IF_X_typical_encoder_cnt_in_sec = 77;    // Typical Counts in 1 second
    IF_X_JointMovementThreshold     = 52;
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
    IF_A_motor_timeout_milliseconds = IF_X_motor_timeout_milliseconds; 
    IF_A_typical_encoder_cnt_in_sec = IF_X_typical_encoder_cnt_in_sec;
    IF_A_JointMovementThreshold     = IF_X_JointMovementThreshold;
    break;

  case 'B':    // Joint B   
    IF_B_range_full_count           = IF_X_range_full_count;
    IF_B_home_direction             = IF_X_home_direction;             
    IF_B_slow_speed                 = IF_X_slow_speed;                 
    IF_B_max_speed                  = IF_X_max_speed;                  
    IF_B_motor_timeout_milliseconds = IF_X_motor_timeout_milliseconds; 
    IF_B_typical_encoder_cnt_in_sec = IF_X_typical_encoder_cnt_in_sec;
    IF_B_JointMovementThreshold     = IF_X_JointMovementThreshold;
    break;
  
  case 'C':    // Joint C   
    IF_C_range_full_count           = IF_X_range_full_count;
    IF_C_home_direction             = IF_X_home_direction;             
    IF_C_slow_speed                 = IF_X_slow_speed;                 
    IF_C_max_speed                  = IF_X_max_speed;                  
    IF_C_motor_timeout_milliseconds = IF_X_motor_timeout_milliseconds; 
    IF_C_typical_encoder_cnt_in_sec = IF_X_typical_encoder_cnt_in_sec;
    IF_C_JointMovementThreshold     = IF_X_JointMovementThreshold;
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
    Serial.print(F("IF_A_JointMovementThreshold: "));     Serial.println(IF_A_JointMovementThreshold);
    Serial.println(F(" "));

    Serial.print(F("IF_B_range_full_count: "));           Serial.println(IF_B_range_full_count);
    Serial.print(F("IF_B_home_direction: "));             Serial.println(IF_B_home_direction);
    Serial.print(F("IF_B_slow_speed: "));                 Serial.println(IF_B_slow_speed);
    Serial.print(F("IF_B_max_speed: "));                  Serial.println(IF_B_max_speed);
    Serial.print(F("IF_B_motor_timeout_milliseconds: ")); Serial.println(IF_B_motor_timeout_milliseconds);
    Serial.print(F("IF_B_JointMovementThreshold: "));     Serial.println(IF_B_JointMovementThreshold);
    Serial.println(F(" "));

    Serial.print(F("IF_C_range_full_count: "));           Serial.println(IF_C_range_full_count);
    Serial.print(F("IF_C_home_direction: "));             Serial.println(IF_C_home_direction);
    Serial.print(F("IF_C_slow_speed: "));                 Serial.println(IF_C_slow_speed);
    Serial.print(F("IF_C_max_speed: "));                  Serial.println(IF_C_max_speed);
    Serial.print(F("IF_C_motor_timeout_milliseconds: ")); Serial.println(IF_C_motor_timeout_milliseconds);
    Serial.print(F("IF_C_JointMovementThreshold: "));     Serial.println(IF_C_JointMovementThreshold);
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

  pinMode(slave_address_sensor_pin, INPUT);
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
