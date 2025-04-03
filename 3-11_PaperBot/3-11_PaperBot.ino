//Libraries needed
#include <Wire.h>
#include <HardwareSerial.h>
//AccelStepper.h was used previously, but has been replaced with a custom library outlined below.

//encoder stuff below
//encoders have two signals, A+B, which are square waves,
//having just one signal allows for tracking motion and position, but not direction—
//with both, one can extrapolate direction from which 

#define ENCODER_A1 7  // Must be an interrupt-capable pin (21) attached to Z motor 1
#define ENCODER_B1 8  // Must be an interrupt-capable pin (30) attached to Z motor 1

volatile long encoderPosition1 = 0;
volatile long global_target_encoderPosition1 = 0;
volatile int lastEncoded1 = 0;


#define ENCODER_A2 3  // Must be an interrupt-capable pin 2, attached to Z motor 2
#define ENCODER_B2 4  // Must be an interrupt-capable pin 3, attached to Z motor 2

volatile long encoderPosition2 = 0;
volatile long global_target_encoderPosition2 = 0;
volatile int lastEncoded2 = 0;

#define ENCODER_Ax 5  // Must be an interrupt-capable pin, attached to X-axis motor
#define ENCODER_Bx 6  // Must be an interrupt-capable pin, attached to X-axis motor

long encoderPositionx = 0;
long global_target_encoderPositionx = 0;
int lastEncodedx = 0;

#define ENCODER_Ay 1  // Must be an interrupt-capable pin, attached to Y-axis motor
#define ENCODER_By 2  // Must be an interrupt-capable pin, attached to Y-axis motor

volatile long encoderPositiony = 0;
volatile long global_target_encoderPositiony = 0;
volatile int lastEncodedy = 0;

//end encoder stuff

//pressure regulator settings below

int ArdBaud = 19200;   // Baud rate for the serial monitor [on the Arduino]
int AlctBaud = 19200;  // Baud rate on the Alicat (see RS-232 settings); default is 19200

int TxPin = 49;  // Arduino Tx pin, connected to the Alicat Rx RS-232 pin (pin 3 on the 8 pin mini-DIN)
int RxPin = 50;  // Arduino Rx pin, connected to the Alicat Tx RS-232 pin (pin 5 on the 8 pin mini-DIN)

const int NUMBER_OF_FORMULATIONS = 4;

//timing for flushing and collecting

int flush_time = 20000; //time to flush lines, ms
int collect_time = 15000;//time to collect, ms

// Array of strings to store pressure values for each formulation
char arr[NUMBER_OF_FORMULATIONS][6][10] = { //the 10 on this array is for 10 bits of each number's string.
  { "16", "24", "24", "24", "36", "36" },
  { "20", "20", "24", "30", "30", "36" },
  { "24", "16", "24", "36", "24", "36" },
  { "28", "12", "24", "42", "18", "36" },
};


HardwareSerial mySerial(2); //defines that we are using TX1 and RX1 UART pins of ESP32 chip, which are pins 17+18
#define RXD1 18 //RXD pin used on ESP32
#define TXD1 17 //TXD pin used on ESP32

char PressureE1[10]; //all the packets sent to pressure regs are 10 characters long (length of ten bytes).
char PressureE2[10];
char PressureE3[10];
char PressureA1[10];
char PressureA2[10];
char PressureA3[10];

const int reg_array_size = 30;//for an array that keeps the 30 most recent pressure values

int E1_reg_array[reg_array_size] = { 0 };
int E2_reg_array[reg_array_size] = { 0 };
int E3_reg_array[reg_array_size] = { 0 };
int A1_reg_array[reg_array_size] = { 0 };
int A2_reg_array[reg_array_size] = { 0 };
int A3_reg_array[reg_array_size] = { 0 };

int formulation_counter = 0; //iterator variable
int pressureFormulation = 0; //iterator variable

// int E1 = 5; //assigning pins for each regulator.
// int E2 = 6;
// int E3 = 7;
// int pin_A1 = 8;
// int pin_A2 = 12;
// int pin_A3 = 13;

int E1_reg_pin = A15; //assigning pins for each regulator
int E2_reg_pin = A13;
int E3_reg_pin = A14;
int A1_reg_pin = A12;
int A2_reg_pin = A11;
int A3_reg_pin = A10;

int delta_smooth = 5; //limit on the acceptable smoothness of PID signal
int delta_abs = 10; //limit on pressure accuracy
bool kill_switch = false; //kill switch/button variable.

bool equilibrate = true; //state variable

int R_index = 0; // the index in the regulator array, the current pressure
int counter = 0; // iterator variable for the position in the regulator array (I think)

//end pressure regulator settings

// Constants and Configurations (global variables for robotic motion)
int z_top = 1000; //in ACTUATION steps, from the bottom position. equivalent to 5".
int Z_HOME = 4000; //comfortable height where needles are retracted and won't brush the plate, in terms of encoder steps (from the top). I.e., we don't retract the needles all the way to the maximum/top height, because that would be less efficient.
float act_rot = 200; //the number of actuations per rotation is 800 
float enc_rot = 4000; // the number of encoder counts per rotation is 4000
int delaymotor = 0; // the number of microseconds used for delays between steps.
bool moveFlag = false;

//BASED ON THE PLATE USED: MAY CHANGE
const int STEPS_TO_BOTTOM_Z = 11195;//in steps; position where needles are fully within the wells. referenced from top so take off z-home for all other movements was +425
const int center_x_1 = 13000; //X distance from starting point to X center of first well (upper left) in encoder steps
const int center_y_1= 8000; // Y distance from starting point to Y center of first well in encoder steps.
const int Y_STEPS_PER_POSITION = 5669; //distance between y positions, in terms of steps. Will change if plate is changed.
const int X_STEPS_PER_POSITION = 2835; //distance between X positions, in terms of steps. Will change is plate is changed.

//pins
const int DIR_PIN_z1 = 41;
const int STEP_PIN_z1 = 42;
const int DIR_PIN_z2 = 37; //46
const int STEP_PIN_z2 = 38; //47
const int DIR_PIN_y = 35;
const int STEP_PIN_y = 36;
const int DIR_PIN_x = 39;
const int STEP_PIN_x = 40;


const int Calibration_BUTTON = 12; //pin for button

bool comm_value = true;

int tolerance = 40;


bool first_round_flag = true;

// Functionality variables
bool verticalStepperDir = false; //flip to change directionality of Z motor
bool finishedFlag = false; //used to stop the void loop()

//New functions defined

// updateEncoder functions track the position of each motor
void updateEncoder1() {
    int A_1 = digitalRead(ENCODER_A1);
    int B_1 = digitalRead(ENCODER_B1);
    int encoded1 = (A_1 << 1) | B_1;
    int sum1 = (lastEncoded1 << 2) | encoded1;

    if (sum1 == 0b0001 || sum1 == 0b0111 || sum1 == 0b1110 || sum1 == 0b1000) {
        encoderPosition1++;
    } else if (sum1 == 0b0010 || sum1 == 0b1101 || sum1 == 0b1011 || sum1 == 0b0100) {
        encoderPosition1--;
    }
    lastEncoded1 = encoded1;
}

void updateEncoder2() {
    int A_2 = digitalRead(ENCODER_A2);
    int B_2 = digitalRead(ENCODER_B2);
    int encoded2 = (A_2 << 1) | B_2;
    int sum2 = (lastEncoded2 << 2) | encoded2;

    if (sum2 == 0b0001 || sum2 == 0b0111 || sum2 == 0b1110 || sum2 == 0b1000) {
        encoderPosition2++;
    } else if (sum2 == 0b0010 || sum2 == 0b1101 || sum2 == 0b1011 || sum2 == 0b0100) {
        encoderPosition2--;
    }
    lastEncoded2 = encoded2;
}

void updateEncoderx(){

  int A_x = digitalRead(ENCODER_Ax);
  int B_x = digitalRead(ENCODER_Bx);
  int encodedx = (A_x << 1) | B_x;
  int sumx = (lastEncodedx << 2) | encodedx;
  if (sumx == 0b0001 || sumx == 0b0111 || sumx == 0b1110 || sumx == 0b1000) {
      encoderPositionx++;
  } else if (sumx == 0b0010 || sumx == 0b1101 || sumx == 0b1011 || sumx == 0b0100) {
      encoderPositionx--;
  }
  lastEncodedx = encodedx;
}

void updateEncodery(){
  int A_y = digitalRead(ENCODER_Ay);
  int B_y = digitalRead(ENCODER_By);
  int encodedy = (A_y << 1) | B_y;
  int sumy = (lastEncodedy << 2) | encodedy;
  if (sumy == 0b0001 || sumy == 0b0111 || sumy == 0b1110 || sumy == 0b1000) {
    encoderPositiony++;
  } else if (sumy == 0b0010 || sumy == 0b1101 || sumy == 0b1011 || sumy == 0b0100) {
    encoderPositiony--;
  }
  lastEncodedy = encodedy;
}

//move [] stepper(s) functions written to replace accelstepper library. 

void move_x_stepper(long number_of_encoder_counts, int dir){
  int step_type = 1;
  if (dir == 0){
    global_target_encoderPositionx = global_target_encoderPositionx - number_of_encoder_counts;
    while (abs(global_target_encoderPositionx - encoderPositionx)>tolerance){
      int fudge_steps = (int)((((float)(encoderPositionx-global_target_encoderPositionx))/enc_rot)*act_rot);
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_x,LOW);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(delaymotor);
        }
      }
      else{
        digitalWrite(DIR_PIN_x,HIGH);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(delaymotor);
        }
      }
    }
  }
  else{
    global_target_encoderPositionx = global_target_encoderPositionx + number_of_encoder_counts;
    while (abs(global_target_encoderPositionx- encoderPositionx)>tolerance){
      int fudge_steps = (int)((((float)(encoderPositionx-global_target_encoderPositionx))/enc_rot)*act_rot);

      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_x,LOW);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(delaymotor);
        }
      }
      else{
        digitalWrite(DIR_PIN_x,HIGH);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(delaymotor);
        }
      }
    }
  }
}


void move_y_stepper(int number_of_encoder_counts, int dir){
  int step_type = 1;
  if (dir == 0){
    global_target_encoderPositiony = global_target_encoderPositiony - number_of_encoder_counts;
    while (abs(global_target_encoderPositiony - encoderPositiony)>tolerance){
      int fudge_steps = (int)((((float)(encoderPositiony-global_target_encoderPositiony))/enc_rot)*act_rot);
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_y,LOW);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(delaymotor);
        }
      }
      else{
        digitalWrite(DIR_PIN_y,HIGH);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(delaymotor);
        }
      }
    }
  }
  else{
    global_target_encoderPositiony = global_target_encoderPositiony + number_of_encoder_counts;
    while (abs(global_target_encoderPositiony- encoderPositiony)>tolerance){
      int fudge_steps = (int)((((float)(encoderPositiony-global_target_encoderPositiony))/enc_rot)*act_rot);
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_y,LOW);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(delaymotor);
        }
      }
      else{
        digitalWrite(DIR_PIN_y,HIGH);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(delaymotor);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(delaymotor);
        }
      }
    }
  }
}

void move_z_steppers(int encoder_counts, int dir){
  int fudge_steps_1 = 0;
  int fudge_steps_2 = 0;
  int counter_z1 = 0;
  int counter_z2 = 0;
  int increment_z1 = 0;
  int increment_z2 = 0;
  if (dir == 0){
    global_target_encoderPosition1 = global_target_encoderPosition1 - encoder_counts;
  }
  else{
    global_target_encoderPosition1 = global_target_encoderPosition1 + encoder_counts;
  }
  while ((abs(global_target_encoderPosition1- encoderPosition1)>tolerance)|(abs(global_target_encoderPosition1- encoderPosition2)>tolerance)){
    counter_z1 = 0;
    counter_z2 = 0;
    if (abs(global_target_encoderPosition1 - encoderPosition1)>tolerance){
      fudge_steps_1 = (int)((((float)(encoderPosition1-global_target_encoderPosition1))/enc_rot)*act_rot);
    }
    else{
      fudge_steps_1 = 0;
    }
    if (abs(global_target_encoderPosition1 - encoderPosition2)>tolerance){
      fudge_steps_2 = (int)((((float)(encoderPosition2-global_target_encoderPosition1))/enc_rot)*act_rot);
    }
    else{
      fudge_steps_2 = 0;
    }

    if (fudge_steps_1 < 0){
      digitalWrite(DIR_PIN_z1,LOW);
      increment_z1 = -1;
      delay(10);
    }
    else{
      digitalWrite(DIR_PIN_z1,HIGH);
      increment_z1 = 1;
      delay(10);
    }

    if (fudge_steps_2 < 0){
      digitalWrite(DIR_PIN_z2,LOW);
      increment_z2 = -1;
      delay(10);
    }
    else{
      digitalWrite(DIR_PIN_z2,HIGH);
      increment_z2 = 1;
      delay(10);
    }

    while ((abs(counter_z1-fudge_steps_1)>0)&&(abs(counter_z2-fudge_steps_2)>0)){
      counter_z1 = counter_z1 + increment_z1;
      counter_z2 = counter_z2 + increment_z2;

      digitalWrite(STEP_PIN_z1,HIGH);
      digitalWrite(STEP_PIN_z2,HIGH);
      delayMicroseconds(delaymotor);
      digitalWrite(STEP_PIN_z1,LOW);
      digitalWrite(STEP_PIN_z2,LOW);
      delayMicroseconds(delaymotor);
    }
    while (abs(counter_z2-fudge_steps_2)>0){
      counter_z2 = counter_z2 + increment_z2;

      digitalWrite(STEP_PIN_z2,HIGH);
      delayMicroseconds(delaymotor);
      digitalWrite(STEP_PIN_z2,LOW);
      delayMicroseconds(delaymotor);
    }
    while (abs(counter_z1-fudge_steps_1)>0){
      counter_z1 = counter_z1 + increment_z1;

      digitalWrite(STEP_PIN_z1,HIGH);
      delayMicroseconds(delaymotor);
      digitalWrite(STEP_PIN_z1,LOW);
      delayMicroseconds(delaymotor);
    }
  }
}

//centering and calibration functions
void center(){
  Serial.println("here1");
  move_x_stepper(center_x_1, 0); // Move to starting well postition
  Serial.println("here2");
  // delay(100);
  move_y_stepper(center_y_1, 0); // Move to starting well position
  Serial.println("here3");
}

void calibrateMotors() {
  // while (digitalRead(Calibration_BUTTON)==LOW);
  encoderPositionx = 0;
  encoderPositiony = 0;
  encoderPosition1 = 0;
  encoderPosition2 = 0;
  Serial.println("here");
  while(digitalRead(Calibration_BUTTON)==LOW);
  center();
  while(digitalRead(Calibration_BUTTON)==LOW);
  // move_z_steppers(Z_HOME,0);
}

void moveVertStepper(int direction) {
  if (verticalStepperDir) {
      verticalStepperDir = false;
      move_z_steppers(STEPS_TO_BOTTOM_Z,1);
  }
  else {
      verticalStepperDir = true;
      move_z_steppers(STEPS_TO_BOTTOM_Z,0);
  }
}

//Pressure regulator functions

void formatPressureValue(char* buffer, const char* prefix, const char* value) { //formats pressure to send to the regulator
  strcpy(buffer, prefix); // Copy the prefix
  strcat(buffer, value); // Concatenate the value to the buffer
  strcat(buffer, "\r"); // Append the carriage return
}


void setPressures(char* E1,char* E2,char* E3,char* A1,char* A2,char* A3){ //function that sets the pressures by formatting pressure values and sending them through the serial
  formatPressureValue(PressureE1, "AS", E1);
  formatPressureValue(PressureE2, "BS", E2);
  formatPressureValue(PressureE3, "CS", E3);
  formatPressureValue(PressureA1, "DS", A1);
  formatPressureValue(PressureA2, "ES", A2);
  formatPressureValue(PressureA3, "FS", A3);
  for (int i = 0; i < strlen(PressureE1); i++) {
    mySerial.print(PressureE1[i]); // Print each character of PressureE1 to mySerial
  }
  for (int i = 0; i < strlen(PressureE2); i++) {
    mySerial.print(PressureE2[i]); // Print each character of PressureE2 to mySerial
  }
  for (int i = 0; i < strlen(PressureE3); i++) {
    mySerial.print(PressureE3[i]); // Print each character of PressureE3 to mySerial
  }
  for (int i = 0; i < strlen(PressureA1); i++) {
    mySerial.print(PressureA1[i]); // Print each character of PressureA1 to mySerial
  }
  for (int i = 0; i < strlen(PressureA2); i++) {
    mySerial.print(PressureA2[i]); // Print each character of PressureA2 to mySerial
  }
  for (int i = 0; i < strlen(PressureA3); i++) {
    mySerial.print(PressureA3[i]); // Print each character of PressureA3 to mySerial
  }
}

void equilibrize() {
  // Serial.println("start:");
  // Serial.println(millis());
  // Serial.println(formulation_counter);
  setPressures(arr[formulation_counter][0],arr[formulation_counter][1],arr[formulation_counter][2],arr[formulation_counter][3],arr[formulation_counter][4],arr[formulation_counter][5]); 
  int R_index_old = 4;
  int old_index = 0;

  int E1_smooth = 10;
  int E2_smooth = 10;
  int E3_smooth = 10;
  int A1_smooth = 10;
  int A2_smooth = 10;
  int A3_smooth = 10;

  int E1_target = ((atof(arr[formulation_counter][0])/50)*5)*(1023.0/5);
  int E2_target = ((atof(arr[formulation_counter][1])/50)*5)*(1023.0/5);
  int E3_target = ((atof(arr[formulation_counter][2])/50)*5)*(1023.0/5);
  int A1_target = ((atof(arr[formulation_counter][3])/50)*5)*(1023.0/5);
  int A2_target = ((atof(arr[formulation_counter][4])/100)*5)*(1023.0/5);
  int A3_target = ((atof(arr[formulation_counter][5])/50)*5)*(1023.0/5);
  Serial.println("start_time");
  Serial.println(millis());
  
  while ((((abs(E1_reg_array[R_index] - E1_target)>delta_abs)|(E1_smooth > delta_smooth)) | ((abs(E2_reg_array[R_index] - E2_target)>delta_abs)|(E2_smooth > delta_smooth)) | ((abs(E3_reg_array[R_index] - E3_target)>delta_abs)|(E3_smooth > delta_smooth)) | ((abs(A1_reg_array[R_index] - A1_target)>delta_abs)|(A1_smooth > delta_smooth)) | ((abs(A3_reg_array[R_index] - A3_target)>delta_abs)|(A3_smooth > delta_smooth))) & (kill_switch == false)) {
    delay(50);//stop the code from hitting early (checks to fast that it just always sees the same value)
    E1_reg_array[R_index] = analogRead(E1_reg_pin);
    E2_reg_array[R_index] = analogRead(E2_reg_pin);
    E3_reg_array[R_index] = analogRead(E3_reg_pin);
    A1_reg_array[R_index] = analogRead(A1_reg_pin);
    A2_reg_array[R_index] = analogRead(A2_reg_pin);
    A3_reg_array[R_index] = analogRead(A3_reg_pin);
    // Serial.println(E1_reg_array[R_index]);
    // Serial.println(E1_target);
    // Serial.println(E2_reg_array[R_index]);
    // Serial.println(E2_target);
    // Serial.println(E3_reg_array[R_index]);
    // Serial.println(E3_target);
    // Serial.println(A1_reg_array[R_index]);
    // Serial.println(A1_target);
    // Serial.println(A2_reg_array[R_index]);
    // Serial.println(A2_target);
    // Serial.println(A3_reg_array[R_index]);
    // Serial.println(A3_target);
    // Serial.println("\n\n--------\n\n");
    counter = counter + 1;
    R_index = counter % reg_array_size;
    if (counter >= R_index_old) {
      old_index = (counter - R_index_old) % reg_array_size;
      E1_smooth = abs(E1_reg_array[R_index] - E1_reg_array[old_index]);
      E2_smooth = abs(E2_reg_array[R_index] - E2_reg_array[old_index]);
      E3_smooth = abs(E3_reg_array[R_index] - E3_reg_array[old_index]);
      A1_smooth = abs(A1_reg_array[R_index] - A1_reg_array[old_index]);
      A2_smooth = abs(A2_reg_array[R_index] - A2_reg_array[old_index]);
      A3_smooth = abs(A3_reg_array[R_index] - A3_reg_array[old_index]);
    }   
    if (digitalRead(4) == HIGH) {
      kill_switch = true;
      setPressures("0","0","0","0","0","0");
    }
  }
  Serial.println("end_time");
  Serial.println(millis());
  // Serial.println("time to finish: ");
  // Serial.println(millis());
  unsigned long int start_time = millis();
  while (((millis() - start_time) < 2000) & (kill_switch == false)) {
    if (digitalRead(4) == HIGH) {
      kill_switch = true;
      setPressures("0","0","0","0","0","0");
    }
  }
  formulation_counter = formulation_counter + 1;
}

void deposit() {
  unsigned long int start_deposit = millis();
  while (((millis() - start_deposit) < collect_time) & (kill_switch == false)) {
    if (digitalRead(Calibration_BUTTON) == HIGH) {
      kill_switch = true;
      setPressures("0","0","0","0","0","0");
    }
  }
}

void flush_lines(){
  unsigned long int start_deposit = millis();
  setPressures(arr[0][0],arr[0][1],arr[0][2],arr[0][3],arr[0][4],arr[0][5]); 
  while (((millis() - start_deposit) < flush_time) & (kill_switch == false)) {
    if (digitalRead(Calibration_BUTTON) == HIGH) {
      kill_switch = true;
      setPressures("0","0","0","0","0","0");
    }
  }
}


void gen_and_collect(){
  // center();
  long z_encoder = 1750;
  move_z_steppers(300,1);
  move_z_steppers(z_encoder,1);
  delay(delaymotor);
  for(int movement_iterator=0;movement_iterator<5;movement_iterator++){
    Serial.println("in the for loop");
    for(int j=0;j<9;j++){
      move_z_steppers(z_encoder,0);
      move_x_stepper(X_STEPS_PER_POSITION, 1);
      delay(delaymotor);
      move_z_steppers(z_encoder,1);
      delay(delaymotor);
    }
    move_z_steppers(z_encoder,0);
    move_y_stepper(Y_STEPS_PER_POSITION, 0);
    delay(delaymotor);
    move_z_steppers(z_encoder,1);
    delay(delaymotor);
    for(int j=0;j<4;j++){
      move_z_steppers(z_encoder,0);
      move_x_stepper(X_STEPS_PER_POSITION, 0);
      delay(delaymotor);
      move_z_steppers(z_encoder,1);
      delay(delaymotor);
    }
    move_z_steppers(z_encoder,0);
    move_y_stepper(Y_STEPS_PER_POSITION, 1);
    delay(delaymotor);
    move_z_steppers(z_encoder,1);
    delay(delaymotor);
  }
  move_z_steppers(z_encoder,0);
}


void setup() {

  //mySerial.begin(AlctBaud, SERIAL_8N1 | SERIAL_INVERTED, RXD1,TXD1); //begin communication between ESP32 and Alicat regulators
  Serial.begin(ArdBaud);
  Serial.println("--------------");

  //encoder setup

  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), updateEncoder1, CHANGE);

  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B2), updateEncoder2, CHANGE);

  pinMode(ENCODER_Ax, INPUT_PULLUP);
  pinMode(ENCODER_Bx, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_Ax), updateEncoderx, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Bx), updateEncoderx, CHANGE);

  pinMode(ENCODER_Ay, INPUT_PULLUP);
  pinMode(ENCODER_By, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_Ay), updateEncodery, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_By), updateEncodery, CHANGE);

  // everything else

  // pinMode(Calibration_BUTTON, INPUT);

  //Serial.println("Position Move all pieces to be furthest from their stepper motors. Press the corresponding button for each axis when completed in order to calibrate.");
  pinMode(DIR_PIN_z1,OUTPUT);
  pinMode(STEP_PIN_z1,OUTPUT);
  
  pinMode(DIR_PIN_z2,OUTPUT);
  pinMode(STEP_PIN_z2,OUTPUT);

  pinMode(DIR_PIN_x,OUTPUT);
  pinMode(STEP_PIN_x,OUTPUT);

  pinMode(DIR_PIN_y,OUTPUT);
  pinMode(STEP_PIN_y,OUTPUT);

  calibrateMotors();

}

void loop() {
  // Serial.println(encoderPositionx);
  // Serial.println(encoderPositiony);
  // Serial.println(encoderPosition1);
  // Serial.println(encoderPosition2);
  // Serial.println("------------");
  // delay(100);
  if (!finishedFlag) {
      Serial.println("in");
      gen_and_collect();
      finishedFlag = true;
      Serial.println("done");
  }
}