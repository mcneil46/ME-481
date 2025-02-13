// Included Libraries
#include <LiquidCrystal.h>
#include "HX711.h"

//Sets up the HX711 Load Cell, DAT = 4, CLK = 5
HX711 scale;
const long LOADCELL_OFFSET = 50682624;
const long LOADCELL_DIVIDER = 5895655;
const int HX_DAT = 4;
const int HX_CLK = 5;
float weight;
long scaleValue;


//AnalogPin to DigitalPin correlation
//A0 = 14, A1 = 15 ... A5 = 19

//sets up limit switch
int limit1 = 16;
int limit2 = 18;

//sets up hall effect sensor
int hallEffect = 2;
int magnet_on;

//defining buttons
const int button1Pin = 1; // constant button 1 pin -- known as the START button
const int button2Pin = 15; // constant button 2 pin -- known as the BACK button
const int button3Pin = 2; // constant button 3 pin -- known as the CHANGE UNITS button
int unitCycle = 0; //Used to keep track of button3Pin location

//declares rotary encoder
const int encPinA = 14; //pin A determines if there is a change in state
const int encPinB = 16; //pin B determines the direction
int encStateA; //stores current encoder position
int lastStateA = LOW; //determines if encoder moved or not (HIGH if moved)
int encDirection = 0;

//sets up the stepper motor
int driverPUL = 7;
int driverDIR = 6;
//Speed of motor. Lower = more torque but slower speed
int pd_fast = 1; //Equal to 300RPM
int pd_torque = 3; //Equal to 100RPM
//90RPM = Max torque, 810RPM = Max Speed
boolean setdir = LOW;
int motorDist = 0;

//Stepper Motor Functions
void motorMove(){
  while((magnet_on == LOW) && (limit1 && limit2 != LOW)){
    digitalWrite(driverDIR, setdir);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd_torque);
    digitalWrite(driverPUL, LOW);
    delayMicroseconds(pd_torque);
  }
}

void revMotor(){
  setdir != setdir;
}

void motorJog(int unitCycle){
  //Checks value of unit cycle. Moves motor in direction that is called and by distance unitCycle specifies
  //200 steps to complete one revolution, 1 revolution is 2mm travel

  if(unitCycle == 0){ //5mm Travel
  //the jog will only occur as long as a magnet is being read and the limit switches aren't on
    while((magnet_on == LOW) && (limit1 && limit2 != LOW)){
      for(int i = 0; i < 500; i++){
        digitalWrite(driverDIR, setdir);
        digitalWrite(driverPUL, HIGH);
        delayMicroseconds(pd_fast);
        digitalWrite(driverPUL, LOW);
        delayMicroseconds(pd_fast);
      }
    }
  }
  else if(unitCycle == 1){ //20mm Travel
    while((magnet_on == LOW) && (limit1 && limit2 != LOW)){
      for(int i = 0; i < 4000; i++){
        digitalWrite(driverDIR, setdir);
        digitalWrite(driverPUL, HIGH);
        delayMicroseconds(pd_fast);
        digitalWrite(driverPUL, LOW);
        delayMicroseconds(pd_fast);
      }
    }
  }
  else{//50mm Travel
    while((magnet_on == LOW) && (limit1 && limit2 != LOW)){
      for(int i = 0; i < 10000; i++){
        digitalWrite(driverDIR, setdir);
        digitalWrite(driverPUL, HIGH);
        delayMicroseconds(pd_fast);
        digitalWrite(driverPUL, LOW);
        delayMicroseconds(pd_fast);
      }
    }
  }
}

//History functions
//declares history array parameters
const int maxArraySize = 50;
String historyArray[maxArraySize];
int currentArraySize = 0;

//logs a run into the history array
void logData(String result){

}
//retrieves a result from the history array
void retrieveHistory(){

}
  
//sets up the LCD display
const int rs = 18, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//variables used for state machine
enum StrengthStates {STATE_IDLE, STATE_GO, STATE_JOG, STATE_HISTORY} ;

//declaration of array & integer used for state machine
StrengthStates stateArray[] = {STATE_IDLE, STATE_GO, STATE_JOG, STATE_HISTORY};
int arrayLocation; //will be used to count the step of the encoder which then coordinates to a state in the array
unsigned char strengthState;

void setup(){
    //Declares pins
    //Using INPUT_PULLUP instead of INPUT means that you are using the internal pull-up resistor (usually 20kOhm - 50kOhm) to the positive voltage. If you used INPUT instead, you rely on an external component (such as a resistor or switch)
    //to pull the input pin into a low state or provide a high state (by providing voltage)
    pinMode(button1Pin, INPUT_PULLUP);
    pinMode(button2Pin, INPUT_PULLUP);
    pinMode(button2Pin, INPUT_PULLUP);
  
  	//Declares Rotary Encoder
  	pinMode(encPinA, INPUT);
  	pinMode(encPinB, INPUT);

    //Starts LCD and sets up # of rows & columns
    lcd.begin(16,2);
    lcd.print("Hello, World!"); //Length of string MUST be less than 16 or will be cut off
  
    //Sets up Hall Effect sensor and attaches to pin, reads HIGH when is open and LOW when south pole of magnet is near
  	pinMode(hallEffect, INPUT_PULLUP);

    //Declares first state the program is in and the integer to accompany it
    strengthState = 0;
    arrayLocation = 0;

    //Sets up stepper motor
    pinMode(driverPUL, OUTPUT);
    pinMode(driverDIR, OUTPUT);

    //Sets up HX711 Load Cell
    scale.begin(HX_DAT, HX_CLK);
    scale.set_scale(LOADCELL_DIVIDER); //calibration factor that is experimentally determined

    //Attaches limit switches to interrupt pins 2 & 3
    attachInterrupt(digitalPinToInterrupt(limit1), revMotor, FALLING);
    attachInterrupt(digitalPinToInterrupt(limit2), revMotor, FALLING);

    //To Diagnose code behaviour
    Serial.begin(9600);

    //Returns back calibration factor from load cell
    Serial.println(calibration());

    //tares scale and gets it ready for testing
    scale.tare();
}

void loop(){
  switch(strengthState){
    case STATE_IDLE:
      lcd.print("Select an option");
      encStateA = digitalRead(encPinA);
      //checks if encoder moved
      if(encStateA != lastStateA){
        //If encoder moved, checks the direction it moved in
        if(digitalRead(encPinB) != encDirection){
          //If B is different from A, clockwise movement
          //checks int of array location. If larger than length of array (Array length = 4 but 3 in coding terms), moves it back to zero
          if(arrayLocation < 3){
            arrayLocation++;
          }
          else{
            arrayLocation = 0;
          }
        }
        else{
          if(arrayLocation == 0){
            arrayLocation = 3;
          }
          else{
            arrayLocation --;
          }
        }
      }

      //reads value of arrayLocation and prints out the respective menu
      if(arrayLocation == 0){
        lcd.print("SELECT MENU");
      }
      else if(arrayLocation == 1){
        lcd.print("START TEST");
      }
      else if(arrayLocation == 2){
        lcd.print("JOG MOTOR");
      }
      else{
        lcd.print("CHECK HISTORY");
      }

      //checks if the "Start" button has been pressed
      if(button1Pin == HIGH){
        strengthState = stateArray[arrayLocation];
      }
      break;
    //If button is pressed in STATE_GO, the hall effect is checked so that the door is closed. If closed, motorMove() function is begun
    case STATE_GO:
      scale.power_up(); //Turns the ADC on and prepares it for test

      lcd.println("START TEST MENU");
      lcd.print("PUSH START");
      //Checks if "start" button has been pressed
      if(button1Pin == HIGH){
        lcd.clear();
        //if button pressed, check if Hall effect senses magnet
        if(digitalRead(magnet_on) == LOW){
          //moves motor using motorMove() function
          motorMove();
        }
        else{
          //Informs user the door isn't closed, delays then goes back to start of state
          lcd.println("CLOSE DOOR");
          lcd.print("AND TRY AGAIN");
          delay(1000);
        }
      }
      scale.power_down(); //Puts ADC into sleep mode

      break;
    //STATE_JOG will take a look at the encoder. Button 3 will change distance travelled
    case STATE_JOG:
      lcd.println("PRESS UNIT TO");
      lcd.println("CHANGE DISTANCE");
      //If button 3 pressed, unit cycle then moves up incrementally
      if(digitalRead(button3Pin) == HIGH){
        lcd.clear();
        unitCycle++;
      }
      //Checks value of unit cycle then prints out travel of motor
      if(unitCycle == 1){
        lcd.clear();
        lcd.println("20MM TRAVEL");
      }
      else if(unitCycle == 2){
        lcd.clear();
        lcd.println("50MM TRAVEL");
      }
      else{
        lcd.clear();
        unitCycle = 0;
        lcd.println("5MM TRAVEL");
      }


      if(encStateA != lastStateA){
        //If encoder moved, checks the direction it moved in
        if(digitalRead(encPinB) != encDirection){
          //If B is different from A, clockwise movement
          //motorJog function called and the unitCycle variable is passed through
          motorJog(unitCycle);
        }
        else{
          revMotor();
          motorJog(unitCycle);
        }
      }

      break;
    case STATE_HISTORY:
      lcd.print("HISTORY MENU");
      break;
  }

  
}

//HX711 Functions
//calibrates the load cell
long calibration(){
  scaleValue = scale.read();
  return scaleValue;
}

float getWeight(){
  weight = scale.get_units(5);
  return weight;
}
