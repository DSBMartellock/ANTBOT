#include <Encoder.h>
#include <Servo.h>

//ULTRASONIC VARIABLES
int trigPin = 34;    // Trigger
int echoPin = 35;    // Echo
long duration, cm, inches;

//IR Sensor:
//tcrt-5000L
int irLEDPin = 22;
int irPin = A15;
int a,b,c;

// CALIBRATION-!-!-!-!-!-!-!-!------------------------------------------------------------------
// Threshold for Black Detection:
#define black_detect 20 //if the value drops below 20, probably detected a line. 

// ---------------------------------------------------------------------------------------------


Encoder rt_wheel_encoder(2,19);        //Arduino Mega interrupt pins - 2, 3, 18, 19, 20, 21
long right_encoder;

Servo claw_servo;                     // create servo object to control a servo
int claw_control = 28;
Servo left_wheel_servo;               // create servo object to control a servo
int wheel_control = 30 ;

long left_wheel_pos = 95;               // variable to store the servo position


int claw_pos = 100;

#define claw_neutral 100 
#define claw_open 0
#define claw_closed 160


// Motor Variables -------------------------------------------------------------------------------

// variable to store right encoder posistion
int rt_wheel_pos = -999;

// wired connections
#define HG7881_B_IA 4 // D10 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IB 3 // D11 --> Motor B Input B --> MOTOR B -
 
// functional connections
#define MOTOR_B_PWM HG7881_B_IA // Motor B PWM Speed
#define MOTOR_B_DIR HG7881_B_IB // Motor B Direction
 
// the actual values for "fast" and "slow" depend on the motor
#define PWM_SLOW 100  // arbitrary slow speed PWM duty cycle
#define PWM_FAST 500 // arbitrary fast speed PWM duty cycle
#define DIR_DELAY 100 // brief delay for abrupt motor changes

// ------------------------------------------------------------------------------------------------

void setup() {
  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irLEDPin, OUTPUT);

  //Motor
  pinMode( MOTOR_B_DIR, OUTPUT );
  pinMode( MOTOR_B_PWM, OUTPUT );
  digitalWrite( MOTOR_B_DIR, LOW );
  digitalWrite( MOTOR_B_PWM, LOW );
  
  claw_servo.attach(claw_control); 
  left_wheel_servo.attach(wheel_control);

  Serial.begin(9600);

}

void loop() {

  //INITIAL SETTINGS:
  claw_servo.write(claw_closed); 
  forward(500);

  while(1){
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
   
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    duration = pulseIn(echoPin, HIGH);
   
    // Convert the time into a distance
    inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
      
    digitalWrite(irLEDPin,HIGH);    // Turning ON LED
    delayMicroseconds(500);  //wait
    a=analogRead(irPin);        //take reading from photodiode(pin A15) :noise+signal
    digitalWrite(irLEDPin,LOW);     //turn Off LED
    delayMicroseconds(500);  //wait
    b=analogRead(irPin);        // again take reading from photodiode :noise
    c=a-b;                    //taking differnce:[ (noise+signal)-(noise)] just signal
  
    right_encoder = rt_wheel_encoder.read();
  
    Serial.print(inches);
    Serial.print("in, ");
    Serial.print(c);
    Serial.print(" IR reading, ");
    Serial.print(right_encoder);
    Serial.print(" Encoder reading");
    Serial.println();
    delay(250);


    if(c >= -black_detect){
      Serial.print("Black detected!");
      line_detected();
    }
    else{
      forward(500);  
    }
   
    
  }
  
  


 // Program that moves the left_wheel_servo the same number of degrees as the encoder moves. 
// long new_rt;
//
//  new_rt = rt_wheel_encoder.read();
//
//
//  if(new_rt != rt_wheel_pos){
//    //add the amount of change from the old posistion to the new posistion 
//
//    left_wheel_pos = rt_wheel_pos - new_rt; 
//    rt_wheel_pos = new_rt;
//    Serial.println("Right wheel posistion ");
//    Serial.println(new_rt);
//    Serial.println("Left Servo Posistion");
//    Serial.println(left_wheel_pos);
//    
//  }
//  else{
//    left_wheel_pos = 95.9;
//  }
//  left_wheel_servo.write(left_wheel_pos); 
//
//  delay(15);



  
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    claw_servo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                          // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    claw_servo.write(pos);              // tell servo to go to position in variable 'pos'
//    
//    left_wheel_servo.write(pos);
//    delay(15);                          // waits 15ms for the servo to reach the position
//  }

}

void line_detected(){
  full_stop();
  reverse(PWM_FAST);
  delay(1000);
  full_stop();
  delay(500);
  left_turn(90);
  delay(2500);
  
}


void full_stop(){
    motor_hard_stop();
    servo_stop();
}

void forward(int Speed){
  servo_forward();
  motor_forward(Speed);
}

void reverse(int Speed){
  servo_reverse();
  motor_reverse(500);  
}

void left_turn(int degrees){
  motor_forward(500);
  servo_reverse();
}

void right_turn(int degrees){
  motor_reverse(500);
  servo_forward();
}


//MOTOR
void motor_forward(int Speed){
    // set the motor speed and direction
    digitalWrite( MOTOR_B_DIR, LOW ); // 
    analogWrite( MOTOR_B_PWM, Speed); // 
}

void motor_reverse(int Speed){
    // set the motor speed and direction
    digitalWrite( MOTOR_B_DIR, HIGH ); // direction = reverse
    analogWrite( MOTOR_B_PWM, 255-Speed ); //
}

void motor_soft_stop(){
    digitalWrite( MOTOR_B_DIR, LOW );
    digitalWrite( MOTOR_B_PWM, LOW );
}

void motor_hard_stop(){
    digitalWrite( MOTOR_B_DIR, HIGH );
    digitalWrite( MOTOR_B_PWM, HIGH );
}


//SERVO
void servo_forward(){
  // set servo speed and direction
  left_wheel_servo.write(180);
}

void servo_reverse(){
  // set servo speed and direction
  left_wheel_servo.write(0);
}

void servo_stop(){
  left_wheel_servo.write(left_wheel_pos);
}


//SENSORS
long filter_encoder(int encoder_reading){
  return encoder_reading;
}
