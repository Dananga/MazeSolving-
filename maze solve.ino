// IR Sensors
#define LS 7  // left sensor
#define MS1 6  // middle sensor 1
#define MS2 5  // middle sensor 2
#define RS 4   // right sensor 

#define LM1 12     // left motor pin 1
#define LM2 13     // left motor pin 2
#define RM1 8     // right motor pin 1
#define RM2 9     // right motor pin 2
#define LMpwm 11  // left motor speed control
#define RMpwm 10  //right motor speed control

#define led_LS A5
#define led_MS1 A4
#define led_MS2 A3
#define led_RS A2

#define led1 A0 
#define led2 A1


int sensor[4] = {0, 0, 0, 0};

int motor_spd = 150; //default motor speed


float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;


String path = "";  // for store solved maze
int mazeOk = 0;

void setup() {
  pinMode(LS, INPUT);
  pinMode(MS1, INPUT);
  pinMode(MS2, INPUT);
  pinMode(RS, INPUT);

  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(LMpwm, OUTPUT);
  pinMode(RMpwm, OUTPUT);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led_LS, OUTPUT);
  pinMode(led_MS1, OUTPUT);
  pinMode(led_MS2, OUTPUT);
  pinMode(led_RS, OUTPUT);

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);

  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
  Serial.println("Started !!");
  delay(1000);
}
void loop() {
  read_sensors();
  if (error == 100) {               // left
    do {
      read_sensors();
      analogWrite(LMpwm, 110); //Left Motor Speed
      analogWrite(RMpwm, 90); //Right Motor Speed
      sharpLeftTurn();
    } while (error != 0);
  } else if (error == 101) {          // right
    analogWrite(LMpwm, 110); //Left Motor Speed
    analogWrite(RMpwm, 90); //Right Motor Speed
    forward();
    delay(200);
    stp();
    read_sensors();
    if (error == 102) {
      do {
        analogWrite(LMpwm, 110); //Left Motor Speed
        analogWrite(RMpwm, 90); //Right Motor Speed
        sharpRightTurn();
        read_sensors();
      } while (error != 0);
    }
  } else if (error == 102) {        // Make left turn untill it detects straight path
    //Serial.print("\t");
    //Serial.println("Sharp Left Turn");
    path.concat("U");
    do {
      analogWrite(LMpwm, 110); //Left Motor Speed
      analogWrite(RMpwm, 90); //Right Motor Speed
      sharpLeftTurn();
      read_sensors();
      if (error == 0) {
        stp();
        delay(200);
      }
    } while (error != 0);
  } else if (error == 103) {        // Make left turn untill it detects straight path or stop if dead end reached.
    if (mazeOk == 0) {
      analogWrite(LMpwm, 110); //Left Motor Speed
      analogWrite(RMpwm, 90); //Right Motor Speed
      forward();
      delay(200);
      stp();
      read_sensors();
      if (error == 103) {    //stop
        stp();
        digitalWrite(led1, HIGH);
        digitalWrite(led2, HIGH);
        mazeOk = 1;
      } else {       //left
        analogWrite(LMpwm, 110); //Left Motor Speed
        analogWrite(RMpwm, 90); //Right Motor Speed
        path.concat("L");
        sharpLeftTurn();
        delay(200);
        do {
          read_sensors();
          analogWrite(LMpwm, 110); //Left Motor Speed
          analogWrite(RMpwm, 90); //Right Motor Speed
          sharpLeftTurn();
        } while (error != 0);
      }
    } else {
      for (int i = 0; i < path.length(); i++ ) {
        int r = path[i];
        if (r == "S") {
          forward();
        } else if (r == "L") {
          do {
            read_sensors();
            analogWrite(LMpwm, 110); //Left Motor Speed
            analogWrite(RMpwm, 90); //Right Motor Speed
            sharpLeftTurn();
          } while (error != 0);
        }
        else if (r == "R") {
          do {
            read_sensors();
            analogWrite(LMpwm, 90); //Left Motor Speed
            analogWrite(RMpwm, 110); //Right Motor Speed
            sharpRightTurn();
          } while (error != 0);
        }
      }
    }
  } else {
    calc_pid();
    motor_control();
  }
  path.replace("LUL", "S");
  path.replace("SUL", "R");
  path.replace("SRUL", "R");

}


void read_sensors() {
  sensor[0] = digitalRead(LS);
  sensor[1] = digitalRead(MS1);
  sensor[2] = digitalRead(MS2);
  sensor[3] = digitalRead(RS);

  digitalWrite(led_LS, sensor[0]);
  digitalWrite(led_MS1, sensor[1]);
  digitalWrite(led_MS2, sensor[2]);
  digitalWrite(led_RS, sensor[3]);
  

  error = ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) ?   3 :
          ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0)) ?   2 :
          ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0)) ?   1 :    //forward with error correction(to right)
          ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) ?   0 :    //forward
          ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0)) ?  -1 :    //forward with error correction(to left)
          ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1)) ?  -2 :
          ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1)) ?  -3 :
          ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) ? 100 :    //left turn  1111
          ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) ? 101 :    // right turn 0111
          ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) ? 102 :    //u turn 0000
          ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) ? 103 : 4; //turn left or stop
}

void calc_pid() {
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{

  int left_motor_speed = motor_spd - PID_value;
  int right_motor_speed = motor_spd + PID_value;

  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(LMpwm, left_motor_speed); //Left Motor Speed
  analogWrite(RMpwm, right_motor_speed - 30); //Right Motor Speed
  forward();
}

void forward() {
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}
void reverse() {
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}
void right() {
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
}
void left() {
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}
void sharpRightTurn() {
    digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}
void sharpLeftTurn() {

  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}
void stp() {
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
}
