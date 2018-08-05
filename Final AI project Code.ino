//---------------------------------------------QTRC Sensor--------------------------------------------------------------

#include<QTRSensors.h>

#define NUM_SENSORS 7
#define TIMEOUT     2500

QTRSensorsRC qtrrc ((unsigned char[]) {
  2, 3, 4, 7, 8, 11, 12
}, NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);

unsigned int position;

unsigned int sensorValues[NUM_SENSORS];
unsigned int sensorValues1[NUM_SENSORS];
unsigned int sensorValues2[NUM_SENSORS];
unsigned int sensorValues3[NUM_SENSORS];

unsigned int previousValues[NUM_SENSORS];

unsigned int sensor_sum[NUM_SENSORS];
unsigned int sensor_avg[NUM_SENSORS];

int trig1 = 22;
int echo1 = 23;

int trig2 = 24;
int echo2 = 25;

int distance1;
int distance2;

unsigned long duration1;
unsigned long duration2;

double location;
double location1;
double location2;

int KP = 800;
double error_distance ;

int right_motor1 = 9;
int right_motor2 = 10;
int right_motor_speed1;
int right_motor_speed2;

int left_motor1 = 5;
int left_motor2 = 6;
int left_motor_speed1;
int left_motor_speed2;


int i, j, k, x, y, c, d ;
int l = 0;
int Line;
int angle = 0;

int error;
int error1;
int error2;
int error3;
int previous_error;
//.23
float kp = .023;
int avarage_speed = 120; //150
int maximum_speed = 255;
int error_in_speed;

void setup() {

  Serial.begin(9600);

  pinMode (trig1, OUTPUT);
  pinMode (echo1, INPUT);

  pinMode (trig2, OUTPUT);
  pinMode (echo2, INPUT);

  pinMode(right_motor1, OUTPUT);
  pinMode(right_motor2, OUTPUT);
  pinMode(left_motor1, OUTPUT);
  pinMode(left_motor2, OUTPUT);

  pinMode(13, OUTPUT);


  delay(500);

  digitalWrite(13, HIGH);

  for ( int i = 0; i < 400; i++) {
    qtrrc.calibrate();
  }

  digitalWrite(13, LOW);
}


void loop() {

  calc_error();
  save_values();
  motor_control();

}

void scan_line () {

  if (Line == 0) {
    position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 1);
    x = 0;
    y = 1000;    

  }

  else {
    position = qtrrc.readLine(sensorValues);
    x = 1000;
    y = 0;
  
  }
  
  for (unsigned char i  = 0; i < 7 ; i++ ){
    if (sensorValues[i] < 300 ) sensorValues[i] = 0 ;
    else sensorValues[i] = 1000;
  }

}


void calc_distance() {

  digitalWrite (trig2, LOW);
  delayMicroseconds (2);
  digitalWrite (trig2, HIGH);
  delayMicroseconds (10);

  duration2 = pulseIn (echo2, HIGH, 60000);
  distance2 = duration2 * 0.036 / 2 ;

  delay(1);

  digitalWrite (trig1, LOW);
  delayMicroseconds (2);
  digitalWrite (trig1, HIGH);
  delayMicroseconds (10);

  duration1 = pulseIn (echo1, HIGH, 60000);
  distance1 = duration1 * 0.036 / 2 ;

  delay(1);

}

void calc_distance_error() {

  location1 = (distance1 * 0 + distance2 * 1) ;
  location2 = (distance1 + distance2);
  location = location1 / location2 ;
  error_distance = 0.5 - location;
  error_in_speed = KP * error_distance;
}


void go_forward () {

  digitalWrite (13, HIGH);
  avarage_speed = 220;

do {
      calc_distance();
      calc_distance_error();
      motor_control();

      scan_line ();

      error = 3000 - position ;
      error1 = abs(error1);
      error_in_speed = kp * error1 ;

    } while ((sensorValues[3] != x));

  digitalWrite (13, LOW);
  avarage_speed = 100;

}


void right () {

  digitalWrite (13, HIGH);

  do {

    if ( (sensorValues[0] == x) && (sensorValues[6] == x) )  {

      if (angle < 5) {
        left ();
        angle = 6;
      }

      else {};

      c = 0;
    }

    else c = 1;

    analogWrite (right_motor1, 80);
    analogWrite (right_motor2, 0);
    analogWrite (left_motor1, 80);
    analogWrite (left_motor2, 0);

    scan_line ();

  } while ( sensorValues[6] != y );

  if (c == 1 ) {

    do {

      analogWrite (right_motor1, 0);
      analogWrite (right_motor2, 120);
      analogWrite (left_motor1, 120);
      analogWrite (left_motor2, 0);

      scan_line ();
      error = 3000 - position;
      error_in_speed = kp * error;

    } while ( sensorValues[6] != x );
  }

  else {};

  digitalWrite (13, LOW);

}


void left () {

  digitalWrite (13, HIGH);

  do {

    if ( (sensorValues[0] == x) && (sensorValues[6] == x) ) {
      d = 0;
    }
    else {
      d = 1;
    }

    analogWrite (right_motor1, 80);
    analogWrite (right_motor2, 0);
    analogWrite (left_motor1, 80);
    analogWrite (left_motor2, 0);

    scan_line ();

  } while ( sensorValues[0] != y );

  if (d == 1 ) {

    do {

      analogWrite (right_motor1, 120);
      analogWrite (right_motor2, 0);
      analogWrite (left_motor1, 0);
      analogWrite (left_motor2, 120);

      scan_line ();
      error = 3000 - position;
      error_in_speed = kp * error;

    } while ( sensorValues[0] != x );

  }

  else {};

  digitalWrite (13, LOW);

}


void turn_right () {

  do {
    
    if ( (sensorValues[0] == x) && (sensorValues[6] == x) )  {

      if (angle < 5) {
        left ();
        angle = 6;
      }

      else {};

      c = 0;
    }

    else c = 1;

    analogWrite (right_motor1, 80);
    analogWrite (right_motor2, 0);
    analogWrite (left_motor1, 80);
    analogWrite (left_motor2, 0);

    scan_line ();

  } while ( sensorValues[6] != y );

  if (c == 1 ) {

  do {

    analogWrite (right_motor1, 0);
    analogWrite (right_motor2, 120);
    analogWrite (left_motor1, 120);
    analogWrite (left_motor2, 0);

    scan_line ();
    error = 3000 - position;
    error_in_speed = kp * error;

  } while ( sensorValues[6] != x );
  }

  else {};

}


void turn_left () {

  do {

    
        if ( (sensorValues[0] == x) && (sensorValues[6] == x) ) {
      d = 0;
    }
    else {
      d = 1;
    }

    analogWrite (right_motor1, 80);
    analogWrite (right_motor2, 0);
    analogWrite (left_motor1, 80);
    analogWrite (left_motor2, 0);

    scan_line ();

  } while ( sensorValues[0] != y);

  if (d == 1 ) {

  do {

    analogWrite (right_motor1, 120);
    analogWrite (right_motor2, 0);
    analogWrite (left_motor1, 0);
    analogWrite (left_motor2, 120);

    scan_line ();
    error = 3000 - position;
    error_in_speed = kp * error;

  } while ( sensorValues[0] != x );
  }

  else {};

}


void calc_error () {


  while ( l < 1 ) {

    position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 1);

    if ( (sensorValues[0] <= 100) && (sensorValues[6] <= 100) ) Line = 1 ;
    else Line = 0;

    l++;
  }


  scan_line ();


  if ((sensorValues[0] != y ) && ( sensorValues[3] != x )  && (sensorValues[6] != y )) {

    if (Line == 0) {
      position = qtrrc.readLine(sensorValues);
      Line = 1;
      x = 1000;
      y = 0;
    }

    else {
      position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 1);
      Line = 0;
      x = 0;
      y = 1000;
    }
  }


  error = 3000 - position ;
  error2 = abs(error);


  if ( (sensorValues[0] == x) && (sensorValues[6] == x) )  {

    if ( (angle < 6) ) {
      left ();
      angle = 6;
    }
    
    else error_in_speed = kp * error;

    angle ++;
  }

  if ( (sensorValues[0] == x) && (sensorValues[1] == y ) && (sensorValues[3] == x) ) {
    turn_left ();
    angle++;
  }

  if ( (sensorValues[3] == x) && (sensorValues[5] == y) && (sensorValues[6] == x ) ) {
    turn_right ();
    angle++;
  }

  if ( (sensorValues[0] == x) && (sensorValues[4] == x ) ) {
    left ();
    angle++;
  }

  if ( (sensorValues[2] == x) && (sensorValues[6] == x ) ) {
    right ();
    angle++;
  }

  if ( (previous_error <= 900) && (error2 == 3000) ) go_forward ();

//  if (  (previousValues[0] == y) && (previousValues[2] == x) && (previousValues[4] == x) && (sensorValues[6] == y) && (sensorValues[2] == y) && (sensorValues[4] == y ) ) go_forward ();

  else error_in_speed = kp * error ;

}

void save_values() {

  error = abs (error);

//    if ( error == 0 && previous_error == 0 ) {
//      avarage_speed = avarage_speed + 5 ;
//    }
//    
//    else avarage_speed = 80 ;

  previous_error = error;
  
    for (unsigned char i = 0; i < NUM_SENSORS; i++) {
    previousValues[i]  =  sensorValues[i];
  }

}

void motor_control() {

  right_motor_speed1 = avarage_speed + error_in_speed;
  right_motor_speed2 = 0;
  left_motor_speed1 = avarage_speed - error_in_speed;
  left_motor_speed2 = 0;

  if (left_motor_speed1 > 255) left_motor_speed1 = 255;
  if (right_motor_speed1 > 255) right_motor_speed1 = 255;
  if (right_motor_speed1 < 0) right_motor_speed1 = 0;
  if (left_motor_speed1 < 0) left_motor_speed1 = 0;

  analogWrite (right_motor1, right_motor_speed1);
  analogWrite (right_motor2, 0);
  analogWrite (left_motor1, left_motor_speed1);
  analogWrite (left_motor2, 0);

}