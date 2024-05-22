#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#include <PID_v1.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;
Servo my_servo;

// High-pass filter variabelen
float alpha = 0.8; // Instelbare filterfactor
float previous_x, previous_y, previous_z;
float servo_pos = 90;

// PID variabelen

char receivedChar;
boolean newData = false;

unsigned long changeTime = 0; //last time the sensor was triggered
volatile unsigned long quarterSpins = 0;
unsigned long currentTime = millis();
double driverOut = 0;
double difference = 0;
double setPoint = 0;
String inString;
int Kp = 0;
int Ki = 0;
int Kd = 0;

PID myPID(&difference, &driverOut, &setPoint,Kp,Ki,Kd, DIRECT);

// Kalman-filter variabelen
double Q = 0.1; // Procesruis
double R = 1;   // Meetruis
double x = 0;   // Geschatte staat
double P = 1;   // Geschatte foutcovariantie
double K;       // Kalman-gain

// Functie voor het toepassen van een high-pass filter
float highPassFilter(float current_value, float previous_value) {
  return alpha * (previous_value + current_value);
}


void set(int pos, int val){
  int newer = val;
  int older = pos;
  if ( newer > older ){
    while ( newer > older ) {
      newer = newer - 1;
      my_servo.write(newer);
      delay(50);
    }
  }
  if ( older > newer ) {
    while ( older > newer ) {
      newer = newer + 1;
      my_servo.write(newer);
      delay(50);
    }
  }
}


// // Functie voor het toepassen van het Kalman-filter
// double kalmanFilter(double measurement) {
//     // Voorspelling
//     double x_pred = x;
//     double P_pred = P + Q;

//     // Update
//     K = P_pred / (P_pred + R);
//     x = x_pred + K * (measurement - x_pred);
//     P = (1 - K) * P_pred;

//     return x; // Retourneer de gefilterde waarde
// }

void setup() {
  Serial.begin(9600);
  Wire.begin();
  my_servo.attach(9);
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // Stop alles als er geen verbinding met MPU6050 is
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // Gyroscoop en accelerometer kalibreren
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  // Haal gyrowaarden op
  float gyro_x = mpu.getAngleX();
  float gyro_y = mpu.getAngleY();
  float gyro_z = mpu.getAngleZ();
  
  // Pas de high-pass filter toe op elke gyrowaarde
  float current_x = highPassFilter( gyro_x, previous_x );
  float current_y = highPassFilter( gyro_y, previous_y );
  float current_z = highPassFilter( gyro_z, previous_z );
  
//   // Pas het Kalman-filter toe op elke gefilterde gyrowaarde
//   double filtered_x = kalmanFilter(current_x);
//   double filtered_y = kalmanFilter(current_y);
//   double filtered_z = kalmanFilter(current_z);
    myPID.Compute();

    Serial.print("X : ");
    Serial.print(round(current_x/4));
    Serial.print("\tY : ");
    Serial.print(round(current_y/4));
    Serial.print("\tZ : ");
    Serial.print(round(current_z/4));
    Serial.print("\tPos : ");
    Serial.print(servo_pos);
    Serial.print("\out : ");
    Serial.println(driverOut);

  
  if (servo_pos < 180 && servo_pos > 0){
    if(round(driverOut)/4 > setPoint){
      set(servo_pos, servo_pos + difference);
      servo_pos = servo_pos + difference;
    }
    else if(round(driverOut)/4 < setPoint){
      set(servo_pos, servo_pos + difference);
      servo_pos = servo_pos + difference;
    }
  }

  difference = setPoint - round( driverOut )/4;
  driverOut = current_z;
  // Update vorige hoeken voor de volgende iteratie
  previous_x = current_x;
  previous_y = current_y;
  previous_z = current_z;
  delay(4);
}
