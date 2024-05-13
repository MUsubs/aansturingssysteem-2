#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;
Servo myservo;

// High-pass filter variabelen
float alpha = 0.8; // Instelbare filterfactor
float previousX, previousY, previousZ;
float servoPos = 90;


// Kalman-filter variabelen
double Q = 0.1; // Procesruis
double R = 1;   // Meetruis
double x = 0;   // Geschatte staat
double P = 1;   // Geschatte foutcovariantie
double K;       // Kalman-gain

// Functie voor het toepassen van een high-pass filter
float highPassFilter(float currentValue, float previousValue) {
  return alpha * (previousValue + currentValue);
}


void set(int pos, int val){
  int newer = val;
  int older = pos;
  if ( newer > older ){
    while ( newer > older ) {
      newer = newer - 1;
      myservo.write(newer);
      delay(50);
    }
  }
  if ( older > newer ) {
    while ( older > newer ) {
      newer = newer + 1;
      myservo.write(newer);
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
  Serial.begin(115200);
  Wire.begin();
  myservo.attach(9);
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
  float gyroX = mpu.getAngleX();
  float gyroY = mpu.getAngleY();
  float gyroZ = mpu.getAngleZ();
  
  // Pas de high-pass filter toe op elke gyrowaarde
  float currentX = highPassFilter(gyroX, previousX);
  float currentY = highPassFilter(gyroY, previousY);
  float currentZ = highPassFilter(gyroZ, previousZ);
  
//   // Pas het Kalman-filter toe op elke gefilterde gyrowaarde
//   double filteredX = kalmanFilter(currentX);
//   double filteredY = kalmanFilter(currentY);
//   double filteredZ = kalmanFilter(currentZ);

    Serial.print("X : ");
    Serial.print(round(currentX/4));
    Serial.print("\tY : ");
    Serial.print(round(currentY/4));
    Serial.print("\tZ : ");
    Serial.print(round(currentZ/4));
    Serial.print("\tPos : ");
    Serial.println(servoPos);

    delay(50);
  
  if (servoPos < 180 && servoPos > 0){
    if(round(currentZ)/4 > 0){
      set(servoPos, servoPos - 1);
      servoPos = servoPos - 1;
    }
    else if(round(currentZ)/4 < 0){
      set(servoPos, servoPos + 1);
      servoPos = servoPos + 1;
    }
  }

  // Update vorige hoeken voor de volgende iteratie
  previousX = currentX;
  previousY = currentY;
  previousZ = currentZ;
  delay(50);
}
