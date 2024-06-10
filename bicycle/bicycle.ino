#include "Wire.h"
#include "I2Cdev.h"
#include "math.h"
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Arduino.h>

Servo servo;
// MPU6050 nesnesi oluştur
Adafruit_MPU6050 mpu;

#define Kp  300
#define Kd  0
#define Ki  30
#define sampleTime  0.002
#define targetAngle 1
//-2.5di

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;
unsigned long last_time = 0;

unsigned long timeControl = 0;

void setup() {
  // Seri haberleşmeyi başlat
  Serial.begin(115200);
  
  // I2C haberleşmesini başlat
  Wire.begin(21, 22);

  servo.attach(4);

  servo.write(0);
  // MPU6050'i başlat
  if (!mpu.begin()) {
    Serial.println("MPU6050 başlatılamadı! Bağlantıları kontrol edin.");
    while (1);
  }
  Serial.println("MPU6050 başarıyla başlatıldı!");

  // Sensörün veri hızı ve aralık ayarlarını yap (isteğe bağlı)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  delay(100);

}

void loop() {
  //TO DO sample time bul
  //Serial.println(millis() - timeControl);
  // timeControl = millis();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
   // read acceleration and gyroscope values
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;

    // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);   //current angleyi yazdırmayı dene
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
 // errorSum = constrain(errorSum, -300, 300);    
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;

  
  //motorPower = constrain(motorPower, 0, 180);
  servo.write(map(motorPower, -900, 900, 0,180));
  Serial.println(map(motorPower, -900, 900, 0,180));
  //Serial.println(".");

}



