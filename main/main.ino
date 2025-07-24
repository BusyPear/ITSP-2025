// #include <ESP32Servo.h>
// #include <PID_v1.h>
#include <Wire.h>
// #include <SPI.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// Filter variables
const int num_avg = 5;

// BMP initialization
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)
double alt[num_avg];

// IMU initialiation
#define SDA_PIN 21
#define SCL_PIN 22
MPU9250_asukiaaa mySensor;
double accel_offset_X = 0, accel_offset_Y = 0, accel_offset_Z = 0;
double gyro_offset_X = 0, gyro_offset_Y = 0, gyro_offset_Z = 0;
double accel_X[num_avg], accel_Y[num_avg], accel_Z[num_avg];
double gyro_X[num_avg], gyro_Y[num_avg], gyro_Z[num_avg];
double angle_X[num_avg], angle_Y[num_avg], angle_Z[num_avg];

// EDF initialization
// Servo esc;
// int esc_pin = 32;

unsigned long t = 0;

void swap(double& a, double& b){
  double temp;
  temp = a; a = b; b = temp;
}

void setupBMP(){
  Serial.println("Connecting to BMP3xx...");

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3xx sensor, check wiring!");
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("BMP3xx sensor found!");
}

double getAltitude(){
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return -1;
  }

  alt[0] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  if(alt[num_avg-1] != 0){
    // alt[0] = (alt[0]+alt[1]+alt[2]+alt[3]+alt[4])/5;
    for(int i=1; i<num_avg; i++){
      alt[0] += alt[i];
    }
    alt[0] /= num_avg;
  }
  Serial.print("Altitude = ");
  Serial.println(alt[0]);
  // swap(alt1, alt2); swap(alt2, alt3); swap(alt3, alt4); swap(alt5, alt4);
  for(int i=(num_avg-1); i>=1; i--){
    swap(alt[i], alt[i-1]);
  }
  return alt[0];
}

void setupIMU(){
  Serial.println("Connecting to MPU9250...");

  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();

  Serial.println("MPU9250 (Accel + Gyro separately) found!");
  
}

void calibIMU(){
  const int num_samples = 500;
  double ax = 0, ay = 0, az = 0;
  double wx = 0, wy = 0, wz = 0;

  for(int i=0; i<num_samples; i++){
    mySensor.accelUpdate();
    mySensor.gyroUpdate();

    ax += mySensor.accelX();
    ay += mySensor.accelY();
    az += mySensor.accelZ();

    wx += mySensor.gyroX();
    wy += mySensor.gyroY();
    wz += mySensor.gyroZ();

    delay(5);
  }

  accel_offset_X = ax / num_samples;
  accel_offset_Y = ay / num_samples;
  accel_offset_Z = (az / num_samples); // Subtract 1g for gravity

  gyro_offset_X = wx / num_samples;
  gyro_offset_Y = wy / num_samples;
  gyro_offset_Z = wz / num_samples;
}

// Axis = Up -> Y; Right -> x
void getAccel(){
  mySensor.accelUpdate();

  // Apply moving average on accel values
  {
    // Update accel_X
    accel_X[0] = mySensor.accelX() - accel_offset_X;
    if(accel_X[num_avg-1] != 0){
      for(int i=1; i<num_avg; i++){
        accel_X[0] += accel_X[i];
      }
      accel_X[0] /= num_avg;
    }
    // Update accel_Y
    accel_Y[0] = mySensor.accelY() - accel_offset_Y;
    if(accel_Y[num_avg-1] != 0){
      for(int i=1; i<num_avg; i++){
        accel_Y[0] += accel_Y[i];
      }
      accel_Y[0] /= num_avg;
    }
    // Update accel_Z
    accel_Z[0] = mySensor.accelZ() - accel_offset_Z;
    if(accel_Z[num_avg-1] != 0){
      for(int i=1; i<num_avg; i++){
        accel_Z[0] += accel_Z[i];
      }
      accel_Z[0] /= num_avg;
    }
  }

  Serial.println("[ACCELERATION]");
  Serial.print("X: "); Serial.print(accel_X[0]);
  Serial.print(" Y: "); Serial.print(accel_Y[0]);
  Serial.print(" Z: "); Serial.println(accel_Z[0]);
  
  // Swap accel values for moving average
  {
    // Swap accel_X values
    for(int i=(num_avg-1); i>=1; i--){
      swap(accel_X[i], accel_X[i-1]);
    }
    // Swap accel_Y values
    for(int i=(num_avg-1); i>=1; i--){
      swap(accel_Y[i], accel_Y[i-1]);
    }
    // Swap accel_Z values
    for(int i=(num_avg-1); i>=1; i--){
      swap(accel_Z[i], accel_Z[i-1]);
    }
  }
}

void getGyro(){
  mySensor.gyroUpdate();

  // Apply moving average on gyro values
  {
    // Update gyro_X
    gyro_X[0] = mySensor.gyroX() - gyro_offset_X;
    if(gyro_X[num_avg-1] != 0){
      for(int i=1; i<num_avg; i++){
        gyro_X[0] += gyro_X[i];
      }
      gyro_X[0] /= num_avg;
    }
    // Update gyro_Y
    gyro_Y[0] = mySensor.gyroY() - gyro_offset_Y;
    if(gyro_Y[num_avg-1] != 0){
      for(int i=1; i<num_avg; i++){
        gyro_Y[0] += gyro_Y[i];
      }
      gyro_Y[0] /= num_avg;
    }
    // Update gyro_Z
    gyro_Z[0] = mySensor.gyroZ() - gyro_offset_Z;
    if(gyro_Z[num_avg-1] != 0){
      for(int i=1; i<num_avg; i++){
        gyro_Z[0] += gyro_Z[i];
      }
      gyro_Z[0] /= num_avg;
    }
  }

  Serial.println("[GYROSCOPE]");
  Serial.print("X: "); Serial.print(gyro_X[0]);
  Serial.print(" Y: "); Serial.print(gyro_Y[0]);
  Serial.print(" Z: "); Serial.println(gyro_Z[0]);

  // Swap gyro values for moving average
  {
    // Swap gyro_X values
    for(int i=(num_avg-1); i>=1; i--){
      swap(gyro_X[i], gyro_X[i-1]);
    }
    // Swap gyro_Y values
    for(int i=(num_avg-1); i>=1; i--){
      swap(gyro_Y[i], gyro_Y[i-1]);
    }
    // Swap gyro_Z values
    for(int i=(num_avg-1); i>=1; i--){
      swap(gyro_Z[i], gyro_Z[i-1]);
    }
  }
}

// angle_M means angle of rocket from positive(?) M axis
// Say angle_X = yaw and angle_Y = pitch, of course angle_Z = roll
void getAngle(){
  // TODO - Sensor fusion from Accel and Gyro

  // Below variable defines how significant Accel data is in calculating angle(0 to 1)
  const int weight_accel = 0.5; // 1 means Gyro data is irrelevent, 0 means Accel data is irrelevent, 0.5 means both are equally significant

  // Calculating angle_X
  
}

// void setupESC(){
//   esc.attach(esc_pin);
//   esc.writeMicroseconds(1000);
//   delay(5000);
// }

// void poweredAscent(){
//   t = millis();
//   if(t <= 4){
//     esc.writeMicroseconds(1800);
//   }
//   else if(t <= 5){
//     esc.writeMicroseconds(1800);
//   }
//   else{
//    esc.writeMicroseconds(1000); 
//   }
// }

void setup() {
  Serial.begin(115200);
  delay(5000);
  while(!Serial);
  // setupBMP();
  setupIMU();
  calibIMU();
}

void loop() {
  // getAltitude();
  getAccel();
  getGyro();
  delay(100);
}
