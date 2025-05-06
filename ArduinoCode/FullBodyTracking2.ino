#include<Wire.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>

MPU6050 mpu[6];  // Array of 6 MPU6050 objects

SoftwareSerial BTserial(2, 3); // RX | TX

// Digital pins controlling the AD0 pin of each MPU (Pins 2 to 7)
int AD0_pins[] = {4, 5, 6, 7, 8, 9};

const int MPU6050_addr=0x69;

int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;

void setup() {
  BTserial.begin(57600);
  Serial.begin(115200);
  Wire.begin(0x69);  // Initialize the I2C bus

  // Initialize each MPU6050 sensor with alternate addresses
  for (int i = 0; i < 6; i++) {
    pinMode(AD0_pins[i], OUTPUT);  // Set the AD0 pins as output
    digitalWrite(AD0_pins[i], HIGH); // Set the AD0 pin low initially to use 0x68 address

    mpu[i].initialize();  // Initialize the MPU6050 sensor

    // Check if the sensor is connected
    if (!mpu[i].testConnection()) {
      digitalWrite(AD0_pins[i], LOW);
      BTserial.print("MPU6050 at index ");
      BTserial.print(i);
      BTserial.print(" not connected!");
      BTserial.println();
      Serial.print("MPU6050 at index ");
      Serial.print(i);
      Serial.print(" not connected!");
      Serial.println();
    } else {
      digitalWrite(AD0_pins[i], LOW);
      BTserial.print("MPU6050 at index ");
      BTserial.print(i);
      BTserial.print(" initialized successfully!");
      BTserial.println();
      Serial.print("MPU6050 at index ");
      Serial.print(i);
      Serial.print(" initialized successfully!");
      Serial.println();
    }

    digitalWrite(AD0_pins[i], LOW);
  }
}

void loop(){
  for(int i = 0; i < 6; i++)
  {
    Wire.setWireTimeout(10000, true);
    digitalWrite(AD0_pins[i], HIGH); // Set the AD0 pin low initially to use 0x68 address

    Wire.beginTransmission(MPU6050_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_addr,14,true);

    AccX=Wire.read()<<8|Wire.read();
    AccY=Wire.read()<<8|Wire.read();
    AccZ=Wire.read()<<8|Wire.read();
    Temp=Wire.read()<<8|Wire.read();
    GyroX=Wire.read()<<8|Wire.read();
    GyroY=Wire.read()<<8|Wire.read();
    GyroZ=Wire.read()<<8|Wire.read();

    Serial.print(i);
    Serial.print(": AX "); Serial.print(AccX);
    Serial.print(" AY "); Serial.print(AccY);
    Serial.print(" AZ "); Serial.print(AccZ);
    Serial.print(" GX "); Serial.print(GyroX);
    Serial.print(" GY "); Serial.print(GyroY);
    Serial.print(" GZ "); Serial.println(GyroZ);

    BTserial.print(i);
    BTserial.print(": AX "); BTserial.print(AccX);
    BTserial.print(" AY "); BTserial.print(AccY);
    BTserial.print(" AZ "); BTserial.print(AccZ);
    BTserial.print(" GX "); BTserial.print(GyroX);
    BTserial.print(" GY "); BTserial.print(GyroY);
    BTserial.print(" GZ "); BTserial.println(GyroZ);

    Wire.endTransmission(true);
    digitalWrite(AD0_pins[i], LOW);
    Wire.clearWireTimeoutFlag();

    delay(10);
  }
}