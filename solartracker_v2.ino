#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <EEPROM-Storage.h>

char commandSep = ':';

//persistence
EEPROMStorage<int> ee_lightSensorDifference(0, 0);
EEPROMStorage<int> ee_lightSensorTolerance(5, 25);
EEPROMStorage<int> ee_lightSensorLowLight(10, 10);
EEPROMStorage<int> ee_motor_power(15, 50);
EEPROMStorage<float> ee_gyro_tol(20, 0.5);

//PINDEFINITION
int pinLightSensorButton = 30;
int pinPanelLeftButton = 26;
int pinPanelRightButton = 28;
int pinPanelHomeButton = 32;

//VARIABLES
bool lightSensorButton = false;
bool panelLeftButton = false;
bool panelRightButton = false;
bool panelHomeButton = false;

//BTS7960 motor driver sketch
//PINDEFINTION
//int R_PWM = 5;
//int L_PWM = 6;
//int R_EN = 4;
//int L_EN = 7;
int pinMotor1 = 9;
int pinMotor2 = 8;
int pinMotorpower = 10;

int pinEndTaster1 = 3; //for clockwise
int pinEndTaster2 = 2; //for counterclockwise

//VARIABLES
int motor_power = 50;
bool valEndTaster1 = 0;
bool valEndTaster2 = 0;

//Gyro
Adafruit_MPU6050 mpu;
float gyro_tol = 0.5;
float gyro_y_axis = 0.0;

//Lightsensors
//PINDEFINITION
int lightSensor1 = A0;
int lightSensor2 = A1;

//VARIABLES
int valLightSensor1 = 0;
int valLightSensor2 = 0;
int lightSensorDifference = 0;
int lightSensorTolerance = 25;
int lightSensorLowLight = 0;
int lightSensorCounter = 0;

int buzzer = 53;

void setup()
{
  Serial.begin(9600);   //Tx0 and Rx0  //Serielle Schnittstelle für PC Starten
  Serial1.begin(9600);  //Tx1 and Rx1  //Serielle Schnittstelle 1 für Bluetooth starten
  //Serial2;            //Tx2 und Rx2 (Nicht benötigt)
  //Serial3;            //Tx3 und Rx3 (Nicht benötigt)
  
  setupGyro();
  setupMotor();
  initDefaultValues();
  Serial.println("Initialization finished");
  beep();
}

void initDefaultValues() {
  motor_power = ee_motor_power.get();
  gyro_tol = ee_gyro_tol.get();
  lightSensorDifference = ee_lightSensorDifference.get();
  lightSensorTolerance = ee_lightSensorTolerance.get();
  lightSensorLowLight = ee_lightSensorLowLight.get();
}

void setupMotor() {
  pinMode(pinEndTaster1, INPUT_PULLUP);
  pinMode(pinEndTaster2, INPUT_PULLUP);

  pinMode(pinLightSensorButton, INPUT);
  pinMode(pinPanelLeftButton, INPUT);
  pinMode(pinPanelRightButton, INPUT);
  pinMode(pinPanelHomeButton, INPUT);

  //we use interrupt to immediatly stop the motor
  attachInterrupt(digitalPinToInterrupt(pinEndTaster1), emergencyStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinEndTaster2), emergencyStop, FALLING);

//  pinMode(R_PWM, OUTPUT);
//  pinMode(L_PWM, OUTPUT);
//  pinMode(R_EN, OUTPUT);
//  pinMode(L_EN, OUTPUT);
    pinMode(pinMotorpower, OUTPUT);
    pinMode(pinMotor1, OUTPUT);
    pinMode(pinMotor2, OUTPUT);
  //pinMode(statusLED, OUTPUT);
}

void emergencyStop() {
  if (digitalRead(pinEndTaster1) == LOW || digitalRead(pinEndTaster2) == LOW) {
    Serial.println("Motor Stop");
    motor_stop();
  }
}

void setupGyro() {
  // Try to initialize mpu
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  //mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop()
{
  readSensors();
  //updateMode();
  adjustMotors();
  communicate();
  //delay(500);
}

void log(String msg){
  Serial.println(msg);
}

void beep(){
  tone(buzzer,500,1000);
}

void readSensors() {
  //read switches
  //left/right end switch
  valEndTaster1 = digitalRead(pinEndTaster1);
  valEndTaster2 = digitalRead(pinEndTaster2);

  //lightsensor on/off switch
  lightSensorButton  = digitalRead(pinLightSensorButton);
  //directional switches left/right
  panelLeftButton = digitalRead(pinPanelLeftButton);
  panelRightButton = digitalRead(pinPanelRightButton);
  //home switch
  panelHomeButton = digitalRead(pinPanelHomeButton);

  //read lightsensors
  valLightSensor1 = analogRead(lightSensor1);
  valLightSensor2 = analogRead(lightSensor2);

  //read gyro
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyro_y_axis = a.acceleration.y;
  
  String msg = (String)"EndTaster1:"+valEndTaster1+" EndTaster2:"+valEndTaster2;
  msg += (String)" lightSensorButton:"+lightSensorButton+" panelLeftButton:"+panelLeftButton+" panelRightButton:"+panelRightButton+" panelHomeButton:"+panelHomeButton;
  msg += (String)" gyro_y_axis:"+gyro_y_axis;
  msg += (String)" lightSensorDifference"+lightSensorDifference;
  msg += (String)" motor_power"+motor_power;
  msg += (String)" valLightSensor1:"+valLightSensor1+" valLightSensor2:"+valLightSensor2;
  log(msg);
}

void updateMode() {
  
  if(lightSensorButton){
    lightSensorCounter++;
    if(lightSensorCounter >20){
        calibrateLightSensors("");
        lightSensorCounter=0;
    }
  }else{
    Serial.print((String) "Counter: "+lightSensorCounter);
    lightSensorCounter = 0;
  }
}

void homePanel(){
  log("home");
  int dir =1;
  while(dir != 0){
    //read gyro
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_y_axis = a.acceleration.y;
    dir = getGyroSensorDirection();
    switch (dir) {
    case 1:
      motor_cw();
      break;
    case -1:
      motor_ccw();
      break;
    default:
      motor_stop();
      break;
    }
  }
}
void adjustMotors() {
  
  int dir = 0;
  
  if(  panelLeftButton == HIGH && panelRightButton == HIGH){
      log("adjustMotors: gyro");
      dir = getGyroSensorDirection();
  }else if( panelLeftButton == HIGH ){
      log("adjustMotors: left");
      dir = 1;
  }else if( panelRightButton == HIGH){
      log("adjustMotors: right");
      dir = -1;
  }else if (lightSensorButton == HIGH){
      log("adjustMotors: light");
      dir = getLightSensorDirection();
  }else{
      log("adjustMotors: off");
      dir = 0;
  }

  switch (dir) {
    case 1:
      motor_cw();
      break;
    case -1:
      motor_ccw();
      break;
    default:
      motor_stop();
      break;
  }
}

void communicate() {
  //first check message handling
  if (Serial1.available() <= 0) return;

  String sendText = Serial1.readString();
  int sepPos = sendText.indexOf(commandSep);
  if (sepPos < 0) return;

  //split command and parameter
  String command = sendText.substring(0, sepPos);
  String param = sendText.substring(sepPos + 1);

  if (command.equals("INFO")) {
    sendInfo(param);
  } else if (command.equals("UPD_LIGHT_CALIBRATE")) {
    calibrateLightSensors(param);
  } else if (command.equals("UPD_LIGHT_DIFF")) {
    updateLightSensorDifference(param);
  } else if (command.equals("UPD_LIGHT_LOWLIGHT")) {
    updateLightSensorLowLight(param);
  } else if (command.equals("UPD_LIGHT_TOL")) {
    updateLightSensorTolerance(param);
  } else if (command.equals("UPD_GYRO_TOL")) {
    updateGyroTolerance(param);
  } else if (command.equals("UPD_MOTOR_PWR")) {
    updateMotorPower(param);
  }
}

void sendData(String transmitData) {
  Serial1.println(transmitData);
  Serial1.flush();
}

void sendInfo(String param) {

}

/**
   read the two lightsensor, calculate difference and store to eeprom
*/
void calibrateLightSensors(String param) {
  beep();
  valLightSensor1 = analogRead(lightSensor1);
  valLightSensor2 = analogRead(lightSensor2);
  lightSensorDifference = valLightSensor1 - valLightSensor2;
  ee_lightSensorDifference.set(lightSensorDifference);
  delay(1000);
  beep();

}

/**
   converts string to int, updates difference and stores to eeprom
*/
void updateLightSensorDifference(String param) {
  if (param.length() == 0) return;
  int value = param.toInt();
  //no correct value
  if ( value == 0 ) return;

  lightSensorDifference = value;
  ee_lightSensorDifference.set(lightSensorDifference);
}

void updateLightSensorTolerance(String param) {
  if (param.length() == 0) return;
  int value = param.toInt();
  //no correct value
  if ( value == 0 ) return;

  lightSensorTolerance = value;
  ee_lightSensorTolerance.set(lightSensorTolerance);
}

void updateGyroTolerance(String param) {
  if (param.length() == 0) return;
  float value = param.toFloat();
  //no correct value
  if ( value == 0.0 ) return;

  gyro_tol = value;
  ee_gyro_tol.set(gyro_tol);
}

void updateMotorPower(String param) {
  if (param.length() == 0) return;
  int value = param.toInt();
  //no correct value
  if ( value == 0 ) return;

  motor_power = value;
  ee_motor_power.set(motor_power);
}

void updateLightSensorLowLight(String param) {
  if (param.length() == 0) return;
  int value = param.toInt();
  //no correct value
  if ( value == 0 ) return;

  lightSensorLowLight = value;
  ee_lightSensorLowLight.set(lightSensorLowLight);
}

//determine direction to home the panel
int getGyroSensorDirection() {
  if (gyro_y_axis > gyro_tol) {
    return 1;
  } else if (gyro_y_axis < -gyro_tol) {
    return -1;
  }
  return 0;
}

int getLightSensorDirection() {
  int change =  valLightSensor1 - valLightSensor2 - lightSensorDifference;
  if (change > lightSensorTolerance) {
    return 1;
  } else if (change < -lightSensorTolerance) {
    return -1;
  }
  return 0;
}

void motor_cw() {
  if (digitalRead(pinEndTaster1) == LOW) return;

//  analogWrite(R_PWM, motor_power);
//  analogWrite(L_PWM, 0);
//  digitalWrite(R_EN, HIGH);
//  digitalWrite(L_EN, HIGH);
    digitalWrite(pinMotor1, HIGH);
    digitalWrite(pinMotor2, LOW);
    analogWrite(pinMotorpower,motor_power);
}

void motor_ccw() {
  if (digitalRead(pinEndTaster2) == LOW) return;

  //analogWrite(R_PWM, 0);
  //analogWrite(L_PWM, motor_power);
  //digitalWrite(R_EN, HIGH);
  //digitalWrite(L_EN, HIGH);
    digitalWrite(pinMotor1,LOW );
    digitalWrite(pinMotor2,HIGH );
    analogWrite(pinMotorpower,motor_power);
}

void motor_stop() {
  //analogWrite(R_PWM, 0);
  //analogWrite(L_PWM, 0);
  //digitalWrite(R_EN, LOW);
  //digitalWrite(L_EN, LOW);
    digitalWrite(pinMotor1,LOW );
    digitalWrite(pinMotor2,LOW );
    analogWrite(pinMotorpower,0);
}
