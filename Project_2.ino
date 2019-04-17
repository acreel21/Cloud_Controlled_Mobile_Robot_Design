#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <math.h>
#include <Wire.h>
#include "Adafruit_LSM303.h"
#include "Adafruit_LSM303_U.h"
#include "arduino_secrets.h"
#define encoderR 5
#define encoderL 6
#define motorRfow 9
#define motorRbac 10
#define motorLfow 11
#define motorLbac 12
#define BUFSZ 90

int L = 9; //base length
float r = 2.5; //radius of wheels
float c = 2*3.14*L; //circumference
float phi = (2*c)/L; //angle
float phiGlobal = 0; 
float xGlobal = 0; 
float yGlobal = 0;
float matr[4][4] = {{cos(phiGlobal), sin(phiGlobal), 0, 0}, {sin(phiGlobal), cos(phiGlobal), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
float matl[4][4] = {{cos(phiGlobal), sin(phiGlobal), 0, 0}, {sin(phiGlobal), cos(phiGlobal), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
float matTrg[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
float matTlg[4][4]  = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
float matRotr[4][4] = {{cos(phi), -sin(phi), 0, 0}, {sin(phi), cos(phi), 0, (L/2)}, {0, 0, 1, 0}, {0, 0, 0, 1}}; //Matrix angle math for robot
float matRotl[4][4] = {{cos(-phi), -sin(-phi), 0, 0}, {sin(-phi), cos(-phi), 0, (-L/2)}, {0, 0, 1, 0}, {0, 0, 0, 1}}; //Matrix angle math for robot
float matPosr[4][4] = {{1, 0, 0, 0}, {0, 1, 0, (-L/2)}, {0, 0, 1, 0}, {0, 0, 0, 1}}; //Matrix position math for robot
float matPosl[4][4] = {{1, 0, 0, 0}, {0, 1, 0, (L/2)}, {0, 0, 1, 0}, {0, 0, 0, 1}}; //Matrix position math for robot
float temp[4][4] = {0};
int i,j,k = 0;
int cntR = 0; //tick for right wheel
int cntL = 0; //tick for left wheel
int count = 0; //average tick count
float kg = 0.0133333; //gear ratio
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int vel = 0;
int w = 0;
int errorDir = 0;
int errorSpe = 0;
int Kpspe = 4;
int Kpdir = 4;
int pickup = 0;
int Ml = 0; //left motor
int Mr = 0; //right motor
float theta = 0;

IPAddress ip(192, 168, 1, 99); //set-up static ip address
IPAddress gate(192, 168, 0, 1);
IPAddress sub(255, 255, 0, 0);
IPAddress ip2;
IPAddress server(192,168,1,99); //udp ip address
unsigned int localPort = 5005; //udp port
WiFiUDP Udp;
char recvBuffer[BUFSZ];

void setup() {
  // put your setup code here, to run once:
  wifiSetup(); //setup wifi
  openPort(); //UDP set-up
  intiStruct(); //set-up struct
  pinSetup(); //configure pins and interrupts
  imuSetup();  //set-up IMU 
}

void loop() {
  // put your main code here, to run repeatedly:
  checkImu(); //Updates IMU data 
  checkUDP(); //Updates UDP data
  setSpe(Velocity); //Feedback for speed
  setDir(Theta); //Feedback for theta
  if (pickup == 1){ //check if robot was pickup or not
    Ml = 0;
    Mr = 0;
  }
  setMotor(); //turns the motor at the right speed
}

void wifiSetup(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  WiFi.begin(ssid, pass);
  ip2 = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip2);
}

void openPort(){
  //UDP setup
  Udp.begin(localPort);
}

void intiStruct(){
  typedef struct Robot{
    double Velocity;
    double Theta;
    int Mode;
  } Robot;
}

void pinSetup(){
  pinMode(motorRfow, OUTPUT);
  pinMode(motorRbac, OUTPUT);
  pinMode(motorLfow, OUTPUT);
  pinMode(motorLbac, OUTPUT);
  pinMode(encoderR, INPUT);
  pinMode(encoderL, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderR), encoderR_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderL), encoderL_ISR, RISING);
}

void imuSetup(){
  //IMU setup
}

void encoderR_ISR(){
  cntR += 1;
  phiGlobal += phi;
  for(i = 0; i < 4; i++){
    for(j = 0; j < 4; j++){
      for(k = 0; k < 4; k++){
        temp[i][j] += matRotr[i][k]*matPosr[k][j];
      }
    }
  }

  for(i = 0; i < 4; i++){
    for(j = 0; j < 4; j++){
      for(k = 0; k < 4; k++){
        matTrg[i][j] += matr[i][k]*temp[k][j];
      }
    }
  }

  for(i = 0; i < 4; i++){
    for(j = 0; j < 4; j++){
       temp[i][j] = 0;
    }
  }

  matr[0][0] = cos(phiGlobal);
  matr[0][1] = sin(phiGlobal);
  matr[1][0] = sin(phiGlobal);
  matr[1][1] = cos(phiGlobal);
  
  //Serial.print("X: ");
  //Serial.println(matTrg[0][3]);
  //Serial.print("Y: ");
  //Serial.println(matTrg[1][3]);
  //Serial.print("ThetaZ: ");
  //Serial.println(phiGlobal);
}

void encoderL_ISR(){
  cntL += 1;
  phiGlobal -= phi;
  for(i = 0; i < 4; i++){
    for(j = 0; j < 4; j++){
      for(k = 0; k < 4; k++){
        temp[i][j] += matRotl[i][k]*matPosl[k][j];
      }
    }
  }

  for(i = 0; i < 4; i++){
    for(j = 0; j < 4; j++){
      for(k = 0; k < 4; k++){
        matTlg[i][j] += matl[i][k]*temp[k][j];
      }
    }
  }

  for(i = 0; i < 4; i++){
    for(j = 0; j < 4; j++){
       temp[i][j] = 0;
    }
  }

  matl[0][0] = cos(phiGlobal);
  matl[0][1] = sin(phiGlobal);
  matl[1][0] = sin(phiGlobal);
  matl[1][1] = cos(phiGlobal);
  
  //Serial.print("X: ");
  //Serial.println(matTrg[0][3]);
  //Serial.print("Y: ");
  //Serial.println(matTrg[1][3]);
  //Serial.print("ThetaZ: ");
  //Serial.println(phiGlobal);
}

void checkImu(){
  
}

void checkUDP(){
  int packetSize = Udp.parsePacket(); //parse packet for UDP
    if (packetSize > 0) {
        // read the packet into recvBuffer
        int len = Udp.read(recvBuffer, BUFSZ); //read different packet from UDP packet
        if (len > 0) {
            Serial.println("Contents:");
            Serial.println(recvBuffer);
        }
        else {
            Serial.println("Read 0 bytes.");
        }
    }
}

int setSpe(double v){
  count = (cntR + cntL)/2; //average ticks together
  float(theta) = (float(count)*(0.5))*kg; //gets speed
  errorSpe = v - theta; //finds error
  vel = errorSpe*Kpspe; //finds velocity for the motors
}

int setDir(double t){
  errorDir = t - phiGlobal; //finds error
  w = errorDir*Kpdir; //finds angluar speed
}

void setMotor(){
   Mr = (((2*vel) + (w*L))/(2*r));  //motor speed right wheel
   Ml = (((2*vel) - (w*L))/(2*r)); //motor speed left wheel
   analogWrite(motorRfow,Mr);
   analogWrite(motorRbac,0);
   analogWrite(motorLfow,Ml);
   analogWrite(motorLbac,0);
}
