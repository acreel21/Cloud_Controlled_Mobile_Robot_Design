#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <math.h>
#include <LSM303.h>
#include <Wire.h>
#include "arduino_secrets.h"
#define encoderR 5
#define encoderL 6
#define motorRfow 9
#define motorRbac 10
#define motorLfow 12
#define motorLbac 11
#define BUFSZ 1024

typedef struct Robot{
  double Velocity;
  double Theta;
  int Mode;
} Robot;

int L = 90; //base length
float r = 30; //radius of wheels
float cl = 2*3.14*(L/2); //circumference
float cr = 2*3.14*(r/2); //circumference
float d = cr/(76*2);
float phi = 90*(d/cl); //angle
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
int Kpdir = 1;
int pickup = 0;
int Ml = 0; //left motor
int Mr = 0; //right motor
float theta = 0;
Robot myRobot;

IPAddress ip(192, 168, 1, 99); //set-up static ip address
IPAddress gate(192, 168, 0, 1);
IPAddress sub(255, 255, 0, 0);
IPAddress ip2;
IPAddress server(192,168,1,99); //udp ip address
unsigned int localPort = 5005; //udp port
WiFiUDP Udp;
LSM303 compass;
char recvBuffer[BUFSZ];

void setup() {
  // put your setup code here, to run once:
  wifiSetup(); //setup wifi
  openPort(); //UDP set-up
  pinSetup(); //configure pins and interrupts
  //imuSetup();  //set-up IMU 
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("In loop");
  //checkImu(); //Updates IMU data 
  checkUDP(); //Updates UDP data
  setSpe(myRobot.Velocity); //Feedback for speed
  setDir(myRobot.Theta); //Feedback for theta
  setMotor(); //turns the motor at the right speed
}

void wifiSetup(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  WiFi.config(ip, gate, sub);
  WiFi.begin(ssid, pass);
  ip2 = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip2);
  Serial.println("WiFi set-up");
}

void openPort(){
  //UDP setup
  Udp.begin(localPort);
  Serial.println("UDP set-up");
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
  Serial.println("Pin all set-up");
}

void imuSetup(){
  //IMU setup
  Wire.begin();
  compass.init();
  compass.enableDefault();
  Serial.println("IMU all set-up");
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
  compass.read();
  float heading = compass.heading();
  //Serial.println("Checking IMU");
}

void checkUDP(){
  Serial.println("IN UDP");
  int packetSize = Udp.parsePacket(); //parse packet for UDP
    if (packetSize > 0) {
        // read the packet into recvBuffer
        int len = Udp.read(recvBuffer, BUFSZ); //read different packet from UDP packet
        if (len > 0) {
            //Serial.println("Contents:");
            //Serial.println(recvBuffer);
            memcpy(&myRobot, recvBuffer, sizeof(myRobot));
            //Serial.println(myRobot.Velocity);
            //Serial.println(myRobot.Theta);
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
  //Serial.print("Theta is: ");
  //Serial.println(vel);
  //Serial.print("errorSpe is: ");
  //Serial.println(vel);
  //Serial.print("Velocity is: ");
  //Serial.println(vel);
}

int setDir(double t){
  errorDir = t - phiGlobal; //finds error
  w = errorDir*Kpdir; //finds angluar speed
  Serial.print("The desired angluar speed:  ");
  Serial.println(t);
  Serial.print("The phi:  ");
  Serial.println(phi);
  Serial.print("The phi Global:  ");
  Serial.println(phiGlobal);
  Serial.print("The angluar speed:  ");
  Serial.println(w);
}

void setMotor(){
  Mr = 9*(((2*vel) + (w*L))/(2*r));  //motor speed right wheel
  Ml = 9*(((2*vel) - (w*L))/(2*r)); //motor speed left wheel
  if (Ml > 255){
    Ml = 255;
  }
  if (Mr > 255){
    Mr = 255;
  }
  if (Ml < 0){
    Ml = 0;
  }
  if (Mr < 0){
    Mr = 0;
  }
  if (pickup == 1){ //check if robot was pickup or not
     Ml = 0;
     Mr = 0;
  }
  Serial.print("Right wheel :  ");
  Serial.println(Mr);
  Serial.print("Left wheel :  ");
  Serial.println(Ml);
  analogWrite(motorRfow,Mr);
  analogWrite(motorRbac,0);
  analogWrite(motorLfow,Ml);
  analogWrite(motorLbac,0);
}
