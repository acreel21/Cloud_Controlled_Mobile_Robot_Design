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

typedef struct Data{
  double X;
  double Y;
  double phi;
} Data;

int L = 90; //base length
float r = 15; //radius of wheels
float cl = 2*3.14*(L); //circumference
float cr = 2*3.14*(r); //circumference
float d = cr/(76*4);
float phi = (360*(d/cl))*((2*3.14)/360); //angle
float deltaX = (L/2)*sin(phi);
float deltaY = (L/2)-(L/2)*cos(phi);
float xGlobal = 0;
float yGlobal = 0;
float phiGlobal = 0;
float deltaXp = 0; 
float deltaYp = 0;
float phiDir = 0;
float head = 0;
int cntR = 0; //tick for right wheel
int cntL = 0; //tick for left wheel
float kg = 0.0133333; //gear ratio
float azold = 0.94;
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int vell = 0;
int velr = 0;
int w = 0;
int errorDir = 0;
int errorSper = 0;
int errorSpel = 0;
int Kpspe = 25;
int Kpdir = 2;
int pickup = 0;
int Ml = 0; //left motor
int Mr = 0; //right motor
float thetar = 0;
float thetal = 0;
Robot myRobot;
Data myData;

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
  imuSetup();  //set-up IMU 
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("In loop");
  checkImu(); //Updates IMU data 
  checkUDP(); //Updates UDP data
  setMotor(); //turns the motor at the right speed
  setMl(myRobot.Velocity); //Feedback for speed
  setMr(myRobot.Velocity); //Feedback for speed
  setDir(myRobot.Theta);
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
  pinMode(encoderR, INPUT_PULLUP);
  pinMode(encoderL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderR), encoderR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderL), encoderL_ISR, CHANGE);
  Serial.println("Pin all set-up");
}

void imuSetup(){
  //IMU setup
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-5779, -4191, -4522};
  compass.m_max = (LSM303::vector<int16_t>){+562, +1726, +1789};
  Serial.println("IMU all set-up");
}

void encoderR_ISR(){
  cntR++;
  phiGlobal += phi;
  deltaXp = ((deltaX*cos(phiGlobal))+deltaY*sin(phiGlobal));
  deltaYp = ((deltaX*sin(phiGlobal))+deltaY*cos(phiGlobal));
  xGlobal += deltaXp;
  yGlobal += deltaYp;
  
  //Serial.print("XR: ");
  //Serial.println(xGlobal);
  //Serial.print("Y: ");
  //Serial.println(yGlobal);
  //Serial.print("ThetaZ: ");
  //Serial.println(phiGlobal*(360/(2*3.14)));
}

void encoderL_ISR(){
  cntL++;
  phiGlobal -= phi;
  deltaXp = ((deltaX*cos(phiGlobal))+deltaY*sin(phiGlobal));
  deltaYp = ((deltaX*sin(phiGlobal))+deltaY*cos(phiGlobal));
  xGlobal += deltaXp;
  yGlobal += deltaYp;
  
  //Serial.print("XL: ");
  //Serial.println(xGlobal);
  //Serial.print("Y: ");
  //Serial.println(yGlobal);
  //Serial.print("ThetaZ: ");
  //Serial.println(phiGlobal*(360/(2*3.14)));
}

void checkImu(){
  compass.read();
  float az = ((double)(compass.a.z)*0.061)/1000.0;
  head = compass.heading((LSM303::vector<int>){0, 0, -1});
  if (head > 110 && head < 190){
    head += 35;
  }
  if (head > 190 && head < 300){
    head += 50;
  }
  if (head > 300){
    head = abs(head - 360);
  }
  //Serial.print("The heading in the IMU:");
  //Serial.println(head);
  float Jz = (az-azold);
  if (Jz >= 0.6){
    pickup = 1;
    //Serial.println("Pickup");
  }
  //Serial.println("Checking IMU");
}

void checkUDP(){
  //Serial.println("IN UDP");
  int packetSize = Udp.parsePacket(); //parse packet for UDP
    if (packetSize > 0) {
        // read the packet into recvBuffer
        int len = Udp.read(recvBuffer, BUFSZ); //read different packet from UDP packet
        if (len > 0) {
            Serial.println("Contents:");
            //Serial.println(recvBuffer);
            memcpy(&myRobot, recvBuffer, sizeof(myRobot));
            if (myRobot.Mode == 1){
              pickup = 0;
              phiGlobal = 0;
              xGlobal = 0;
              yGlobal = 0;
              errorDir = 0;
              errorSpel = 0;
              errorSper = 0;
            }
            else if (myRobot.Mode == 2){
              Serial.println("Mode 2");
              myData.X = xGlobal;
              myData.Y = yGlobal;
              myData.phi = head;
              Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
              char txBuffer[128] = { 0 };
              memcpy(txBuffer, &myData, sizeof(Data));
              Udp.write(txBuffer, sizeof(Data));
              Udp.endPacket();
            }
            Serial.println(myRobot.Velocity);
            Serial.println(myRobot.Theta);
        }
        else {
            Serial.println("Read 0 bytes.");
        }
    }
}

int setMl(double v1){
  float(thetal) = (float(cntL)*(0.5))*kg; //gets speed
  errorSpel = v1 - thetal; //finds error
  vell = errorSpel*Kpspe; //finds velocity for the motors
  //Serial.print("Desired Velocity is: ");
  //Serial.println(v1);
  //Serial.print("Theta is: ");
  //Serial.println(thetal);
  //Serial.print("errorSpeL is: ");
  //Serial.println(errorSpel);
  //Serial.print("Velocity in set M1 is: ");
  //Serial.println(vell);
  cntL=0;
}

int setMr(double v2){
  float(thetar) = (float(cntR)*(0.5))*kg; //gets speed
  errorSper = v2 - thetar; //finds error
  velr = errorSper*Kpspe; //finds velocity for the motors
  //Serial.print("Desired Velocity is: ");
  //Serial.println(v2);
  //Serial.print("Theta right is: ");
  //Serial.println(thetar);
  //Serial.print("errorSpe is: ");
  //Serial.println(errorSper);
  //Serial.print("Velocity right is: ");
  //Serial.println(velr);
  cntR=0;
}

int setDir(double t){
  phiDir = (float)phiGlobal*(360/(2*3.14));//0.1*(float)phiGlobal*(360/(2*3.14))+0.9*(int)head;
  errorDir = t - phiDir; //finds error
  w = errorDir*Kpdir; //finds angluar speed
  //Serial.print("The desired angluar speed:  ");
  //Serial.println(t);
  //Serial.print("The heading:  ");
  //Serial.println(head);
  Serial.print("The phi:  ");
  Serial.println(phiDir);
  //Serial.print("The phi Global:  ");
  //Serial.println(phiGlobal);
  Serial.print("The error is:  ");
  Serial.println(errorDir);
  Serial.print("The angluar speed:  ");
  Serial.println(w);
}

void setMotor(){
  Mr = (((2*velr) + (w*L))/(2*r));  //motor speed right wheel
  Ml = (((2*vell) - (w*L))/(2*r)); //motor speed left wheel
  //Serial.print("Velocity in set M1 is: ");
  //Serial.println(vell);
  //Serial.print("Velocity right is: ");
  //Serial.println(velr);
  //Serial.print("The angluar speed:  ");
  //Serial.println(w);
  //Serial.print("Right wheel :  ");
  //Serial.println(Mr);
  //Serial.print("Left wheel :  ");
  //Serial.println(Ml);
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
  if (pickup == 1 || myRobot.Mode == 3 || myRobot.Mode == 1){ //check if robot was pickup or not
     Ml = 0;
     Mr = 0;
  }
  analogWrite(motorRfow,Mr);
  analogWrite(motorRbac,0);
  analogWrite(motorLfow,Ml);
  analogWrite(motorLbac,0);
}
