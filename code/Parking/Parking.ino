#include <Arduino_FreeRTOS.h>

const int ENA = 3;
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 7;
const int ENB = 6;
const int buzzer = 11;
#define echoPinForward A3 
#define trigPinForward 12

#define echoPinRightForward A2 
#define trigPinRightForward 10

#define echoPinRightBackward A1 
#define trigPinRightBackward 9


#define echoPinBackward A0 
#define trigPinBackward 8

void car(void *pvParameters);

void carSetup(){
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode(buzzer,OUTPUT);
}

void sensorsSetup(){
  pinMode(trigPinForward, OUTPUT);
  pinMode(echoPinForward, INPUT);

  pinMode(trigPinRightForward, OUTPUT);
  pinMode(echoPinRightForward, INPUT);

  pinMode(trigPinRightBackward, OUTPUT); 
  pinMode(echoPinRightBackward, INPUT);

    pinMode(trigPinBackward, OUTPUT);
  pinMode(echoPinBackward, INPUT);
}




void setup() {
    carSetup();
  sensorsSetup();
  Serial.begin(9600);
  xTaskCreate(car, "CarParking",1000,NULL,1,NULL);
  // put your setup code here, to run once:

}

int state = 0;
bool v = false;
bool SC = false;
int l1 =255;
int l2 = 155;
int l3 = 70;


void park(){
  if(state==1){
    Serial.println(1);
    if(v){
      
      if((readRightForwardSensor()<20)){
        right();
        delay(280);
        stopCar();
        delay(500);
        v=false;
        state=2;
      }
      else{
        forward();
      }
    }
    else{
      if((readRightForwardSensor()>=20)&&(readRightBackwardSensor()>=20)){
        v=true;
      }
      forward();
      
    }
   
    
  }
  else if(state == 2){
    int d =readBackwardSensor();
    int volume = (int)((d*255)/7.0);
    if(volume>l2){
      volume = l3;
    }
    else if(volume>l1){
      volume = l2;
    }
    else{
      volume = l1;
    }
    analogWrite(buzzer, volume);
    Serial.println(2);
    if(d<7){
      stopCar();
      delay(200);
      analogWrite(buzzer, 0);
      left();
      delay(260);
      stopCar();
      delay(200);
      state=3;
      }
      else{
         backward();
      }

     
    
  }
  else if(state ==3){
    Serial.println(3);
    if(readForwardSensor()<7){
      stopCar();
      delay(200);
      state=0;
    }
    else{
      forward();
    }
    
    
  }
  else{
    //Serial.println(5);
    if(!SC){
      stopCar();
      
      SC=true;
    }
    if(Serial.available()>0&&Serial.read()==97){
          state = 1;
          SC=false;
      }
 
    
  }
   delay(50);
}

void loop() {

  // put your main code here, to run repeatedly:

}
void forward(){
  if(readForwardSensor()>7){
      analogWrite(ENA,65);
      analogWrite(ENB,85);
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);
      digitalWrite(IN4,HIGH);
      digitalWrite(IN3,LOW);
  }
  else{
    stopCar();
   
  }

}
void backward(){
      analogWrite(ENA,65);
      analogWrite(ENB,85);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);
      digitalWrite(IN4,LOW);
      digitalWrite(IN3,HIGH);
  

}

void right(){
  analogWrite(ENA,200);
  analogWrite(ENB,190);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN4,HIGH);
  digitalWrite(IN3,LOW);
}

void left(){
  analogWrite(ENA,200);
  analogWrite(ENB,190);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,HIGH);
}
void stopCar(){
    analogWrite(ENA,0);
  analogWrite(ENB,0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,LOW);
}

int readForwardSensor(){
  digitalWrite(trigPinForward, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPinForward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinForward, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPinForward, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;

}
int readRightForwardSensor(){
  digitalWrite(trigPinRightForward, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPinRightForward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRightForward, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPinRightForward, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;

}

int readRightBackwardSensor(){
  digitalWrite(trigPinRightBackward, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPinRightBackward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRightBackward, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPinRightBackward, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;

}

int readBackwardSensor(){
  digitalWrite(trigPinBackward, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPinBackward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBackward, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPinBackward, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;

}
void car(void *pvParameters){
  while(1){
     park();
  }
  
}
