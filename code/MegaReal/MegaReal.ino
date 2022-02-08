
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include <TouchScreen.h>
#define MINPRESSURE 200
#define MAXPRESSURE 1000
#include <TEA5767N.h>  //https://github.com/mroger/TEA5767
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define sensor_DO 31
#define servo 30
#define servoPower 33
#include <Servo.h>


#define sensorPower 35
#define sensorPin A8
#define bit0 32
#define bit1 33
int value = 0;

Servo myservo;  
int pos = 10;
int dir = 2;


TEA5767N radio = TEA5767N();
const int XP = 8, XM = A2, YP = A3, YM = 9; //ID=0x9341
const int TS_LEFT = 113, TS_RT = 896, TS_TOP = 101, TS_BOT = 903;
const int radioOn = 22;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_GFX_Button on_btn, off_btn , ch1_btn , ch2_btn , ch3_btn , ch4_btn;
int pixel_x, pixel_y;     //Touch_getXY() updates global vars

 bool onRadio = false;
 float currentCh = 88.7;
 float ch1 = 88.7;
 float ch2 = 90.9;
 float ch3 = 88.2;
 float ch4 = 92.7;
 float radioFreq = 88.7; 

 const char *ch1S = "88.7 MHZ";
 const char *ch2S = "90.9 MHZ";
 const char *ch3S = "88.2 MHZ";
 const char *ch4S = "92.7 MHZ";
 const char *currentChS = ch1S;


bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, 240); //.kbv makes sense to me
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, 320);
    }
    return pressed;
}




void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg)
{
    int16_t x1, y1;
    uint16_t wid, ht;
    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(GREEN);
    tft.setTextSize(sz);
    tft.print(msg);
   
}

 void screen(){
      bool down = Touch_getXY();
    on_btn.press(down && on_btn.contains(pixel_x, pixel_y));
    off_btn.press(down && off_btn.contains(pixel_x, pixel_y));
    
    ch1_btn.press(down && ch1_btn.contains(pixel_x, pixel_y));
    ch2_btn.press(down && ch2_btn.contains(pixel_x, pixel_y));
    ch3_btn.press(down && ch3_btn.contains(pixel_x, pixel_y));
    ch4_btn.press(down && ch4_btn.contains(pixel_x, pixel_y));
    if (on_btn.justReleased())
        on_btn.drawButton();
    if (off_btn.justReleased())
        off_btn.drawButton();
        
    if (ch1_btn.justReleased())
        ch1_btn.drawButton();  
    if (ch2_btn.justReleased())
        ch2_btn.drawButton();
    if (ch3_btn.justReleased())
        ch3_btn.drawButton();
    if (ch4_btn.justReleased())
        ch4_btn.drawButton();  
        
    if (!onRadio&&on_btn.justPressed()) {
        onRadio = true;
        on_btn.drawButton(true);
        tft.fillRect(0, 80, 250, 80, BLACK);
        showmsgXY(0, 100, 4, NULL, currentChS);
    }
    if (onRadio&&off_btn.justPressed()) {
        onRadio = false;
        off_btn.drawButton(true);
        tft.fillRect(0, 80, 250, 80, BLACK);
        showmsgXY(0, 100, 4, NULL, "off");
    }
     if (onRadio&&ch1_btn.justPressed()) {
        currentChS = ch1S;
        currentCh=ch1;
        ch1_btn.drawButton(true);
        tft.fillRect(0, 80, 250, 80, BLACK);
        showmsgXY(0, 100, 4, NULL, currentChS);
    }     
     if (onRadio&&ch2_btn.justPressed()) {
        currentChS = ch2S;
        currentCh=ch2;
        ch2_btn.drawButton(true);
        tft.fillRect(0, 80, 250, 80, BLACK);
        showmsgXY(0, 100, 4, NULL, currentChS);
    }
     if (onRadio&&ch3_btn.justPressed()) {
        currentChS = ch3S;
        currentCh=ch3;
        ch3_btn.drawButton(true);
        tft.fillRect(0, 80, 250, 80, BLACK);
        showmsgXY(0, 100, 4, NULL, currentChS);
    } 
     if (onRadio&&ch4_btn.justPressed()) {
        currentChS = ch4S;
        currentCh=ch4;
        ch4_btn.drawButton(true);
        tft.fillRect(0, 80, 250, 80, BLACK);
        showmsgXY(0, 100, 4, NULL, currentChS);
    }     
}
int readSensor() {

  digitalWrite(sensorPower, HIGH);  // Turn the sensor ON
  delay(100);              // wait 10 milliseconds
  value = analogRead(sensorPin);    // Read the analog value form sensor
  digitalWrite(sensorPower, LOW);
  Serial.println(value);
  if(value<470||value>700){
    return 0;
  }
  
  return map(value,470,700,0,3);   
  //return value;
  //return val;
}

void printNum(int n){
  if(n==2){
    digitalWrite(bit0, LOW);
    digitalWrite(bit1, HIGH);
  }
  else if(n==1){
    digitalWrite(bit0, HIGH);
    digitalWrite(bit1, LOW);
  }
  else{
   digitalWrite(bit0, LOW);
   digitalWrite(bit1, LOW);
  }
}
void taskScreenSetup(){
      radio.selectFrequency(88.7);
    radio.setMonoReception();
    radio.setStereoNoiseCancellingOn();
    pinMode(radioOn, OUTPUT);
    
    Serial.begin(9600);
    uint16_t ID = tft.readID();
    tft.begin(ID);
    tft.setRotation(0);            //PORTRAIT
    tft.fillScreen(BLACK);
    on_btn.initButton(&tft,  60, 200, 100, 40, WHITE, CYAN, BLACK, "ON", 2);
    off_btn.initButton(&tft, 180, 200, 100, 40, WHITE, CYAN, BLACK, "OFF", 2);
    ch1_btn.initButton(&tft, 60, 250, 100, 40, WHITE, CYAN, BLACK, "CH1", 2);
    ch2_btn.initButton(&tft, 180, 250, 100, 40, WHITE, CYAN, BLACK, "CH2", 2);
    ch3_btn.initButton(&tft, 60, 300, 100, 40, WHITE, CYAN, BLACK, "CH3", 2);
    ch4_btn.initButton(&tft, 180, 300, 100, 40, WHITE, CYAN, BLACK, "CH4", 2);
    on_btn.drawButton(false);
    off_btn.drawButton(false);
    ch1_btn.drawButton(false);
    ch2_btn.drawButton(false);
    ch3_btn.drawButton(false);
    ch4_btn.drawButton(false);
}
void task1Loop(){
        if(onRadio&&!digitalRead(radioOn)){
          digitalWrite(radioOn,HIGH);
          radio.selectFrequency(radioFreq);
      }
       else if(!onRadio&&digitalRead(radioOn)){
          digitalWrite(radioOn,LOW);
      }
      if(currentCh!=radioFreq){
        radioFreq=currentCh;
        radio.selectFrequency(radioFreq);
      }
      screen();   
}
void rainTaskSetup(){
   myservo.attach(servo); 
  pinMode(servoPower,OUTPUT);
}
void rainTaskLoop(){
    int val = digitalRead(sensor_DO);


  if (val == 0) {
   myservo.write(pos);
   if((pos>=170&&dir>0)||(pos<=10&&dir<0)){
    dir*=-1;              
   }
   pos+=dir;
  } 

  delay(30);
}

void waterLevelSetup(){
    pinMode(sensorPower, OUTPUT);
  pinMode(bit0, OUTPUT);
  pinMode(bit1, OUTPUT);
  pinMode(sensorPin,INPUT);
  
  // Set to LOW so no power flows through the sensor
  digitalWrite(sensorPower, LOW);
  digitalWrite(bit0, LOW);
  digitalWrite(bit1, LOW);
}

void waterLevelLoop(){
  int level = readSensor();
  printNum(level);
  Serial.print("Water level: ");
  Serial.println(level);
  
  delay(100);
}

void RadioTask(void *pvParameters);
void RainTask(void *pvParameters);
void FuelTask(void *pvParameters);
SemaphoreHandle_t sem;

void setup(void)
{
taskScreenSetup();
rainTaskSetup();
waterLevelSetup();
sem = xSemaphoreCreateBinary();
 xTaskCreate(RadioTask, "RadioTask",1000,NULL,1,NULL);
 xTaskCreate(RainTask, "RainTask",1000,NULL,1,NULL);
 xTaskCreate(FuelTask, "FuelTask",1000,NULL,1,NULL);
 xSemaphoreGive(sem);
    //showmsgXY(0, 100, 4, NULL, "808.08 MHZ");
}



/* two buttons are quite simple
 */


void loop(void)
{
    

    
}
void RadioTask(void *pvParameters){
  while(1){
    xSemaphoreTake(sem,portMAX_DELAY);
    task1Loop();
    xSemaphoreGive(sem);
    vTaskDelay(1);
  }
}

void RainTask(void *pvParameters){
  while(1){
    //xSemaphoreTake(sem,portMAX_DELAY);
    rainTaskLoop();
   // xSemaphoreGive(sem);
    //vTaskDelay(1);
  }
}

void FuelTask(void *pvParameters){
  while(1){
    xSemaphoreTake(sem,portMAX_DELAY);
    waterLevelLoop();
    xSemaphoreGive(sem);
    vTaskDelay(1);
  }
}
