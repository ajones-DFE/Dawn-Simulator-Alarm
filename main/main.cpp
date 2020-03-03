// main.cpp / sketch.ino

// a library or two... ///////////////////////////////////////////////////////
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include "private.h"
#include "unphone.h"
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
// libraries for projects; comment out as required
#include <Adafruit_VS1053.h>      // the audio chip
#include <RCSwitch.h>             // 433 MHz remote switching
#include <DHTesp.h>               // temperature / humidity sensor
#include <GP2Y1010_DustSensor.h>  // the Sharp dust sensor
#include <Adafruit_NeoMatrix.h>   // neopixel matrix
#include <Adafruit_NeoPixel.h>    // neopixels generally
#include <Adafruit_MotorShield.h> // the hbridge motor driver
#include <Adafruit_TSL2591.h>     // light sensor
//Extra alarm libraries
#include "Adafruit_HX8357.h"     // tft display local hacked version
#include <Adafruit_STMPE610.h>  // touch screen
#include <Adafruit_GFX.h>        // core graphics library
#include <string>
#include <math.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// OTA, MAC address, messaging, loop slicing//////////////////////////////////
int firmwareVersion = 33; // keep up-to-date! (used to check for updates)
char *getMAC(char *);    // read the address into buffer
char MAC_ADDRESS[13];    // MAC addresses are 12 chars, plus the NULL
int loopIter = 0;        // loop slices

String apSSID = String("ProjectThing"); // SSID of the AP
String apPassword = "pwUnPhone";     // passkey for the AP

#define PIN A7 // Neopixel Board
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 32
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel pixelss you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Methods

//calculate increments for transitions
double calcInc(double start, double finish);

// altered lcdMessage method
void printMessage(char *s, uint16_t x, uint16_t y, uint16_t fontSize, uint16_t txtColor);

// message on screen
void lcdMessage(char *);

// pixel fade methods
void fadeDarkBlueToFirst();
void fadeFirstToSecond();
void fadeSecondToThird();
void fadeThirdToFourth();
void fadeFourthToFifth();
void fadeFifthToSixth();
void fadeSixthToFinal();


// clear NeoPixel
void clear();

// display Screens
void homeScreen();
void setAlarmScreen();
void alarmScreen();
void snoozeScreen();
void dawnSim();
void updateTime();
void timeZoneScreen();
void alterTimeZone();
void wifiScreen();


// fade
void pixelsFade(uint16_t *pixelArray, int size, int baseR, int baseG, int baseB, int incR, int incG, int incB);

const uint16_t GLOBAL_STEPS = 100;

// pixel transition methods
double darkToLightBlueInc = calcInc(30.0,100.0);
double lightToOrangeRedInc = calcInc(0.0,255.0);
double lightToOrangeGreenInc = calcInc(0.0,60.0);
double lightToOrangeBlueInc = calcInc(100.0,0.0);
double orangeToYellowGreenInc = calcInc(60.0,255.0);
double yellowToWhiteBlueInc = calcInc(0.0,255.0);

// specific pixels on the NeoPixel
uint16_t inner[] = {27, 28};
uint16_t inner_middle[] = {18, 19, 20, 21, 26, 29};
uint16_t outer_middle[] = {9, 10, 11, 12, 13, 14, 17, 22, 25, 30};
uint16_t outer[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 15, 16, 23, 24, 31};

// timezones
int timezone[] = {-1200,-1100,-1000,-930,-900,-800,-700,-600,-500,-400,-330,-300,-200,-100,0,
100,200,300,330,400,430,500,530,545,600,630,700,800,845,900,930,1000,1030,1100,1200,1245,1300,1400};

uint16_t timezoneIndex = 14; //+00:00
int temp_tzIndex = 14; //+00:00
bool isChecked = false;
bool tempChecked = false;
int temp_alarm_hour = 0; //temporary time
int temp_alarm_minute = 0;
int alarm_hour = 0;
int alarm_minute = 0;

// touch coordinates
uint16_t xC = 0;
uint16_t yC = 0;

int screen = 1; // which screen unPhone displays first
int lastScreen = 0;
int lastMin = 0;

#define NTP_OFFSET  0 // In seconds
#define NTP_INTERVAL 60000    // In miliseconds
#define NTP_ADDRESS  "1.europe.pool.ntp.org"

// the LCD and touch screen
#define TFT_DC   33
Adafruit_STMPE610 ts = Adafruit_STMPE610(IOExpander::TOUCH_CS);

// calibration data for converting raw touch data to the screen coordinates
#define TS_MINX 3800
#define TS_MAXX 100
#define TS_MINY 100
#define TS_MAXY 3750

// NTP server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);
//delay values (e.g loadtime,etc)

// SETUP: initialisation entry point /////////////////////////////////////////
void setup() {
  UNPHONE_DBG = true;
  unPhone::begin();
  // power management
  unPhone::printWakeupReason(); // what woke us up?
  unPhone::checkPowerSwitch();  // if power switch is off, shutdown

  //set up LED pixels and touch screen
  pixels.begin();
  TS_Point p = ts.getPoint();
  if(! ts.begin()) {
      Serial.println("STMPE NOT FOUND!");
      while(1);
    }

  Serial.printf("Hello from DawnSimAlarm...\n");
  Serial.printf("Version 33\n");
  delay(2000);

  getMAC(MAC_ADDRESS);          // store the MAC address
  apSSID.concat(MAC_ADDRESS);   // add the MAC to the AP SSID
  Serial.printf("\nsetup...\nESP32 MAC = %s\n", MAC_ADDRESS);

  //Inform user of current process
  wifiScreen();

  //Wifi Provisioning
  Serial.printf("doing wifi manager\n");
  bool connected = joinmeManageWiFi(apSSID.c_str(), apPassword.c_str()); // get net connection
  Serial.printf("wifi manager done\n\n");

  //Display Privisioning Feedback to User
  if (connected) {
    printMessage("Wifi Connected!",30,300,3,HX8357_GREEN); // say Wifi Connected
  } else {
    printMessage("Failed to Connect...",45,300,2,HX8357_RED);
    printMessage("Check your Credentials...",12,350,2,HX8357_RED);
  }

  delay(3000);

  //Feedback to user about OTA Updates
  unPhone::tftp->fillScreen(HX8357_WHITE);
  printMessage("Checking for updates", 45 , 100 , 2 ,HX8357_BLACK);

  //OTA Updates
  Serial.printf("firmware is at version %d\n", firmwareVersion);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // let wifi settle
  joinmeOTAUpdate(
    firmwareVersion, _GITLAB_PROJ_ID,
    // "", // for publ repo "" works, else need valid PAT: _GITLAB_TOKEN,
    _GITLAB_TOKEN,
    "MyProjectThing%2Ffirmware%2F"
  );

  delay(300); printf("\n"); delay(500); printf("\n");

  // LoRaWAN example
  // if(false) loraMessage();

  Serial.printf("battery voltage = %3.3f\n", unPhone::batteryVoltage());

  //get time
  timeClient.begin();
  timeClient.update();

}

void loop() {
  micros(); // update overflow

  //Get Current Time
  timeClient.update();
  int curHour = timeClient.getHours();
  int curMin = timeClient.getMinutes();

  //Initiate Alarm if ready
  if ((curHour == alarm_hour) && (curMin == alarm_minute) && (timeClient.getSeconds()<2)) {
    screen = 3;
  }

  //Display Appropriate Screen
  unPhone::checkPowerSwitch();
  if (lastScreen != screen) {
    if (screen == 1) {
      clear();
      homeScreen();
    } else if (screen == 2) {
      clear();
      setAlarmScreen();
    } else if (screen == 3) {
      alarmScreen();
      dawnSim();
    } else if (screen == 4) {
      clear();
      snoozeScreen();
    } else if (screen == 5) {
      clear();
      timeZoneScreen();
    }
    lastScreen = screen;
  }

  //Update time on screen if minute has passed
  if ((screen == 1) && (lastMin != curMin)){
    homeScreen();
    lastMin = curMin;
  }

  //Record touch readings
  while (!ts.bufferEmpty()) {
    // retrieve a point
    TS_Point p = ts.getPoint();
    p.x = map(p.x, TS_MAXX, TS_MINX, HX8357_TFTWIDTH, 0);
    p.y = map(p.y, TS_MAXY, TS_MINY, 0, HX8357_TFTHEIGHT);

    xC = p.x;
    yC = p.y;
  }

  delay(100);

  //Actions for touch coordinates
  if (ts.bufferSize() == 0) { //Home Screen
    if (screen == 1) {
      //Click set alarm
      if (xC >24 & xC<296 & yC >354 & yC<456) {
        screen = 2;
      }
      //Click set time zone
      else if (xC >24 & xC<296 & yC >234 & yC<336) {
        screen = 5;
      }
    } else if (screen == 2) { //SetAlarmScreen
      //home button
      if (xC >19 & xC<106 & yC >349 & yC<461) {
        screen = 1;
        temp_alarm_hour = alarm_hour;
        temp_alarm_minute = alarm_minute;
      }
      //save alarm btn
      else if (xC >114 & xC<301 & yC >349 & yC<461) {
        screen = 1;
        alarm_hour = temp_alarm_hour;
        alarm_minute = temp_alarm_minute;
        Serial.println("New time");
        Serial.println(alarm_hour);
        Serial.println(alarm_minute);
      }
      //top left arrow
      else if (xC >59 & xC<116 & yC >79 & yC<151) {
        temp_alarm_hour -= 1;
        updateTime();
        setAlarmScreen();
      }
      //bottom left arrow
      else if (xC >59 & xC<116 & yC >219 & yC<291) {
        temp_alarm_hour += 1;
        updateTime();
        setAlarmScreen();
      }
      //top right arrow
      else if (xC >219 & xC<276 & yC >79 & yC<151) {
        temp_alarm_minute -= 1;
        updateTime();
        setAlarmScreen();
      }
      //bottom right arrow
      else if (xC >219 & xC<276 & yC >219 & yC<291) {
        temp_alarm_minute += 1;
        updateTime();
        setAlarmScreen();
      }
    } else if (screen == 3) { //Alarm Screen
      //Off Button
      if (xC >79 & xC<241 & yC >319 & yC<431) {
        clear();
        screen = 1;
      }
      //snooze button
      else {
        clear();
        screen = 4;
        alarm_minute = curMin + 2;
        updateTime();
      }
    } else if (screen == 4) { //Snooze Screen
      //Off Button
      if (xC >14 & xC<306 & yC >289 & yC<461) {
        screen = 1;
        alarm_minute -= 2;
        updateTime();
      }
    } else if (screen == 5) { //SetTimeZone
      //home btn
      if (xC >19 & xC<106 & yC >349 & yC<461) {
        screen = 1;
        temp_tzIndex = timezoneIndex;
        tempChecked = isChecked;
      }
      //save zone btn
      else if (xC >114 & xC<301 & yC >349 & yC<461) {
        screen = 1;
        timezoneIndex = temp_tzIndex;
        isChecked = tempChecked;
        alterTimeZone();
        Serial.println("New time zone");
        Serial.println(timezone[timezoneIndex]);
      }
      //top arrow
      else if (xC > 124 & xC<181 & yC >74 & yC<151) {
        temp_tzIndex -= 1;
        if (temp_tzIndex < 0){
          temp_tzIndex = 37;
        }
        timeZoneScreen();
      }
      //bottom arrow
      else if (xC > 124 & xC<181 & yC >209 & yC<286) {
        temp_tzIndex += 1;
        if (temp_tzIndex == 38){
          temp_tzIndex = 0;
        }
        timeZoneScreen();
      }
      //Daylight Saving Checkbox
      else if (xC > 29 & xC < 76 & yC > 294 & yC < 341) {
        tempChecked = !tempChecked;
        timeZoneScreen();
      }
    }
    //reset touch coordinates
    xC = 0;
    yC = 0;
  }

  // allow the protocol CPU IDLE task to run periodically
  if(loopIter % 2500 == 0) {
    if(loopIter % 25000 == 0)
      D("completed loop %d, yielding 1000th time since last\n", loopIter)
    delay(100); // 100 appears min to allow IDLE task to fire
  }
  loopIter++;
}

// misc utilities ////////////////////////////////////////////////////////////
// get the ESP's MAC address
char *getMAC(char *buf) { // the MAC is 6 bytes; needs careful conversion...
  uint64_t mac = ESP.getEfuseMac(); // ...to string (high 2, low 4):
  char rev[13];
  sprintf(rev, "%04X%08X", (uint16_t) (mac >> 32), (uint32_t) mac);

  // the byte order in the ESP has to be reversed relative to normal Arduino
  for(int i=0, j=11; i<=10; i+=2, j-=2) {
    buf[i] = rev[j - 1];
    buf[i + 1] = rev[j];
  }
  buf[12] = '\0';
  return buf;
}

// message on LCD
void lcdMessage(char *s) {
  unPhone::tftp->setCursor(0, 465);
  unPhone::tftp->setTextSize(2);
  unPhone::tftp->setTextColor(HX8357_CYAN, HX8357_BLACK);
  unPhone::tftp->print("                          ");
  unPhone::tftp->setCursor(0, 465);
  unPhone::tftp->print(s);
}

//GUI for TimeZoneScreen
void timeZoneScreen(){
  unPhone::tftp->fillScreen(HX8357_WHITE);
   //Title
   printMessage("Set Time Zone",40,15,3,HX8357_BLACK); // print set alarm time
   printMessage("UTC: +00:00",60, 50,3,HX8357_BLACK); // print set alarm time

   //Time Feedback
   int zone = timezone[temp_tzIndex];
   if (tempChecked) {
    zone += 100;
   }
   char symbol = '+';
   if (zone < 0) {
    symbol = '-';
    zone -= 2*zone;
   }
   unPhone::tftp->drawChar(10, 150, symbol, HX8357_BLACK, HX8357_WHITE, 8);
   char digit1 = (char)(((zone / 100) / 10)+48);
   char digit2 = (char)(((zone / 100) % 10)+48);
   unPhone::tftp->drawChar(60, 150, digit1, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(110,150, digit2, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(160,150, ':', HX8357_BLACK, HX8357_WHITE, 8);
   digit1 = (char)(((zone % 100) / 10)+48);
   digit2 = (char)(((zone % 100) % 10)+48);
   unPhone::tftp->drawChar(210, 150, digit1, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(260, 150, digit2, HX8357_BLACK, HX8357_WHITE, 8);

   //Arrows
   unPhone::tftp->drawChar(130, 75, 30, HX8357_BLACK, HX8357_WHITE, 9);
   unPhone::tftp->drawChar(130, 215, 31, HX8357_BLACK, HX8357_WHITE, 9);

   //Daylight Saving Button
   if (tempChecked) {
    unPhone::tftp->fillRect(30, 295, 45, 45, HX8357_BLACK);
    printMessage("x", 35 , 286 , 7 ,HX8357_WHITE);
   } else {
    unPhone::tftp->drawRect(30, 295, 45, 45, HX8357_BLACK);
   }

   printMessage("Daylight Savings?", 85 , 310 , 2 ,HX8357_BLACK);


   //Home Button
   unPhone::tftp->drawChar(20, 350, 127, HX8357_BLACK, HX8357_WHITE, 15);

   //Save button
   unPhone::tftp->fillRect(115,350,185,90,HX8357_GREEN);
   printMessage("Set Zone", 135 , 385 , 3 ,HX8357_BLACK);
}

//GUI for snoozeScreen
void snoozeScreen() {
  unPhone::tftp->fillScreen(HX8357_WHITE);
  unPhone::tftp->fillRect(15,290,290,170,HX8357_BLACK);
  printMessage("OFF", 100 , 350 , 7 ,HX8357_WHITE);
}

//GUI for alarmScreen
void alarmScreen() {
  unPhone::tftp->fillScreen(HX8357_RED);
  unPhone::tftp->fillRect(80,320,160,110,HX8357_BLACK);
  printMessage("SNOOZE", 40 , 125, 7 ,HX8357_WHITE);
  printMessage("OFF", 100 , 350 , 7 ,HX8357_WHITE);
}

//GUI for setAlarmScreen
void setAlarmScreen() { // TIDY THIS
   unPhone::tftp->fillScreen(HX8357_WHITE);
   //Title
   printMessage("Set Alarm Time",40,15,3,HX8357_BLACK); // print set alarm time

   //Time Feedback
   char digit1 = (char)((temp_alarm_hour / 10)+48);
   char digit2 = (char)((temp_alarm_hour % 10)+48);
   unPhone::tftp->drawChar(40, 150, digit1, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(90,150, digit2, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(140,150, ':', HX8357_BLACK, HX8357_WHITE, 8);
   digit1 = (char)((temp_alarm_minute / 10)+48);
   digit2 = (char)((temp_alarm_minute % 10)+48);
   unPhone::tftp->drawChar(190, 150, digit1, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(240, 150, digit2, HX8357_BLACK, HX8357_WHITE, 8);

   //Arrows
   unPhone::tftp->drawChar(65, 80, 30, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(215, 80, 30, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(65, 220, 31, HX8357_BLACK, HX8357_WHITE, 8);
   unPhone::tftp->drawChar(215, 220, 31, HX8357_BLACK, HX8357_WHITE, 8);

   //Home Button
   unPhone::tftp->drawChar(20, 350, 127, HX8357_BLACK, HX8357_WHITE, 15);

   //Save button
   unPhone::tftp->fillRect(115,350,185,90,HX8357_GREEN);
   printMessage("Set Alarm", 130 , 385 , 3 ,HX8357_BLACK);
}

//GUI for homeScreen
void homeScreen(){
  unPhone::tftp->fillScreen(HX8357_WHITE);

  //Title
  printMessage("Dawn Simulator",40,15,3,HX8357_BLACK);

  //Current time feedback
  char digit1 = (char)((timeClient.getHours() / 10)+48);
  char digit2 = (char)((timeClient.getHours() % 10)+48);
  unPhone::tftp->drawChar(40, 110, digit1, HX8357_BLACK, HX8357_WHITE, 8);
  unPhone::tftp->drawChar(90, 110, digit2, HX8357_BLACK, HX8357_WHITE, 8);
  unPhone::tftp->drawChar(140,110, ':', HX8357_BLACK, HX8357_WHITE, 8);
  digit1 = (char)((timeClient.getMinutes() / 10)+48);
  digit2 = (char)((timeClient.getMinutes() % 10)+48);
  unPhone::tftp->drawChar(190, 110, digit1, HX8357_BLACK, HX8357_WHITE, 8);
  unPhone::tftp->drawChar(240, 110, digit2, HX8357_BLACK, HX8357_WHITE, 8);

  //Navigation Buttons
  unPhone::tftp->fillRect(25,235,270,100,HX8357_BLUE);
  unPhone::tftp->fillRect(25,355,270,100,HX8357_RED);
  printMessage("Set Time Zone", 80 , 280 , 2 ,HX8357_WHITE);
  printMessage("Set Alarm Time",75, 400, 2 ,HX8357_WHITE);
}

//User feedback for WIFI connection
void wifiScreen() {
  unPhone::tftp->fillScreen(HX8357_WHITE);
  printMessage("Connecting to Wifi...",35,100,2,HX8357_BLACK); // say Connecti0ng to wifi
  printMessage("Do WiFi provisioning",42,150,2,HX8357_BLACK);
  printMessage("if this takes a while",35,200,2,HX8357_BLACK);
}

void printMessage(char *s, uint16_t x, uint16_t y, uint16_t fontSize, uint16_t txtColor) {
  unPhone::tftp->setCursor(x, y);
  unPhone::tftp->setTextSize(fontSize);
  unPhone::tftp->setTextColor(txtColor);
  unPhone::tftp->print("                          ");
  unPhone::tftp->setCursor(x, y);
  unPhone::tftp->print(s);
}

//LED pixel alarm
void dawnSim() {
  clear();
  delay(2000);

  //set all to dark blue
  pixels.setBrightness(20);
  pixels.fill(pixels.Color(0, 0, 20),0,31);
  pixels.show();

  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //First Fade
  fadeDarkBlueToFirst();
  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //Second Fade
  fadeFirstToSecond();
  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //3rd Fade
  fadeSecondToThird();
  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //Fourth Fade
  fadeThirdToFourth();
  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //Fifth Fade
  fadeFourthToFifth();
  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //Sixth Fade
  fadeFifthToSixth();
  delay(2000);
  //check for touch
  if (ts.bufferSize() != 0){
    clear();
    return;
  }

  //Final Fade to full white
  fadeSixthToFinal();
  do { //Wait until touch
    delay(100);
  } while (ts.bufferSize() == 0);
  clear();
}

//Fade Functions
void fadeDarkBlueToFirst() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(inner,2,0,0,20,0,0,int(i*darkToLightBlueInc));
    pixels.show();
    delay(50);
  }
}
void fadeFirstToSecond() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(inner_middle,6,0,0,20,0,0,int(i*darkToLightBlueInc));
    pixelsFade(inner,2,0,0,100,int(i * lightToOrangeRedInc),int(i * lightToOrangeGreenInc),int(i * lightToOrangeBlueInc));
    pixels.show();
    delay(50);
  }
}

void fadeSecondToThird() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(outer_middle,10,0,0,20,0,0,int(i*darkToLightBlueInc));
    pixelsFade(inner_middle,6,0,0,100,int(i * lightToOrangeRedInc),int(i * lightToOrangeGreenInc),int(i * lightToOrangeBlueInc));
    pixelsFade(inner,2,255,60,0,0,int(i * orangeToYellowGreenInc),0);
    pixels.show();
    delay(50);
  }
}

void fadeThirdToFourth() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(outer,14,0,0,20,0,0,int(i*darkToLightBlueInc));
    pixelsFade(outer_middle,10,0,0,100,int(i * lightToOrangeRedInc),int(i * lightToOrangeGreenInc),int(i * lightToOrangeBlueInc));
    pixelsFade(inner_middle,6,255,60,0,0,int(i * orangeToYellowGreenInc),0);
    pixelsFade(inner,2,255,255,0,0,0,int(i * yellowToWhiteBlueInc));
    pixels.show();
    delay(50);
  }
}

void fadeFourthToFifth() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(outer,14,0,0,100,int(i * lightToOrangeRedInc),int(i * lightToOrangeGreenInc),int(i * lightToOrangeBlueInc));
    pixelsFade(outer_middle,10,255,60,0,0,int(i * orangeToYellowGreenInc),0);
    pixelsFade(inner_middle,6,255,255,0,0,0,int(i * yellowToWhiteBlueInc));
    pixels.show();
    delay(50);
  }
}

void fadeFifthToSixth() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(outer,14,255,60,0,0,int(i * orangeToYellowGreenInc),0);
    pixelsFade(outer_middle,10,255,255,0,0,0,int(i * yellowToWhiteBlueInc));
    pixels.show();
    delay(50);
  }
}

void fadeSixthToFinal() {
  for(int i=1; i<=GLOBAL_STEPS; i++) {
    pixelsFade(outer,14,255,255,0,0,0,int(i * yellowToWhiteBlueInc));
    pixels.show();
    delay(50);
  }
}

//Correct Time format
void updateTime() {
  //alarm min
  if (alarm_minute > 59) {
    alarm_minute = alarm_minute % 60;
    alarm_hour += 1;
  } else if (alarm_minute < 0) {
    alarm_minute = (alarm_minute % 60) + 60;
    alarm_hour -= 1;
  }
  //alarm hour
  alarm_hour = alarm_hour % 24;
  if (alarm_hour < 0) {
    alarm_hour += 24;
  }
  //temp min
  temp_alarm_minute = temp_alarm_minute % 60;
  if (temp_alarm_minute < 0) {
    temp_alarm_minute += 60;
  }
  //temp hour
  temp_alarm_hour = temp_alarm_hour % 24;
  if (temp_alarm_hour < 0) {
    temp_alarm_hour += 24;
  }
}

//Calculation increments for fading
double calcInc(double start, double finish) {
    return (finish-start)/GLOBAL_STEPS;
  }

//Implement selected time zone
void alterTimeZone() {
  int z = timezone[timezoneIndex];
  int zHours = z / 100;
  int zMins = z % 100;
  int offset = (zHours * 3600) + (zMins * 60);
  if (isChecked) {
    offset += 3600;
  }
  timeClient.setTimeOffset(offset);
}

void clear() { // set all pixels to black
  for (int i = 0; i < NUMPIXELS; i++)
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  pixels.show();
  for (int i = NUMPIXELS - 1; i >= 0; i--)
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  pixels.show();
}

//Fade pixels in passed array by passed values
void pixelsFade(uint16_t *pixelArray, int size, int baseR, int baseG, int baseB, int incR, int incG, int incB){
  for (int i = 0; i < size; i++) {
    pixels.setPixelColor(pixelArray[i],pixels.Color(baseR + incR, baseG + incG, baseB + incB));
  }
}
