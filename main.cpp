#define BLYNK_PRINT Serial
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <AceButton.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "utilities.h"
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include "index.h"

#define M_PI 3.14159265358979323846264338327950288
const double fences[1][10][2] = {{{10.776483, 106.574757},
                                  {10.776454, 106.575447},
                                  {10.775522, 106.575229},
                                  {10.775733, 106.574491},
                                  {10.775953, 106.574554},
                                  {10.776759, 106.575219},
                                  {10.775976, 106.575826},
                                  {10.775592, 106.575722},
                                  {10.775215, 106.574754},
                                  {10.776580, 106.576113},}
};
/*Variables to store AP credentials*/
String ssid = "Phat";
String password = "0937875855";
int WiFiConnectMode = 1; /* 0: smart config; 1: hard code*/
double vido1, kinhdo1;
int sat;
String date;
char lati[12];
char longi[12];
int targetStatus;
int fence;
char cumulativeAngle[12];
int deviceStatus = 0;
ESP8266WebServer gpsServer(80);
void connectWifi();
void updateLatLon();
void pip();
void handleRoot();
void fenceSelect();
void gps_data();

SoftwareSerial SerialAT(13, 15); // RX, TX d7 d8  
#define rxGPS D5
#define txGPS D6 
SoftwareSerial mygps(rxGPS,txGPS);
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 //Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

using namespace ace_button;
#define button1 16 
bool button_State;

#define SMS_Button  2
#define Call_Button 10
String message = "The Best  ";
String mobile_number = "+84937875855";

String message_with_data;
ButtonConfig config1;
AceButton call_button(&config1);
ButtonConfig config2;
AceButton sms_button(&config2);

void handleEvent_call(AceButton*, uint8_t, uint8_t);
void handleEvent_sms(AceButton*, uint8_t, uint8_t);
#define SerialMon Serial

char auth[] = "ZiW0-16cRnLFoxGrWlpbFks4P8y1oVPQ";
char apn[]  = "internet";
char user[] = "";
char pass[] = "";



TinyGPSPlus gps;  
boolean send_alert_once = true;
boolean multiple_sms = false;
unsigned long previousMillis=0;
unsigned long int previoussecs = 0; 
unsigned long int currentsecs = 0; 
 unsigned long currentMillis = 0;
 int secs = 0; 
 int pmints = 0; 
 int mints = 0; // current mints
 int interval= 1 ;
 
 
const float maxDistance = 0;
float initialLatitude = 10.776258;
float initialLongitude = 106.575406;
float latitude, longitude;
float vido, kinhdo;
void getGps(float& vido, float& kinhdo);
WidgetMap myMap(V0);
 
BlynkTimer timer;

float velocity;     //Variable  to store the velocity
float sats;         //Variable to store no. of satellites response
String bearing;
unsigned int move_index = 1; 
void print_speed(){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
   if (gps.location.isValid() == 1){
    display.setTextSize(1);
    
    display.setCursor(25, 5);
    display.print("Vdo: ");
    display.setCursor(50, 5);
    display.print(gps.location.lat(),6);
    display.setCursor(25, 20);
    display.print("KDo: ");
    display.setCursor(50, 20);
    display.print(gps.location.lng(),6);
    display.setCursor(25, 35);
    display.print("TDo: ");
    display.setCursor(65, 35);
    display.print(gps.speed.kmph());
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("VTinh:");
    display.setCursor(25, 50);
    display.print(gps.satellites.value());
    display.setTextSize(1);
    display.setCursor(70, 50);
    display.print("DoCao:");
    display.setCursor(95, 50);
    display.print(gps.altitude.meters(), 0);
    display.display();
   } else{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();

   }
}




void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    Blynk.virtualWrite(V3, "GPS ERROR");  // Value Display widget  on V3 if GPS not detected
  }
}
TinyGsm modem(SerialAT);
  
  
void setup()
{
  pinMode(SMS_Button, INPUT);
  pinMode(Call_Button, INPUT);
  pinMode(button1, INPUT);
  Serial.begin(9600);
  delay(10);
  SerialAT.begin(9600);
  delay(3000);
  Serial.println("dang chay ...");
  modem.restart();
   Blynk.begin(auth,modem, apn, user, pass,"blynk.iot-cm.com",8080);
   mygps.begin(9600);
   connectWifi();

    gpsServer.on("/", handleRoot);
    gpsServer.on("/status", fenceSelect);
    gpsServer.on("/values", gps_data);
    gpsServer.begin();
    
   timer.setInterval(5000L, checkGPS);
   config1.setEventHandler(handleEvent_call);
  config2.setEventHandler(handleEvent_sms);
   call_button.init(Call_Button);
   sms_button.init(SMS_Button);
   SerialAT.println("AT"); //Check GSM Module
   delay(1000);
   SerialAT.println("ATE1"); //Echo ON
   delay(1000);
   SerialAT.println("AT+CPIN?"); //Check SIM ready
   delay(1000);
   SerialAT.println("AT+CMGF=1"); //SMS text mode
   delay(1000);
   SerialAT.println("AT+CNMI=1,1,0,0,0");
   delay(1000);
  
   if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
    
  }
  display.clearDisplay();
  display.display();
  delay(2000);
  
   
}

float getDistance(float flat1, float flon1, float flat2, float flon2) {

  // Variables
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
}

void getGps(float& vido, float& kinhdo)
{
  // Can take up to 60 seconds
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;){
    while (mygps.available()){
      if (gps.encode(mygps.read())){
        newData = true;
        break;
      }
    }
  }
   if (newData) //If newData is true
  {
    vido = gps.location.lat(),6;
    kinhdo = gps.location.lng(),6;
    newData = false;
  }
  else {
    Serial.println("No GPS data is available");
    vido = 0;
    kinhdo = 0;
  }
}
void sendAlert()
{
  //return;
  String sms_data;
  sms_data = "BAN DA RA NGOAI HANH RAO HAVERSINE \r";
  sms_data += "http://www.google.com/maps/place/";
  sms_data += String(gps.location.lat(),6) + "," + String(gps.location.lng(),6);

  //return;
  SerialAT.print("AT+CMGF=1\r");
  delay(1000);
  SerialAT.print("AT+CMGS=\""+mobile_number+"\"\r");
  delay(1000);
  SerialAT.print(sms_data);
  delay(100);
  SerialAT.write(0x1A); //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully.");
  
}
void sendphat()
{
  //return;
  String sms_data;
  sms_data = "TOI CAN GIUP DO HELP ME !!! SOS .\r";
  sms_data += "http://www.google.com/maps/place/";
  sms_data += String(gps.location.lat(),6) + "," + String(gps.location.lng(),6);
 
  //return;
  SerialAT.print("AT+CMGF=1\r");
  delay(1000);
  SerialAT.print("AT+CMGS=\"+84937875855\"\r"); 
  delay(1000);
  SerialAT.print(sms_data);
  delay(100);
  SerialAT.write(0x1A); //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully.");
  
}


void displayInfo()
{
  if (gps.location.isValid() )
  {
    sats = gps.satellites.value();       //get number of satellites
    latitude = (gps.location.lat(),6);     //Storing the Lat. and Lon.
    longitude = (gps.location.lng(),6);
    velocity = gps.speed.kmph();         //get velocity
    bearing = TinyGPSPlus::cardinal(gps.course.value());     // get the direction
 
    Serial.print("SATS:  ");
    Serial.println(sats);  // float to x decimal places
    Serial.print("LATITUDE:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONGITUDE: ");
    Serial.println(longitude, 6);
    Serial.print("SPEED: ");
    Serial.print(velocity);
    Serial.println("kmph");
    Serial.print("DIRECTION: ");
    Serial.println(bearing);
 
    Blynk.virtualWrite(V1, String(latitude, 6));
    Blynk.virtualWrite(V2, String(longitude, 6));
    Blynk.virtualWrite(V3, sats);
    Blynk.virtualWrite(V4, velocity);
    Blynk.virtualWrite(V5, bearing);
    myMap.location(move_index, latitude, longitude, "GPS_Location");
  }
  Serial.println();
}

void loop()
{
  getGps(vido, kinhdo);
  float distance = getDistance(vido, kinhdo, initialLatitude, initialLongitude);
  Serial.print("Latitude= "); Serial.println(vido, 6);
  Serial.print("Lngitude= "); Serial.println(kinhdo, 6);
  Serial.print("initialLatitude= "); Serial.println(initialLatitude, 6);
  Serial.print("initialLngitude= "); Serial.println(initialLongitude, 6);
  Serial.print("current Distance= "); Serial.println(distance);
  if(distance > maxDistance){
    multiple_sms = true;
    if(send_alert_once == true){
      sendAlert();
     
      send_alert_once = false;
      
    }
  }
  else{
    send_alert_once = true;
    multiple_sms = false;
   
  }
  
 
  
   while(SerialAT.available()){
    Serial.println(SerialAT.readString());
  }
  while(Serial.available())  {
    SerialAT.println(Serial.readString());
  }
  
  while (mygps.available() > 0)
  {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      displayInfo();
  }
  Blynk.run();
  timer.run();
   sms_button.check();
  call_button.check();
  
  
  
  
  ///
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (mygps.available())
    {
      if (gps.encode(mygps.read()))
      {
        newData = true;
      }
    }
  }
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
    print_speed();
  }
  else
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
  } 
   button_State = digitalRead(button1);   //We are constantly reading the button State
 
  if (button_State == HIGH) {            //And if it's pressed
    Serial.println("Button pressed");   //Shows this message on the serial monitor
    delay(200);                         //Small delay to avoid detecting the button press many times
    
    sendphat();                          //And this function is called

 }
 
  if (SerialAT.available()){            //Displays on the serial monitor if there's a communication from the module
    Serial.write(SerialAT.read()); 
  }
  
  while (mygps.available()){
        deviceStatus = 1;
        updateLatLon();
        pip();        
        gpsServer.handleClient();
    }
    gpsServer.handleClient();
  String msg=Serial.readStringUntil('\r');
  Serial.println(msg);
  
}
void connectWifi(){
    if(WiFiConnectMode == 0){
        // Operate the ESP12E in wifi station mode for smart config
        WiFi.mode(WIFI_STA);

        // Begin the smart configuration to get the Access Point credentials
        WiFi.beginSmartConfig();
        Serial.println("------------------------------------------------");
        Serial.print("Waiting for SmartConfig ");
        while (!WiFi.smartConfigDone()) {
          delay(250);
            Serial.print(".");
        }
        Serial.println();
        Serial.println("SmartConfig done.");

        // Print the AP credentials to the serial monitor
        ssid = WiFi.SSID();
        password = WiFi.psk();
        Serial.println("------------------------------------------------");
        Serial.print("Acesspoint SSID : ");
        Serial.println(ssid);
        Serial.print("Acesspoint password : ");
        Serial.println(password);
        Serial.println("------------------------------------------------");

        // Connect the ESP12E to the AP
        Serial.print("Connecting to Access Point ");
        while (WiFi.status() != WL_CONNECTED) {
            delay(100);
            Serial.print(".");
        }
        Serial.println();
        Serial.println("Connected.");
        Serial.println("------------------------------------------------");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.println("------------------------------------------------");
    }
    else{
        String ssid = "Phat"; //Access point ssid
        String password = "0937875855"; //Access point password
        WiFi.begin(ssid,password);
        Serial.println("------------------------------------------------");
        Serial.print("Connecting to Access Point ");
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println("------------------------------------------------");
    }
}

void updateLatLon(){
    if (gps.encode(mygps.read()))
    {  
      sat = gps.satellites.value();
      vido1 = gps.location.lat();
      kinhdo1 = gps.location.lng();
      dtostrf(vido1,9,7,lati);
      dtostrf(kinhdo1,9,7,longi);
      Serial.print("SATS: ");
      Serial.println(sat);
      Serial.print("LAT: ");
      Serial.println(vido1,6);
      Serial.print("LONG: ");
      Serial.println(kinhdo1,6);
      Serial.print("ALT: ");
      Serial.println(gps.altitude.meters());
      Serial.print("SPEED: ");
      Serial.println(gps.speed.mps());
 
      Serial.print("Date: ");
      date = String(gps.date.day())+"/"+gps.date.month()+"/"+gps.date.year();
      Serial.println(date);
 
      Serial.print("Hour: ");
      Serial.print(gps.time.hour()); Serial.print(":");
      Serial.print(gps.time.minute()); Serial.print(":");
      Serial.println(gps.time.second());
      Serial.println("---------------------------");
      delay(1000);
    }
}
void pip(){
    int fenceSize = sizeof(fences[fence - 1])/sizeof(fences[fence - 1][0]);
    double vectors[fenceSize][2];
    for(int i = 0; i < fenceSize; i++){
        vectors[i][0] = fences[fence - 1][i][0] - vido1;
        vectors[i][1] = fences[fence - 1][i][1] - kinhdo1;
    }
    double angle = 0;
    double num, den;
    for(int i = 0; i < fenceSize; i++){
      num = (vectors[i%fenceSize][0])*(vectors[(i+1)%fenceSize][0])+ (vectors[i%fenceSize][1])*(vectors[(i+1)%fenceSize][1]);
        den = (sqrt(pow(vectors[i%fenceSize][0],2) + pow(vectors[i%fenceSize][1],2)))*(sqrt(pow(vectors[(i+1)%fenceSize][0],2) + pow(vectors[(i+1)%fenceSize][1],2)));
        angle = angle + (180*acos(num/den)/M_PI);
    }
    dtostrf(angle,9,7,cumulativeAngle);
    if(angle > 355 && angle < 365)
        targetStatus = 1;
    else
        targetStatus = 0;
}
void handleRoot(){
    String s = webpage;
    gpsServer.send(200, "text/html", s);
}

void fenceSelect(){
    fence = gpsServer.arg("fenceValue").toInt();
    gpsServer.send(200, "text/plane", String(fence));
}

void gps_data(){
    String payload = String(sat) + "#" + date + "#" + lati + "#" + longi;
    if(targetStatus == 0)
        payload = payload + "#outside";
    else
        payload = payload + "#inside";
    payload = payload + "#" + cumulativeAngle;
    if(deviceStatus == 0)
        payload = payload + "#offline";
    else
        payload = payload + "#online";
    gpsServer.send(200, "text/plane", payload);
}

void handleEvent_sms(AceButton* /* button */, uint8_t eventType,
                     uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      // Serial.println("kEventPressed");
      message_with_data = message + "Latitude = " + (String)latitude + "Longitude = " + (String)longitude;
      modem.sendSMS(mobile_number, message_with_data);
      message_with_data = "";
      break;
    case AceButton::kEventReleased:
      //Serial.println("kEventReleased");
      break;
  }
}
void handleEvent_call(AceButton* /* button */, uint8_t eventType,
uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      // Serial.println("kEventPressed");
      modem.callNumber(mobile_number);
      break;
    case AceButton::kEventReleased:
      //Serial.println("kEventReleased");
      break;
  }
}