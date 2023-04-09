#define ARDUINOJSON_DECODE_UNICODE 0 

#include <stdint.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>

//#include "HUSKYLENS.h"

#if defined(ARDUINO_AVR_UNO)
SoftwareSerial Serial1(2, 3);
#endif


StaticJsonDocument<256> myStateDoc;
JsonObject myState;




#define DEBUG_DELAY          0x7F  
#define VEHICLE_LIGHT_STEP   0x40
#define JOYSTICK_DATA_LENGTH 0xFF
#define JOYSTICK_DATA_START  0x11
#define JOYSTICK_DATA_START0 0x22
#define JOYSTICK_DATA_END    0x23
#define JOYSTICK_DATA_END0   0x12

int E1 = 4;    //PLL based M1 Speed Control
int E2 = 7;    //PLL based M2 Speed Control
int M1 = 5;    //PLL based M1 Direction Control
int M2 = 6;    //PLL based M2 Direction Control
int R1 = A7;   //Servo Motor Pin

#if defined(ARDUINO_AVR_UNO)
int L1 = 11;    //Front Light Control
int L2 = 10;   //Rare Light Control
int I1 = 8;
int O1 = 9;
#endif


#if defined(ARDUINO_AVR_MEGA2560)
int L1 = 14;    //Front Light Control
int L2 = 15;   //Rare Light Control
int I1 = 8;
int O1 = 9;
#endif


//Stream* _cam;
Stream* _log;
Stream* _pad;

Servo servo;

// General State Parts
bool useCam        = false;
bool useLogs       = true;
bool useDelay      = false;
bool useHusky      = false;

// Joystick Buttons State Parts 
bool pressed1      = false;
bool pressed2      = false;
bool pressed3      = false;
bool pressed4      = false;
bool pressedU      = false;
bool pressedL      = false;
bool pressedR      = false;
bool pressedD      = false;
bool pressedL1     = false;
bool pressedL2     = false;
bool pressedR1     = false;
bool pressedR2     = false;
bool pressedStart  = false;
bool pressedSelect = false;

uint8_t driveMode = 1;  

String mac = "n/a";// Drive mode 1 - vehicle drives front/back by left joystick, left/right - by right, drive mode 2 - all directions handled by left joystick, drive mode 3 - all directions handled by right joystick
 
// Joystick Message Retrieving Parts   
bool newData       = false;
uint8_t numReceived = 0;
uint8_t receivedBytes[JOYSTICK_DATA_LENGTH];

// Vehicle Speed/State Parts
uint8_t v1 = 0x7F, v2 = 0x7F;
uint8_t _b3 = 0x00, _b4 = 0x00, b3 = 0x00, b4 = 0x00;
uint8_t _l0 = 0x00;

const int huskyPin = O1;   //ledPin
const int buttonPin = I1; //Button to perform interrupt 
int huskyToggle = LOW;     //led state 

//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;  
unsigned long last_button_time = 0; 

void driveMotor(byte m1p, byte m2p)
{   
    if(useLogs) 
    {        
      String m = "DriveMotor["; m += String(driveMode); m += "]: "; m += String(m1p, HEX); m += ","; m += String(m2p,HEX);
    
      sendLog(m);
      if(useDelay) delay(1000);
    }
    
    int16_t a = m1p>=0x7F ? m1p-0x7F : 0x7F-m1p;
    int16_t b = m2p>=0x7F ? m2p-0x7F : 0x7F-m2p;
    int16_t c = sqrt(a>b ? a*a-b*b : b*b-a*a);
  
    float kX = 1, kY = 1;
    byte x = 0x7F, y = 0x7F, dY = m2p >= 0x7F ? m2p - 0x7F : 0x7F - m2p;
  
    if(dY<=0x0F) 
    {
      kX = m2p > 0x7F ? 1 : 1;
      kY = m2p < 0x7F ? 1 : 1;
    } else if(dY<=0x1F) 
    {
      kX = m2p > 0x7F ? 1 : 0.875;
      kY = m2p < 0x7F ? 1 : 0.875;
    } else if(dY<=0x2F) 
    {
      kX = m2p > 0x7F ? 1 : 0.75;
      kY = m2p < 0x7F ? 1 : 0.75;
    } else if(dY<=0x3F) 
    {
      kX = m2p > 0x7F ? 1 : 0.625;
      kY = m2p < 0x7F ? 1 : 0.625;
    } else if(dY<=0x4F) 
    {
      kX = m2p > 0x7F ? 1 : 0.5;
      kY = m2p < 0x7F ? 1 : 0.5;
    } else if(dY<=0x5F) 
    {
      kX = m2p > 0x7F ? 1 : 0.375;
      kY = m2p < 0x7F ? 1 : 0.375;
    } else if(dY<=0x6F) 
    {
      kX = m2p > 0x7F ? 1 : 0.25;
      kY = m2p < 0x7F ? 1 : 0.25;
    } else if(dY<=0x7F) 
    {
      kX = m2p > 0x7F ? 1 : 0;
      kY = m2p < 0x7F ? 1 : 0;
    }
  
    x = m1p >= 0x7F ? 0x7F + c*kX : 0x7F - c*kX;
    y = m1p >= 0x7F ? 0x7F + c*kY : 0x7F - c*kY;
  
    if(x != 0x7F)
    {
      digitalWrite(E1, HIGH);
      analogWrite(M1, x);
    } else 
    {
      digitalWrite(E1, LOW);
    }
    if(y != 0x7F)
    {
      digitalWrite(E2, HIGH);
      analogWrite(M2, y);
    } else
    {
      digitalWrite(E2, LOW);
    }
}

void light(uint8_t level)
{
    if(useLogs) 
    {        
        String m = "Light: "; m += _l0 > 0   ? "ON" : "OFF"; m += "->"; m += level > 0 ? "ON" : "OFF"; m += ", level: "; m += String(level, HEX);
        
        sendLog(m);
        if(useDelay) delay(DEBUG_DELAY);
    }
   
    byte x = level == 0 ? HIGH : LOW;
    
    digitalWrite(L1, x);
    digitalWrite(L2, x);
    delay(10);
    analogWrite(L1, level);
    analogWrite(L2, level);

    _l0 = level;
}

uint8_t getXMove(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  switch (driveMode)
  {
    case 1:  return b;
    case 2:  return d;
    case 3:  return b;
    default: return b;
  }
}

uint8_t getYMove(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  switch (driveMode)
  {
    case 1:  return c;
    case 2:  return c;
    case 3:  return a;
    default: return c;
  }
}
//
//void readMac() {
//  if(mac == "n/a") {
//      Serial.println(" ");
//      Serial.write("+++"); Serial.println("\r\n"); delay(500);  
//      String z = Serial.readString(); z.trim();
//      //Serial.println(z);
//      Serial1.println(z);
//      //if(z.indexOf("Enter AT Mode") != -1) {
//        Serial.write("AT+MAC=?\r\n");/*Serial.println("\r\n"); */delay(500);  
//        z = Serial.readString(); z.trim();
//        mac = z;
////        type = getType(mac);
//  
//        //Serial.println("AT+MAC=?");
//        //Serial.println(z);
//        Serial1.println(z);
//
//        Serial.write("AT+EXIT\r\n");/*Serial.println("\r\n"); */delay(500);
//    
//        z = Serial.readString(); z.trim();
////        if(z != "OK") { err = true; }
//        //Serial.println("AT+EXIT");
//        //Serial.println(z);
//        Serial1.println(z);
//      //}
//    }
//}
//
//bool _init  =false;
//String getType()
//{
//  readMac();
//  
//  if(mac == "0xC4BE8423427B") return "Car";
//  if(mac == "0xC4BE84201608") return "Tank";
//  if(mac == "0x") return "Tank";
//  if(mac == "0x30E283AD8A78") return "Mega";
//
//  _init = true;
//
//  return "Thing";
//}


void sendMessage(Stream* stream, String m)
{
  int len = m.length();
  byte src[len+0x08];

  m.getBytes(src, len+1);
  stream->write(JOYSTICK_DATA_START);
  // stream->write(JOYSTICK_DATA_START0);
  stream->write(src, len);
  stream->write(JOYSTICK_DATA_END);
  // stream->write(JOYSTICK_DATA_END0);
}

void checkButtons()
{
    if(b4 == _b4 && b3 == _b3) return;

    pressed1      = ((b3 & 0x01) == 0x01);
    pressed2      = ((b3 & 0x04) == 0x04);
    pressed3      = ((b3 & 0x08) == 0x08);
    pressed4      = ((b3 & 0x02) == 0x02);
    pressedL1     = ((b3 & 0x40) == 0x40);
    pressedL2     = ((b3 & 0x80) == 0x80);
    pressedR1     = ((b3 & 0x10) == 0x10);
    pressedR2     = ((b3 & 0x20) == 0x20); 
    pressedU      = ((b4 & 0x10) == 0x10);
    pressedL      = ((b4 & 0x20) == 0x20);
    pressedR      = ((b4 & 0x40) == 0x40);
    pressedD      = ((b4 & 0x80) == 0x80);
    pressedStart  = ((b4 & 0x0B) == 0x0B);
    pressedSelect = ((b4 & 0x07) == 0x07);    
 
    _b3 = b3;
    _b4 = b4;

    if(useLogs) 
    { 
        String m = "Buttons: ";  m += String(b3, HEX);  m += String(b4, HEX);  m += " [";
        
        if(pressedStart)  m += ("START ");
        if(pressedSelect) m += ("SELECT ");
        if(pressedL1)     m += ("L1 ");
        if(pressedL2)     m += ("L2 ");
        if(pressedR1)     m += ("R1 ");
        if(pressedR2)     m += ("R2 ");
        if(pressed1)      m += ("1 ");
        if(pressed2)      m += ("2 ");
        if(pressed3)      m += ("3 ");
        if(pressed4)      m += ("4 ");
        if(pressedU)      m += ("UP ");
        if(pressedL)      m += ("LEFT ");
        if(pressedR)      m += ("RIGHT ");
        if(pressedD)      m += ("DOWN ");
        m += (" ]");
      
        sendLog(m);
        if(useDelay) delay(DEBUG_DELAY);
    }
    
        //Control Drive Mode
        if(pressedSelect && pressed1) driveMode = 1;
        if(pressedSelect && pressed2) driveMode = 2;
        if(pressedSelect && pressed3) driveMode = 3;        
        if(pressedSelect && pressedStart) {
          if(useCam==true) useCam = false; else useCam = true;
        }

        //Control Debug Logging and Delays
        if(pressedR1 && pressed1) useLogs = true;
        if(pressedR2 && pressed1) useLogs = false;
        if(pressedR1 && pressed2) useDelay = true;        
        if(pressedR2 && pressed2) useDelay = false;

        //Control Vehicle Lights
        if(pressedL1 || pressedL2)
        {  
            int16_t l0 = _l0;
            l0 = (pressedL1) ? _l0 + VEHICLE_LIGHT_STEP : l0;  
            l0 = (pressedL2) ? _l0 - VEHICLE_LIGHT_STEP : l0;  
            
            if(l0 < 0) l0 = 0; else if(l0 > 0xFF) l0 = 0xFF;
            
            light(l0); 
        }

        if(pressedL == true || pressedR == true)
        {
            int p1 = pressedR ? -5 : +5;
            moveServo(p1);
        }
}

void receiveBytes(Stream* stream) 
{
    static bool recvInProgress = false;
    static uint8_t lastRb = 0;
    static uint8_t ndx = 0;
    uint8_t rb;   

    while (stream->available() > 0 && newData == false) 
    {
        rb = stream->read(); 
        _log->write(rb);       

        if (recvInProgress == true) 
        {
            if (/*!((rb == JOYSTICK_DATA_END0) &&*/ lastRb == JOYSTICK_DATA_END)
            {
                receivedBytes[ndx] = rb;
                if (ndx++ >= JOYSTICK_DATA_LENGTH) ndx = JOYSTICK_DATA_LENGTH - 1;
            }
            else 
            if (stream->available() /*? stream->read() == JOYSTICK_DATA_ENDLO : false*/)
            {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }
        else if (rb == JOYSTICK_DATA_START) recvInProgress = true;

        lastRb = rb;
    }
}

void sendLog(String m)
{
  const int LEN_LIMIT = 0Xff;
  if(useLogs) 
  {  
    StaticJsonDocument<LEN_LIMIT> doc;
    doc["type"] = "log"; // a-ka log item
    doc["value"] = m;

    char buf[LEN_LIMIT];
    int l = serializeJson(doc, buf);
    buf[l++]=0x0D;
    buf[l++]=0x0A;
    buf[l++]=0x00;

    String s = buf;
    sendMessage(&Serial, s);
    //sendMessage(&Serial1, s);
  }  
}

bool showNewData() 
{
    if (newData == true) 
    {
        String m = "This came in: ";
        for (byte n = 0; n < numReceived; n++) { m += String(receivedBytes[n], HEX); m += " "; }
         
        sendLog(m);
        
        newData = false;
        return true;
    }

    return false;
}

void moveServo(int p)
{
    int p0 = servo.read();
    int p1 = p0+p;
    if(p1 < 0) p1 = 0;
    if(p1 > 180) p1 = 180;
   
    servo.write(p1);
    delay(15);
    
    String m = "Servo";
    m += " is moving!";
    m += String(p0);
    m += " -> ";
    m += String(p1);
        
    if(useLogs) sendLog(m);
}

String camAddress[2];
String testCamera()
{
  sendLog("Camera I2C Test Started(1/2)..");
  myStateDoc["value"]["ca"] = JsonArray();
  
  Wire.requestFrom(0x20, 64);
  Serial.println("Camera I2C Request Sent..");
  String msg = "";
  while(Wire.available()) {

    char c = Wire.read();    // Receive a byte as character
    msg += c;
    //Serial.print(c);         // Print the character
  }

  String msg1 = msg;
  msg1.trim();
  int n = 0;
  if(msg1 != "") {
    camAddress[n] = msg1;
    myStateDoc["value"]["ca"][n] = msg;
    n++;
  }
  
  sendLog("Camera I2C Test Started(2/2)..");
  Wire.requestFrom(0x21, 64);
  //Serial.println("Camera I2C Request Sent..");
  msg = "";
  while(Wire.available()) {
    char c = Wire.read();    // Receive a byte as character
    msg += c;
    //Serial.print(c);         // Print the character
  }

  msg1 = msg;
  msg1.trim();
  
  if(msg1 != "") {
    camAddress[n] = msg1;
    myStateDoc["value"]["ca"][n] = msg;
  }

  sendLog("Camera I2C Test Ended");
}

void initMyState(){
    myStateDoc["type"] = "state";
    JsonObject myState = myStateDoc.createNestedObject("value");
    
//    readMac(); 
    myStateDoc["value"]["id"] = mac;
    myStateDoc["value"]["ca"] = JsonArray();
    for(int i = 0; i < 2; i++){
      myStateDoc["value"]["ca"][i] = camAddress[i];
    }
    myStateDoc["value"]["ct"] = "tank";//getType();
//    myState["value"] = JsonArray();    
    myStateDoc["value"]["dm"] = driveMode;
    myStateDoc["value"]["lv"] = _l0;
    myStateDoc["value"]["v"] = JsonArray();
    myStateDoc["value"]["v"][0] = v1;
    myStateDoc["value"]["v"][1] = v2;
}

void sendMyState() {
    char buf[0xFF];

    myStateDoc["value"]["ca"] = JsonArray();
    for(int i = 0; i < /*camAddress.length()*/2; i++){
      myStateDoc["value"]["ca"][i] = camAddress[i];
    }
    //myStateDoc["value"]["ct"] = "tank";
    myStateDoc["value"]["dm"] = driveMode;
    myStateDoc["value"]["lv"] = _l0;
    myStateDoc["value"]["v"] = JsonArray();
    myStateDoc["value"]["v"][0] = v1;
    myStateDoc["value"]["v"][1] = v2;

    int l = serializeJson(myStateDoc, buf);
    String s = buf;
    
    sendMessage(&Serial, s);
    //sendMessage(&Serial1, s);
//    Serial.write(JOYSTICK_DATA_START);
//    Serial.write(JOYSTICK_DATA_START0);
//    Serial.write(buf, l);
//    Serial.write(JOYSTICK_DATA_END);
//    Serial.write(JOYSTICK_DATA_END0);
//    Serial.write(0x0D);
//    Serial.write(0x0A);    
}

void button_ISR(){
//  button_time = millis();
  //check to see if increment() was called in the last 250 milliseconds
//  if (button_time - last_button_time > 250){
    sendLog("Interrupt ");
    
    huskyToggle = !huskyToggle ;
    digitalWrite(huskyPin, huskyToggle );
//    last_button_time = button_time;
//    }
}

void setup() 
{ 
    Serial.begin(115200);
    Serial1.begin(115200);

    //Serial2.begin(115200);
    Wire.begin();
//    Serial3.begin(115200);

    for(int i=4;i<=7;i++) pinMode(i, OUTPUT);

    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(huskyPin, OUTPUT);
    pinMode(buttonPin, INPUT);
    
    //_cam = &(Serial2);
    _log = &(Serial1);
//    _log = &(Serial);
    _pad = &(Serial);

    //sendMessage("CameraIPAddress=" + cip);

    servo.attach(R1);
    
    Wire.begin(); 
    Wire.setClock(100000);
    //sendLog("I2C is ready as well!");

    testCamera();

    initMyState();
    //testCamera();    
    huskyToggle = digitalRead(I1);
    sendLog("RoboTank is ready!");

    testCamera();
}


void loop() 
{ 
    receiveBytes(_pad);    
    if(showNewData())
    {
        static uint8_t pid = 0, _pid = 0;        
        uint8_t a = receivedBytes[0], b = receivedBytes[1], c = receivedBytes[2], d = receivedBytes[3];
        
        v1 = getYMove(a, b, c, d);
        v2 = getXMove(a, b, c, d);

        b3 = receivedBytes[6];
        b4 = receivedBytes[7];

        pid = receivedBytes[4]; 

        if(pid == _pid) return;   
        _pid = pid;

        driveMotor(v1, v2);        
        checkButtons(); 

        if(useCam) testCamera();

        sendMyState();
    }

    int v1 = digitalRead(I1);
    if(v1 != huskyToggle) {
      sendLog("HUSKYLENS button is pressed");

      digitalWrite(huskyPin, v1 == HIGH ? HIGH : LOW);
      huskyToggle = v1;
      //moveServo(30);
    }
    
}
