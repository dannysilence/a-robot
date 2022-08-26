#include <stdint.h>
#include <Arduino.h>

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3);
#endif

#define DEBUG_DELAY          0x7F  
#define VEHICLE_LIGHT_STEP   0x40
#define JOYSTICK_DATA_LENGTH 0x20
#define JOYSTICK_DATA_START  0xAA
#define JOYSTICK_DATA_END    0xBB

int E1 = 4;    //PLL based M1 Speed Control
int E2 = 7;    //PLL based M2 Speed Control
int M1 = 5;    //PLL based M1 Direction Control
int M2 = 6;    //PLL based M2 Direction Control
int L1 = 9;    //Front Light Control
int L2 = 10;   //Rare Light Control

Stream* _pad;
Stream* _log;

// General State Parts
bool useLogs       = true;
bool useDelay      = false;

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
 
// Joystick Message Retrieving Parts   
bool newData       = false;
uint8_t nReceived  = 0;
uint8_t receivedBytes[JOYSTICK_DATA_LENGTH];

// Vehicle Speed/State Parts
uint8_t v1 = 0x7F, v2 = 0x7F, driveMode = 1;          // Drive mode 1 - vehicle drives front/back by left joystick, left/right - by right, drive mode 2 - all directions handled by left joystick, drive mode 3 - all directions handled by right joystick;
uint8_t _b3 = 0x00, _b4 = 0x00, b3 = 0x00, b4 = 0x00, _l0 = 0x00;

void driveMotor(byte m1p, byte m2p)
{   
    if(useLogs) 
    {        
      String m = "DriveMotor["; m += String(driveMode); m += "]: "; m += String(m1p, HEX); m += ","; m += String(m2p,HEX);
    
      _log->println(m);
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
    } else digitalWrite(E1, LOW);
    
    if(y != 0x7F)
    {
      digitalWrite(E2, HIGH);
      analogWrite(M2, y);
    } else digitalWrite(E2, LOW);
}

void light(uint8_t level)
{
    if(useLogs) 
    {        
        String m = "Light: "; m += _l0 > 0   ? "ON" : "OFF"; m += "->"; m += level > 0 ? "ON" : "OFF"; m += ", level: "; m += String(level, HEX);
        
        _log->println(m);
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
      
        _log->println(m);
        if(useDelay) delay(DEBUG_DELAY);
    }
}

void receiveBytes(Stream* stream) 
{
    static bool recvInProgress = false;
    static uint8_t ndx = 0;
    uint8_t rb;   

    while (stream->available() > 0 && newData == false) 
    {
        rb = stream->read();

        if (recvInProgress == true) 
        {
            if (rb != JOYSTICK_DATA_END) 
            {
                receivedBytes[ndx] = rb;
                if (ndx++ >= JOYSTICK_DATA_LENGTH) ndx = JOYSTICK_DATA_LENGTH - 1;
            }
            else 
            {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                nReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }
        else if (rb == JOYSTICK_DATA_START) recvInProgress = true;
    }
}

bool showNewData() 
{
    if (newData == true) 
    {
        if(useLogs) 
        {  
            String m = "This came in: ";
            for (byte n = 0; n < nReceived; n++) { m += String(receivedBytes[n], HEX); m += " "; }
         
            _log->println(m);
        }
        
        newData = false;
        return true;
    }

    return false;
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

void setup() 
{
    Serial.begin(115200);
    Serial1.begin(115200);

    for(int i=4;i<=7;i++) pinMode(i, OUTPUT);

    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);

    _pad = &(Serial);
    _log = &(Serial1);

    _log->println("RoboTank is ready");
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

        //Control Drive Mode
        if(pressedSelect && pressed1) driveMode = 1;
        if(pressedSelect && pressed2) driveMode = 2;
        if(pressedSelect && pressed3) driveMode = 3;        

        //Control Debug Logging and Delays
        if((pressedR1 && pressed1) || (pressedR2 && pressed1)) useLogs  = pressedR1 && pressed1;
        if((pressedR1 && pressed2) || (pressedR2 && pressed2)) useDelay = pressedR1 && pressed2;

        //Control Vehicle Lights
        if(pressedL1 || pressedL2)
        {  
            int16_t l0 = _l0;
            l0 = (pressedL1) ? _l0 + VEHICLE_LIGHT_STEP : l0;  
            l0 = (pressedL2) ? _l0 - VEHICLE_LIGHT_STEP : l0;  
            
            if(l0 < 0) l0 = 0; else if(l0 > 0xFF) l0 = 0xFF;
            
            light(l0); 
        }
    }
}
