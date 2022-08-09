#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3);

#define VEHICLE_CAR  01
#define VEHICLE_TANK 02

#define DEBUG_DELAY 200

const byte VEHICLE_TYPE = VEHICLE_CAR;    // car
//const byte VEHICLE_TYPE = VEHICLE_TANK;    // tank

int E1 = 4;    //M1 Speed Control
int E2 = 7;    //M2 Speed Control
int M1 = 5;    //M1 Direction Control
int M2 = 6;    //M2 Direction Control

int L1 = 5;    //L1 Light Control
int L2 = 6;    //L2 Light Control

int BTN1 = 0x0100;
int BTN2 = 0x0400;
int BTN3 = 0x0800;
int BTN4 = 0x0200;
int BTNR1 = 0x1000;
int BTNR2 = 0x2000;
int BTNL1 = 0x4000;
int BTNL2 = 0x800000;
int BTNSTART = 0x0000000B;
int BTNSELECT = 0x00000007;

uint16_t BLOCK_FROM = 0x100;

Stream* _pad;
Stream* _log;

bool useLogs = false;
bool useDelay = false;

bool blockRare = false;
bool blockFront = false;

bool pressed1 = false;
bool pressed2 = false;
bool pressed3 = false;
bool pressed4 = false;
bool pressedL1 = false;
bool pressedL2 = false;
bool pressedR1 = false;
bool pressedR2 = false;
bool pressedSelect = false;
bool pressedStart = false;

const byte numBytes = 0x20;
byte numReceived = 0;
byte receivedBytes[numBytes];

boolean newData = false;

byte v1 = 0x7F, v2 = 0x7F, b1 = 0x00, b2 = 0x00, b3 = 0x00, b4 = 0x00, _b1 = 0x00, _b2 = 0x00, _b3 = 0x00, _b4 = 0x00, _l0 = 0x00;
int rangeFront = 0, rangeRare = 0;

void SetServoPos(byte a, byte b)
{
  if(VEHICLE_TYPE == VEHICLE_CAR)
  {
  
  } else
  if(VEHICLE_TYPE == VEHICLE_TANK)
  {
    
  }
}

void DriveMotorP(byte m1p, byte m2p)
{    
  /*
    0x7F - engine stop
    if(VEHICLE_TYPE == VEHICLE_CAR)
    {
      m1p => front-rare engine value
      m2p => left-right engine value
    }

    if(VEHICLE_TYPE == VEHICLE_TANK)
    {
      m1p => front-rare level value (Vl = ...)
      m2p => left-right level value (Vr = ...)
    }
  */
  if(useLogs) 
  {        
    String m = "DriveMotorP: ";
    m += String(m1p, HEX);
    m += ",";
    m += String(m2p,HEX);
    
    _log->println(m);

    if(useDelay) delay(1000);
  }
    
  if(VEHICLE_TYPE == VEHICLE_CAR)
  {
    if(m1p != 0x7F)
    {
        digitalWrite(E1, HIGH);
        analogWrite(M1, (m1p));
    } else 
    {
        digitalWrite(E1, LOW);
    }

    if(m2p != 0x7F)
    {
        int m3p = m2p > 0x7F ? 0xFF - m2p - 1 : 0xFF - m2p + 1;
        if(m3p <= 0)
        {
          m3p = 0;
        } else
        if(m3p >= 0xFF)
        {
          m3p = 0xFF;
        }
        digitalWrite(E2, HIGH);
        analogWrite(M2, (m3p));
        
    } else
    {
        digitalWrite(E2, LOW);
    }
  } else
  if(VEHICLE_TYPE == VEHICLE_TANK)
  {  
    //TODO
  }
}

void light(uint8_t level)
{
    if(useLogs) 
    {        
        String m = "Light: ";
        m += String(level, HEX);
        
        _log->println(m);

        if(useDelay) delay(DEBUG_DELAY);
    }
  
    digitalWrite(L1, level);
    digitalWrite(L2, level);

    _l0 = level;
}

const char * vehicleType()
{
  switch(VEHICLE_TYPE)
  {
    case VEHICLE_CAR: return "Car";
    case VEHICLE_TANK: return "Tank";
  }
  return "Unknown";
}

void setup() 
{
    Serial.begin(115200);
    Serial1.begin(115200);

    int i;
    for(i=4;i<=7;i++) pinMode(i, OUTPUT);

    //pinMode(L1, OUTPUT);
    //pinMode(L2, OUTPUT);

    pinMode(A6, INPUT);
    pinMode(A7, INPUT);

    _pad = &(Serial);
    _log = &(Serial1);

    //if(useLogs) 
    //{
        String m = "Robo";
        m += vehicleType();
        m += " is ready!";
        
        _log->println(m);
    //}
}

void loop() 
{ 
    receiveBytes(_pad);
    checkBlocks();
    

    if(showNewData())
    {
        v1 = receivedBytes[2];
        v2 = receivedBytes[1];

        b1 = receivedBytes[4];
        b2 = receivedBytes[5];
        b3 = receivedBytes[6];
        b4 = receivedBytes[7];

        if((v1 < 0x7F && blockRare == false)
        || (v1 > 0x7F && blockFront == false))
        {
            DriveMotorP(v1, v2);
        } else
        {
            DriveMotorP(0x7F, v2);    
        }

        checkButtons();
        if(pressedSelect && pressed1) useLogs = true;
        if(pressedSelect && pressed3) useLogs = false;
        if(pressedSelect && pressed2) useDelay = true;        
        if(pressedSelect && pressed4) useDelay = false;

        if((pressedR2 && pressedL2)
        || (pressedR1 && pressedL1))
        {  
            int16_t l = (pressedR1 && pressedL1) ? _l0 + 0x60 : _l0  - 0x60;  
            if(l < 0) l = 0; else
            if(l > 0xFF) l = 0xFF;
            
            light(l); 
        }
    }

    //checkBlocks();
}

void checkButtons()
{
    if(b4 == _b4 && b3 == _b3 && b2 == _b2 && b1 == _b1) return;
    
    pressedStart = ((b4 & 0x0B) == 0x0B);
    pressedSelect = ((b4 & 0x07) == 0x07);    
    
    pressed1 = ((b3 & 0x01) == 0x01);
    pressed2 = ((b3 & 0x04) == 0x04);
    pressed3 = ((b3 & 0x08) == 0x08);
    pressed4 = ((b3 & 0x02) == 0x02);

    pressedL1 = ((b3 & 0x40) == 0x40);
    pressedL2 = ((b3 & 0x80) == 0x80);
    pressedR1 = ((b3 & 0x10) == 0x10);
    pressedR2 = ((b3 & 0x20) == 0x20);

    _b1 = b1;
    _b2 = b2;
    _b3 = b3;
    _b4 = b4;

    if(useLogs) 
    { 
        String m = "Buttons: ";
        m += String(b1, HEX);
        m += String(b2, HEX);
        m += String(b3, HEX);
        m += String(b4, HEX);
        m += " [";
        
        if(pressedStart) m += ("START ");
        if(pressedSelect) m += ("SELECT ");
        if(pressedL1) m += ("L1 ");
        if(pressedL2) m += ("L2 ");
        if(pressedR1) m += ("R1 ");
        if(pressedR2) m += ("R2 ");
        if(pressed1) m += ("1 ");
        if(pressed2) m += ("2 ");
        if(pressed3) m += ("3 ");
        if(pressed4) m += ("4 ");
        m += (" ]");
      
  
        _log->println(m);
        if(useDelay) delay(DEBUG_DELAY);
    }
}

void checkBlocks()
{
    if(VEHICLE_TYPE == VEHICLE_CAR)
    {
      uint16_t d1 = getRangeFront();
      uint16_t d2 = getRangeRare();

      if(useLogs) 
      {  
          String m = "Distances: ";
          m += String(d1, HEX);
          m += ",";
          m += String(d2, HEX);
          m += " (";
          m += (blockFront ? "blocked"  : "free");
          m += ",";
          m += (blockRare ? "blocked"  : "free");
          m += ")";
          
          _log->println(m);

          if(useDelay) delay(DEBUG_DELAY);
      }
    
      if((v1 < 0x7F && blockRare)
      || (v1 > 0x7F && blockFront))  
      {
          DriveMotorP(0x7F, v2);
      }
    } else
    if(VEHICLE_TYPE == VEHICLE_TANK)
    {
      blockRare = false;
      blockFront = false;
    }
}

void receiveBytes(Stream* stream) 
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    const byte startMarker = 0xAA;
    const byte endMarker = 0xBB;
    byte rb;   

    while (stream->available() > 0 && newData == false) 
    {
        rb = stream->read();

        if (recvInProgress == true) 
        {
            if (rb != endMarker) 
            {
                receivedBytes[ndx] = rb;
                if (ndx++ >= numBytes) 
                {
                    ndx = numBytes - 1;
                }
            }
            else 
            {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == startMarker) 
        {
            recvInProgress = true;
        }
    }
}

bool showNewData() 
{
    if (newData == true) 
    {
        if(useLogs) 
        {  
            _log->print("This came in: ");
            for (byte n = 0; n < numReceived; n++) 
            {
                _log->print(receivedBytes[n], HEX);
                _log->print(' ');
            }
            _log->println();
        }
        
        newData = false;

        return true;
    }

    return false;
}

uint16_t getRangeFront () 
{
    uint16_t value = analogRead (A6);

    if (value < 10) value = 10;
    uint16_t range = ((67870.0 / (value - 3.0)) - 40.0);

    rangeFront = range;
    blockFront = rangeFront <= BLOCK_FROM;
   
    return range;
}

uint16_t getRangeRare () 
{
    uint16_t value = analogRead (A7);
    
    if (value < 10) value = 10;
    uint16_t range = ((67870.0 / (value - 3.0)) - 40.0);

    rangeRare = range;
    blockRare= rangeRare<= BLOCK_FROM;
    
    return range;
}
