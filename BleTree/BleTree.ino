
/*

Ble MAC 0x90E2029E41D9
Ble Name BleTree

*/

#include "dht11.h"
dht11 DHT;
#define MOISTURE_PIN A2  //soil Moisture sensor//
#define DHT11_PIN    9   //DHT11
#define DATA_LENGTH 0xFF
#define DATA_START  0xAA
#define DATA_END    0xBB

int airHumidity;   //environment humidity
int airTemperature;  // environment temperature
int soilHumidity;   //soil moisture

bool useLogs = true;

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

// Vehicle Speed/State Parts
uint8_t driveMode = 0;
uint8_t v1 = 0x7F, v2 = 0x7F;
uint8_t _b3 = 0x00, _b4 = 0x00, b3 = 0x00, b4 = 0x00;
uint8_t _l0 = 0x00;

int setHumidity = 50;

// Controol Message Retrieving Parts   
bool newData       = false;
uint8_t numReceived = 0;
uint8_t receivedBytes[DATA_LENGTH];

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}

void loop()
{
    readSensors();
    receiveBytes(&Serial1);        
    if(showNewData())
    {
        static uint8_t pid = 0, _pid = 0;        
        uint8_t a = receivedBytes[0], b = receivedBytes[1], c = receivedBytes[2], d = receivedBytes[3];
        
        b3 = receivedBytes[6];
        b4 = receivedBytes[7];

        pid = receivedBytes[4]; 

        if(pid == _pid) return;   
        _pid = pid;

        checkButtons();

        if(driveMode == 0) {
          print("op=noop, dm=");
          println(driveMode);
        } else
        if(driveMode == 1) {
          print("op=humidiate, dm=");
          println(driveMode);
          humidiate();
        } else 
        if (driveMode == 2) {
          pumpOn();
          delay(1500);
          pumpOff();
          delay(1500);
        }
    }
}

void readSensors(){
  int chk;
  chk = DHT.read(DHT11_PIN);   //Read Data
  switch (chk){
    case DHTLIB_OK:
                print("OK,\t");
                break;
    case DHTLIB_ERROR_CHECKSUM:
                print("Checksum error,\t");
                break;
    case DHTLIB_ERROR_TIMEOUT:
                print("Time out error,\t");
                break;
    default:
                print("Unknown error,\t");
                break;
  }
  airHumidity=DHT.humidity;
  airTemperature=DHT.temperature;
  soilHumidity=analogRead(MOISTURE_PIN);

  print("airHumidity:");
  print(airHumidity);
  print(",\t");
  print("airTemperature:");
  print(airTemperature);
  print(",\t");
  print("soilHumidity:");
  println(soilHumidity);

  delay(1000);
}

void print(String m)
{
  Serial.print(m);
  Serial1.print(m);
}

void println(String m)
{
  Serial.println(m);
  Serial1.println(m);
}

void print(int m)
{
  Serial.print(m);
  Serial1.print(m);
}

void println(int m)
{
  Serial.println(m);
  Serial1.println(m);
}

//open pump
void pumpOn()
{
  print("pumpOn, dm=");
  println(driveMode);
  
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
}
//close pump
void pumpOff()
{
  print("pumpOff, dm=");
  println(driveMode);
  
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}

void humidiate() {
  soilHumidity = map(analogRead(MOISTURE_PIN), 0, 1023, 0, 100);    //Map analog value to 0~100% soil moisture value
  print("humidiate, dm=");
  print(driveMode);
  print(", cv=");
  print(soilHumidity);
  print(", tv=");
  println(setHumidity);
  if (soilHumidity < setHumidity) {
    pumpOn();
  } else {
    pumpOff();
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
            if (rb != DATA_END) 
            {
                receivedBytes[ndx] = rb;
                if (ndx++ >= DATA_LENGTH) ndx = DATA_LENGTH - 1;
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
        else if (rb == DATA_START) recvInProgress = true;
    }
}

bool showNewData() 
{
    if (newData == true) 
    {
        String m = "This came in: ";
        for (byte n = 0; n < numReceived; n++) { m += String(receivedBytes[n], HEX); m += " "; }
         
        println(m);
        
        newData = false;
        return true;
    }

    return false;
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
      
        println(m);
    }

    
        //Control Drive Mode
    if(pressedStart && pressed1) driveMode = driveMode == 1 ? 0 : 1;
    if(pressedStart && pressed1) driveMode = driveMode == 2 ? 0 : 2;
}
