#include <stdint.h>
#include <Arduino.h>

#define virbrationMotorPin 2

volatile int buttonState[17];
volatile int joystick[6] = {0, 0, 500, 500, 500, 500};
bool upd0 = false, f = false;
unsigned long timer = 0;
int k = 0;
byte buf[JOYSTICK_DATA_LENGTH+2];

void vibr(uint16_t ms)
{
  digitalWrite(virbrationMotorPin, HIGH);
  delay(ms);
  digitalWrite(virbrationMotorPin, LOW);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  delay(500);

  InitIO();
}

void InitIO() {
  for (int i = 0; i < 17; i++) pinMode(i, INPUT);

  pinMode(virbrationMotorPin, OUTPUT);
  digitalWrite(virbrationMotorPin, LOW); // Stop shacking of the gamepad
}

bool readCommand() {
  return false;
}

void loop()
{
  while (true)
  {
    if (millis() - timer >= JOYSTICK_SEND_DELAY)
    {
      bool changed = DataUpdate();
      upd0 = changed;
      if (changed)
      {
        k = 0;
        printData();
      } else 
      {
        if(k++ < 3)
        {
          Serial1.write(buf, JOYSTICK_DATA_LENGTH+2);  
          //Serial1.flush();
          delay(30);
        }
      }

      timer = millis();
    }
  }
}


bool DataUpdate()
{
  int a = 0, b = 0, c = 0, d = 0;

  f = false;

  for (int i = 0; i < 17; i++)
  {
    if (i > 2)
    {
      a = digitalRead(i);
    }
    
    b = buttonState[i];

    if (a != b)
    {
      buttonState[i] = a;
      f = true;
    }
  }

  for (int i = 2; i < 6; i++)
  {
    a = analogRead(i);
    b = joystick[i];
    d = max(500 - a, a - 500);

    if (d <= 21)
    {
      a = 500;
    } else if (a > 1000)
    {
      a = 1000;
    }

    c = max(a - b, b - a);
    if (c >= 4)
    {
      joystick[i] = a;
      f = true;
    }

  }

  return f;
}

void writeData(byte x[JOYSTICK_DATA_LENGTH])
{
  static uint8_t id = 0;
  String str = "";
  int k = 0;
  
  x[k++] = JOYSTICK_DATA_START;

  unsigned long Btns = 0;
  for (int i = 2; i < 6; i++)
  {
    int t = joystick[i];// == 0 ? 1 : joystick[i];
    t = (((t + 1) / 4) + 2) & 255;
    t = t <= 2 ? 0 : t;
    t = t >= 252 ? 255 : t;
    x[k++] = (byte)(t & 0xFF);
  }

  for (int i = 0; i < 17; i++)
  {
    int j = 0;
    j = i < 2 ? i : i - 1;

    if ((i < 2 && buttonState[i] < 100) || (i > 2 && buttonState[i] == 0))
    {
      bitWrite(Btns, j, 1);
    }
  }

  x[k++] = id++;
  //x[k++] = (byte)(((Btns & 0xFF000000) >> 24) & 0xFF);
  x[k++] = (byte)(((Btns & 0xFF0000) >> 16) & 0xFF);
  x[k++] = (byte)(((Btns & 0xFF00) >> 8) & 0xFF);
  x[k++] = (byte)(((Btns & 0xFF)) & 0xFF);
  x[k++] = JOYSTICK_DATA_END;
}


void printData()
{
  writeData(buf);

  for (int i = 0; i < JOYSTICK_DATA_LENGTH; i++)
  {
    byte x = buf[i]; 
    if(i > 0 && i < JOYSTICK_DATA_LENGTH - 1 && x == JOYSTICK_DATA_START)
    {
      x = JOYSTICK_DATA_START+1;
    } else
    if(i > 0 && i < JOYSTICK_DATA_LENGTH - 1 && x == JOYSTICK_DATA_END)
    {
      x = JOYSTICK_DATA_END+1;
    }
  }
  buf[JOYSTICK_DATA_LENGTH] = 0x0D;
  buf[JOYSTICK_DATA_LENGTH+1] = 0x0A;
  Serial1.write(buf, JOYSTICK_DATA_LENGTH+2);  
}
