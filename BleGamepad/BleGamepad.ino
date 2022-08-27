#include <stdint.h>
#include <Arduino.h>

#include "Gamepad.h"

volatile int buttonState[17];
volatile int joystick[6] = {0, 0, 500, 500, 500, 500};
bool f = false;
unsigned long timer = 0;
byte buf[sizeof(GamepadState)];

Stream*  _io;
Stream*  _log;
Gamepad* _pad;
GamepadState* _state;

bool checkDataUpdate()
{
  int a = 0, b = 0, c = 0, d = 0;
  f = false;

  for (int i = 0; i < 17; i++)
  {
    if (i > 2) { a = digitalRead(i); }
    
    b = buttonState[i];

    if (a != b) { buttonState[i] = a; f = true; }
  }

  for (int i = 2; i < 6; i++)
  {
    a = analogRead(i);
    b = joystick[i];
    d = max(500 - a, a - 500);

    if (d <= 21) a = 500; else if (a > 1000) a = 1000;

    c = max(a - b, b - a);
    if (c >= 4) { joystick[i] = a; f = true; }
  }

  return f;
}

void updateState(byte x[sizeof(GamepadState)])
{
  static uint8_t id = 0;
  unsigned long Btns = 0;
  int k = 0;  
  
  for (int i = 2; i < 6; i++)
  {
    int t = joystick[i];
    
    t = (((t + 1) / 4) + 2) & 255;
    t = t <= 2 ? 0 : t;
    t = t >= 252 ? 255 : t;
    
    x[k++] = (byte)(t & 0xFF);
  }

  for (int i = 0; i < 17; i++)
  {
    int j = i < 2 ? i : i - 1;

    if ((i < 2 && buttonState[i] < 100) || (i > 2 && buttonState[i] == 0)) bitWrite(Btns, j, 1);
  }

  x[k++] = id++;
  x[k++] = (byte)(((Btns & 0xFF0000) >> 16) & 0xFF);
  x[k++] = (byte)(((Btns & 0xFF00) >> 8) & 0xFF);
  x[k++] = (byte)(((Btns & 0xFF)) & 0xFF);

  GamepadState state = GamepadState::fromBytes(buf);
  _state = &state;
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  _io  = &(Serial1);
  _log = &(Serial);
  
  Serial.println("Gamepad is ready");
    
  Gamepad pad = Gamepad(_io, _log);
  GamepadState state;
    
  _pad = &pad;    
  _state = &state;

  for (int i = 0; i < 17; i++) pinMode(i, INPUT);
}

void loop()
{
  _log->println("...");
  while (true)
  {   
    if (millis() - timer >= 100)
    {
      if (checkDataUpdate())
      {
        updateState(buf);        
        _pad->send(_state);
      } else _pad->send(_state);

      timer = millis();
    }
  }
}
