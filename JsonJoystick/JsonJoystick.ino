      
#include <stdint.h>
#define DELAY 400
#define virbrationMotorPin 2

int buttonState[17];
int joystick[6] = {0,0,500,500,500,500};
int inputCommand = 0;
bool f=false,upd0=false,upd1=false;

typedef struct Point 
{
  uint16_t X;
  uint16_t Y;

  bool equals(Point other)
  {
    return X == other.X && Y == other.Y;
  }

} Point;

void vibr(uint16_t ms) 
{
  digitalWrite(virbrationMotorPin,HIGH);
  delay(ms);
  digitalWrite(virbrationMotorPin,LOW);

}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
  
  InitIO();             
  
  if(Serial1)
  {
    vibr(250);
  }
}

void InitIO(){ 
  for(int i = 0; i < 17; i++) pinMode(i, INPUT); 
  
  pinMode(virbrationMotorPin,OUTPUT);
  digitalWrite(virbrationMotorPin,LOW);  // Stop shacking of the gamepad
}

unsigned long timer = 0;
bool readCommand()
{
  bool y = Serial1.available() || Serial.available();
  
  if(y) {
    char input = Serial1.read();
    if(Serial1.available()) {
        input=Serial1.read();
    } else 
    if(Serial.available()) {
        input=Serial.read();
    }
    
    switch(input){
      case 'v':
        inputCommand = input;
        digitalWrite(virbrationMotorPin,HIGH);
        break;
      
      case 's':
        inputCommand = input;
        digitalWrite(virbrationMotorPin,LOW);
        break;
        
      default:
        break;
    }
  }

  return y;
}

void loop()
{
  while(true)
  {
    upd0 = false; upd1 = false;
    if(millis() - timer > DELAY)
    {  
      upd0 = DataUpdate();  //read the buttons and the joysticks data

      if(upd0) 
      {      
        printData();   //print the datas and states
      } 
      timer = millis(); 
    

    upd1 = readCommand();
    if(!upd0 && !upd1)
    {
      delay(DELAY/2);
    }
    }
  }
}


bool DataUpdate()
{
  int a = 0, b = 0, c = 0, d = 0;
  f=false;
    
  for(int i = 0; i < 17; i++) 
  {
    if(i > 2)
    {
      a = digitalRead(i); 
    } else
    if(i < 2) 
    {
      a = analogRead(i);
      if(a < 100) a = 0;
    }
    
    b = buttonState[i];
    
    if(a != b)
    //if((i < 2 && a < 100) || (i > 2 && a == 0))
    {
      buttonState[i] = a;
      f = true;
    } 
  }


  //Joysticks (0-1023), let's skip changes less than 4 and valuesnewar 500+-25 tread as 500
  for(int i = 2; i < 6; i++)
  {
    a = analogRead(i);
    b = joystick[i];
    d = max(500-a,a-500);
    
    if(d < 25) 
    {
      a = 500;
    } else
    if(a > 1000) 
    {
      a = 1000;
    }
    
    c = max(a-b,b-a);
    if(c >= 4)
    {
      joystick[i] = a;
      f = true;
    }
    
  }

  return f;
}

String Buttons[17] = { "J2","J1","","S2","S1","UP","LEFT","DOWN","RIGHT","1","4","2","3","RZ1","RZ2","LZ1","LZ2"};

String writeData()
{ 
  String x = "{ \"buttons\": [";
  for(int i = 0; i < 17; i++)
  {
    int j = 0;
    if((i < 2 && buttonState[i] < 100) || (i > 2 && buttonState[i] == 0))
    {
      if(j++>0)
      {
        x += ", ";
      }
      x+= "\"";
      x+= Buttons[i];
      x+="\"";
    }
  }
  x+="], \"sticks\": [";
  for(int i = 2; i < 6; i++)
  {
    x+=joystick[i];
    if(i<5) x+=",";
  }
  x+="], \"state\": ";
  x+=inputCommand;
  x+=" }";

  return x;
}

void printData()
{
  String data = writeData();
  
  
  Serial1.println(data);  
  
  if(Serial)
  {
    Serial.println(data);
  }
}
