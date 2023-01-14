/*

    Based on https://github.com/sparkfun/Spectrum_Shield/blob/master/Firmware/SparkFun_Spectrum_Serial_Plotter_Demo/SparkFun_Spectrum_Serial_Plotter_Demo.ino

*/

#include <NeoPixelBus.h>

class Display
{
  private:
    const int W = 5;
    const int H = 8;
    NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>* strip;
  public:
    void begin(int pin);
    void drawBand(int number, int value);
    void drawBands(int* values);
};

void Display::begin(int pin)
{
  strip = new NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>(80, pin);
  strip->Begin();

  strip->SetPixelColor(0, RgbColor(255,0,0));
  strip->SetPixelColor(1, RgbColor(0,255,0));
  strip->SetPixelColor(2, RgbColor(0,0,255));
  strip->Show();

  delay(1000);

  strip->ClearTo(RgbColor(0,0,0));
  strip->Show();
}

void Display::drawBand(int number, int value)
{
  int a0 = number * 8;
  int b0 = value > 1000 ? 1000 : value; 
  int b1 = b0 < 100 ? 0 : b0 - 100;     
  int b2 = b1 / 9;
  int c0 = b2 < 50 ? 8 * b2 / 50 : 8;
  int c1 = b2 < 50 ? 0 : 8 - (8 * (100 - b2) / 50);

  Serial.print(number); 
  Serial.print("["); 
  Serial.print(value); 
  Serial.print("] => "); 
  Serial.print(b2); 
  Serial.print("%, c0=");
  Serial.print(c0);
  Serial.print(", c1=");
  Serial.print(c1);
  Serial.println("  ");
  
  for(int i = 0; i < 8; i++)
  {
    RgbColor d1 = i < c0 ? RgbColor(0,0,0x7F) : RgbColor(0,0,0);
    RgbColor d2 = i < c1 ? RgbColor(0,0,0x7F) : RgbColor(0,0,0);
    
    strip->SetPixelColor(a0+i, d1);
    strip->SetPixelColor(a0+40+i, d2);
  }
  
  strip->Show();
}

void Display::drawBands(int* values)
{
  for(int i = 0; i < 5; i++)
  {
    this->drawBand(i, values[i]);
  }
}

Display left, right;

//Declare Spectrum Shield pin connections
#define STROBE 4
#define RESET 5
#define DC_One A0
#define DC_Two A1

//Define spectrum variables
int freq_amp;
int Frequencies_One[5];
int Frequencies_Two[5];
//int i;

/********************Setup Loop*************************/
void setup() {
  //Set spectrum Shield pin configurations
  pinMode(STROBE, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(DC_One, INPUT);
  pinMode(DC_Two, INPUT);

  //Initialize Spectrum Analyzers
  digitalWrite(STROBE, LOW);
  digitalWrite(RESET, LOW);
  delay(5);

  Serial.begin(115200);
  Serial.println("SoundSpectrum started");

  left.begin(12);
  right.begin(13);
  delay(200);
}

/**************************Main Function Loop*****************************/
void loop() {
  Read_Frequencies();
  Graph_Frequencies();
  delay(20);
}

/*******************Pull frquencies from Spectrum Shield********************/
void Read_Frequencies() {
  digitalWrite(RESET, HIGH);
  delayMicroseconds(200);
  digitalWrite(RESET, LOW);
  delayMicroseconds(200);

  int j = 0;
  //Read frequencies for each band
  for (freq_amp = 0; freq_amp < 7; freq_amp++)
  {
    if(freq_amp == 3 || freq_amp == 5) continue;
    
    digitalWrite(STROBE, HIGH);
    delayMicroseconds(50);
    digitalWrite(STROBE, LOW);
    delayMicroseconds(50);

    Frequencies_One[j] = analogRead(DC_One);
    Frequencies_Two[j] = analogRead(DC_Two);
    j++;
  }
}

/*****************Print Out Band Values for Serial Plotter*****************/
void Graph_Frequencies() {
  for (int i = 0; i < 5; i++)
  {
    Serial.print(Frequencies_One[i]);
    Serial.print(" ");
//    Serial.print(Frequencies_Two[i]);
//    Serial.print(" ");
//    Serial.print( (Frequencies_One[i] + Frequencies_Two[i]) / 2 );
//    Serial.print("    ");
  }
   Serial.print("      ");
  for (int i = 0; i < 5; i++)
  {
//    Serial.print(Frequencies_One[i]);
//    Serial.print(" ");
    Serial.print(Frequencies_Two[i]);
    Serial.print(" ");
//    Serial.print( (Frequencies_One[i] + Frequencies_Two[i]) / 2 );
//    Serial.print("    ");
  }
  Serial.println();

  left.drawBands(Frequencies_One);
  right.drawBands(Frequencies_Two);
}
