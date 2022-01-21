#include <stdint.h>

#define LEONARDO

#ifdef UNO

#include <SoftwareSerial.h>
SoftwareSerial xbee(2,3);

#endif

#ifdef LEONARDO

#define xbee Serial1

#endif

void ping(String m) { xbee.println(m); }
void log(String m) { Serial.println(m); }

void task1()
{
  log("task1");

  ping("v");delay(250); ping("s");
}


void task2()
{
  log("task2"); 

  ping("v");delay(250); ping("s");
}


void task3()
{
  log("task3");
  
  ping("v");delay(250); ping("s");
}


void task4()
{
  log("task4");
  
  ping("v");delay(250); ping("s");
}


void setup()
{
  Serial.begin(9600);
  xbee.begin(115200);

  log("setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  log("loop");

  if (xbee.available()) {
    int cmd = -1;
    String x = xbee.readString();

    if(x.indexOf("\"S1\"") > 0) {
      if(x.indexOf("\"1\"") > 0) {
        task1();
      } else
      if(x.indexOf("\"2\"") > 0) {
        task2();
      } else
      if(x.indexOf("\"3\"") > 0) {
        task3();
      } else
      if(x.indexOf("\"4\"") > 0) {
        task4();
      } else {
        cmd=-1;
        log("unknown");
      }
    }

    log(x);
  }
}
