#include <stdint.h>
#include <SoftwareSerial.h>

SoftwareSerial xbee(2, 3);

void log(String m) {
  if (Serial) {
    Serial.println(m);
  }
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
    String x = xbee.readString();


    log(x);
  }
}
