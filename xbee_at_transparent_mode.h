#include <stdint.h>

#ifdef ARDUINO_AVR_UNO

#include <SoftwareSerial.h>
SoftwareSerial Serial1(2,3);

#endif



int xid()
{  
  String x = "", y = "", z = "";
  int xs = 20, xm = 100;
  
  Serial1.write(0x2B); delay(xs); 
  Serial1.write(0x2B); delay(xs); 
  Serial1.write(0x2B); delay(xs); 
  delay(xm); x = Serial1.readString(); delay(xm);  
  Serial1.write(0x41); delay(xs);
  Serial1.write(0x54); delay(xs);
  Serial1.write(0x4E); delay(xs);
  Serial1.write(0x49); delay(xs);
  Serial1.write(0x0D); delay(xs);
  delay(xm); y = Serial1.readString(); delay(xm);    
  Serial1.write(0x41); delay(xs);
  Serial1.write(0x54); delay(xs);
  Serial1.write(0x43); delay(xs);
  Serial1.write(0x4E); delay(xs);
  Serial1.write(0x0D); delay(xs);
  delay(xm); x = Serial1.readString(); delay(xm); int p = y.indexOf("-");
  if(p >= 0) { z = y.substring(p+1); } else { p = x.indexOf("-"); if(p > 0) { z = x.substring(p+1); } else { z = "-1"; } }

  return z.toInt();  
}
