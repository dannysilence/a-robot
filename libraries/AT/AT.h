#ifndef __MY___AT__
#define __MY___AT__

#include <Arduino.h>
#include <Stream.h>

#include "BeeDevice.h";
#include "BleDevice.h";

#define DEVICE_BLE 0
#define DEVICE_BEE 1
#define DEVICE_ESP 2

#define DELAY_XS 100
#define DELAY_XM 300
#define DELAY_XL 600

class AT
{
  private:
    int _n;
    int _t;
    bool _open;
    Stream* _dst;
    String _atExit;
    String _atEnter;
  public:
    
    AT(int serialN, int deviceType, String atEnter = "+++", String atExit = "AT+EXIT\r\n");
    ~AT();
    bool open();
    bool close();
    String send(String m);
    char read();
    String readString();
    void write(char c);
    void writeString(String z);
};


class ATDevice
{
  protected:
    String _map[0x10];
    String _addr[0x10];
    String _atEnter;
    String _atExit;
    AT* _device;
    int _devType;
  public:
    ATDevice(int serialN, int devType, String atEnter = "+++", String atExit = "AT+EXIT\r\n");
    virtual String getAddr();
    virtual String getName();
    virtual String getMode();
    virtual String getBind();
    virtual int getType();
    virtual void setName(String x);
    virtual void setMode(String x);
    virtual void setBind(String x);
    virtual void setAddr(String x);
    
    virtual String readString();
    virtual void writeString(String x); 

    virtual void bindTo(int n);
};


class BeeDevice : ATDevice
{
    public:
      BeeDevice(int serialN) : ATDevice(serialN, DEVICE_BEE, "+++", "ATCN\r\n") {
        _map[0] = "ATMY";
        _map[1] = "ATMY=";
        _map[2] = "ATNI";
        _map[3] = "ATNI=";
        _map[4] = "ATAP";
        _map[5] = "ATAP=";
        _map[6] = "ATDL";
        _map[7] = "ATDL=";

        _addr[0] = "0";
        _addr[1] = "2";
        _addr[2] = "4";
        _addr[3] = "6";
        _addr[4] = "8";
      }

      String getAddr();
      String getName();
      String getMode();
      String getBind();
      int getType();
      void setName(String x);
      void setMode(String x);
      void setBind(String x);
      void setAddr(String x);    
      String readString();
      void writeString(String x); 
      void bindTo(int n);
};



class BleDevice : ATDevice
{
    public:
      BleDevice(int serialN): ATDevice(serialN, DEVICE_BLE, "+++", "AT+EXIT\r\n") {
        _map[0] = "AT+MAC=?";
        _map[1] = "AT+MAC=";
        _map[2] = "AT+NAME=?";
        _map[3] = "AT+NAME=";
        _map[4] = "AT+DEFSETTING=?";
        _map[5] = "AT+DEFSETTING=";
        _map[6] = "AT+BIND=?";
        _map[7] = "AT+BIND=";

        _addr[0] = "0x90E2029E41D9";
        _addr[1] = "0xC4BE8423427B";
        _addr[2] = "0xC4BE8423427B";
        _addr[3] = "0xC4BE8423427B";
        _addr[4] = "0xC4BE8423427B";
      }
      
      String getAddr();
      String getName();
      String getMode();
      String getBind();
      int getType();
      void setName(String x);
      void setMode(String x);
      void setBind(String x);
      void setAddr(String x);    
      String readString();
      void writeString(String x); 
      void bindTo(int n);
};


AT::AT(int n, int t, String atEnter = "+++", String atExit = "AT+EXIT\r\n")
{
  _n = n;
  _t = t;
  _atEnter = atEnter;
  _atExit = atExit;

  switch(_n)
  {
    case 0: _dst = (Stream*)(&Serial); break;
    case 1: _dst = (&Serial1); break; 
  }
};

AT::~AT() { }

char AT::read()
{
  char x=0x00;

  if(_dst->available())
  {
    x = _dst->read();  
    delay(DELAY_XM);
  }

  return x;
}

void AT::write(char c)
{ 
  if(_dst->availableForWrite())
  {
    _dst->write(c);
    delay(DELAY_XS);
  }
}


String AT::readString()
{
  String x = "";

  char c = read();
  while(c != '\0' && c != 0x00)
  {
    if(c != '\n' && c != '\r')
    {
      x+=c; 
    }
    c = read();
  }

  delay(DELAY_XL);

  return x;
}


void AT::writeString(String z)
{ 
  if(_dst->availableForWrite())
  {
    _dst->print(z);
    //delay(DELAY_XL);
  }

  delay(DELAY_XL);
}


bool AT::open()
{ 
  if(_open) return;

  writeString(_atEnter);
  String x = readString();
    
  _open = true;
  return true;
}

bool AT::close()
{  
  writeString(_atExit);
  
  if(!_open) return true;

  String x = readString();
  _open = false;
  return true;
}

String AT::send(String m)
{
  open();
  writeString(m);

  String x = readString();
  return x;
}

ATDevice::ATDevice(int serialN, int devType, String atEnter = "+++", String atExit = "AT+EXIT\r\n")
{
    _device=&(AT(serialN, devType, atEnter, atExit));
    _devType = devType;
}


int ATDevice::getType() 
{
    return  _devType;
}

String ATDevice::getAddr() 
{
  String msg = _map[0] + "\r\n";
  _device->open();
  String y = _device->send(msg);
  _device->close();
  return y;
}

String ATDevice::getName()
{
  String msg = _map[2] + "\r\n";
   _device->open();
   String y= _device->send(msg); 
   _device->close();
   return y;
}

String ATDevice::getMode()
{
  String msg = _map[4] + "\r\n";
  _device->open();
  String y = _device->send(msg);
  _device->close();
  return y;
}

String ATDevice::getBind()
{
  String msg = _map[6] + "\r\n";
  _device->open();
  String y = _device->send(msg);
  _device->close();
  return y;
}


void ATDevice::setAddr(String x)
{
  _device->open();
  String y = _map[1]+x+"\r\n";
  String z = _device->send(y);
  _device->close();   
}

void ATDevice::setBind(String x)
{
  _device->open();
  String y = _map[7]+x+"\r\n";
  String z1 = _device->send(y);
  _device->close();   
}

void ATDevice::setName(String x)
{
  _device->open();
  String y = _map[3]+x+"\r\n";
  String z = _device->send(y);
  _device->close();   
}

void ATDevice::setMode(String x)
{
  _device->open();
  String y = _map[5]+x+"\r\n";
  String z = _device->send(y);
  _device->close();   
}

void ATDevice::writeString(String x)
{
  _device->writeString(x);
}

String ATDevice::readString()
{
  return _device->readString();
}

void ATDevice::bindTo(int n)
{
  if(!(n>=0 && n <= 4)) return;
  
  String addr = _addr[n];  
  _device->open();
  String name = _device->send(_map[2]+"\r\n");

  Serial.print("Binding "+name);
  Serial.println(" to "+addr);

  String y = _map[7]+addr+"\r\n";
  _device->send(y);
  _device->close();
}

#endif
