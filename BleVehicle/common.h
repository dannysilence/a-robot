#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <Arduino.h>

#define JOYSTICK_SEND_DELAY   100
#define JOYSTICK_DATA_LENGTH 0x0A
#define JOYSTICK_DATA_START  0xAA
#define JOYSTICK_DATA_END    0xBB

#ifdef ARDUINO_AVR_UNO

#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3);

#endif

class Logger
{
    private:
        bool useLogs;
        Stream* stream;
    public:
        Logger(Stream* out, bool enabled) 
        {
            this->useLogs = enabled;
            this->stream = out;
        }
        Logger(Stream* out)
        {
            this->useLogs = false;
            this->stream = out;
        }
        Logger()
        {
            this->useLogs = false;
            this->stream = &(Serial);
        }

        void write(String message)
        {
            if(this->useLogs)
            {
                this->stream->println(message);
            }
        }
        void println(String message)
        {
            if(this->useLogs)
            {
                this->stream->println(message);
            }
        }

        void write(String message, String a)
        {
            String x =  message + a;
            this->write(x);
        }

        void write(String message, String a, String b)
        {
            String x =  message + a + b;
            this->write(x);
        }

        void write(String message, String a, String b, String c)
        {
            String x =  message + a + b + c;
            this->write(x);
        }

        void write(String message, String a, String b, String c, String d)
        {
            String x =  message + a + b + c + d;
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e)
        {
            String x =  message + a + b + c + d + e;
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f)
        {
            String x =  message + a + b + c + d + e + f; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g)
        {
            String x =  message + a + b + c + d + e + f + g; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g, String h)
        {
            String x =  message + a + b + c + d + e + f + g + h; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g, String h, String k)
        {
            String x =  message + a + b + c + d + e + f + g + h + k; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g, String h, String k, String l)
        {
            String x =  message + a + b + c + d + e + f + g + h + k + l; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g, String h, String k, String l, String m)
        {
            String x =  message + a + b + c + d + e + f + g + h + k + l + m; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g, String h, String k, String l, String m, String n)
        {
            String x =  message + a + b + c + d + e + f + g + h + k + l + m + n; 
            this->write(x);
        }
        void write(String message, String a, String b, String c, String d, String e, String f, String g, String h, String k, String l, String m, String n, String o)
        {
            String x =  message + a + b + c + d + e + f + g + h + k + l + m + n + o; 
            this->write(x);
        }
        bool isEnabled() 
        {
            return this->useLogs;
        }
        void enable() 
        {
            this->useLogs = true;
        }
        void disable() 
        {
            this->useLogs = false;
        }
};

#endif
