#ifndef GAMEPAD_H
#define GAMEPAD_H

typedef struct GamepadState
{
  byte Joystick1[2];
  byte Joystick2[2];
  byte Id;
  byte Buttons[3];
  
  static void clone(GamepadState* src, GamepadState* dst)
  {
    dst->Id = src->Id;
    dst->Buttons[0] = src->Buttons[0];
    dst->Buttons[1] = src->Buttons[1];
    dst->Buttons[2] = src->Buttons[2];
    dst->Joystick1[0] = src->Joystick1[0];
    dst->Joystick1[1] = src->Joystick1[1];
    dst->Joystick2[0] = src->Joystick2[0];
    dst->Joystick2[1] = src->Joystick2[1];
  }
  
  static GamepadState fromBytes(byte* buf)
  {
    GamepadState x;    

    x.Joystick1[0] = buf[0];
    x.Joystick1[1] = buf[1];
    x.Joystick2[0] = buf[2];
    x.Joystick1[1] = buf[3];
    x.Id = buf[4];
    x.Buttons[0] = buf[5];
    x.Buttons[1] = buf[6];
    x.Buttons[2] = buf[7];

    return x;
  }
  
  static byte* toBytes(GamepadState state)
  {
    byte x[sizeof(GamepadState)];

    x[0] = state.Joystick1[0];
    x[1] = state.Joystick1[1];
    x[2] = state.Joystick2[0];
    x[3] = state.Joystick2[1];
    x[4] = state.Id;
    x[5] = state.Buttons[0];
    x[6] = state.Buttons[1];
    x[7] = state.Buttons[2];

    return x;
  }
} GamepadState;


class Gamepad 
{
  private:
    const byte MESSAGE_START = 0xAA;
    const byte MESSAGE_END = 0xBB;
    const byte MESSAGE_LENGTH = sizeof(GamepadState)+2;
    bool hasReadData;
    byte numReceived;
    byte receivedBytes[(sizeof(GamepadState)+2)*2];
    Stream* io;    
    Stream* log;    
  public:
    Gamepad(Stream* ioStream, Stream* logStream)
    {
      this->hasReadData = false;
      this->numReceived = 0;
      this->io  = ioStream;
      this->log = logStream;
    }

    bool receive(GamepadState& state)
    {
      static bool recvInProgress = false;
      static uint8_t ndx = 0;
      byte rb;   

      while (this->io->available() > 0 && this->hasReadData == false) 
      {
        rb = this->io->read();

        if (recvInProgress == true) 
        {
            if (rb != MESSAGE_END) 
            {
                this->receivedBytes[ndx] = rb;
                if (ndx++ >= MESSAGE_LENGTH*2) ndx = MESSAGE_LENGTH*2 - 1;
            }
            else 
            {
                this->receivedBytes[ndx] = '\0'; 
                recvInProgress = false;
                this->numReceived = ndx;  
                ndx = 0;
                this->hasReadData = true;
            }
        }
        else if (rb == MESSAGE_START) recvInProgress = true;
      }

      if (this->hasReadData == true) 
      {
        this->hasReadData = false;

        GamepadState x = GamepadState::fromBytes(this->receivedBytes);
        GamepadState::clone(&x, &state);

        String m = "This came in: ";
        for (byte n = 0; n < this->numReceived; n++) { m += String(this->receivedBytes[n], HEX); m += " "; }
         
        this->log->println(m);
        
        return true;
      }

      return false;
    }

    void send(GamepadState state)
    {
      byte* buf = GamepadState::toBytes(state);
      
      this->io->write(MESSAGE_START);
      this->io->write(buf, sizeof(GamepadState));
      this->io->write(MESSAGE_END);
    }
};


#endif
