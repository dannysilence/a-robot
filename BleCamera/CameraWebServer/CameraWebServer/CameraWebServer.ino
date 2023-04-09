#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/rtc.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#include "rom/rtc.h"
#endif

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <BleSerial.h>
BleSerial ble;

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

#define JOYSTICK_DATA_LENGTH 0x40
#define JOYSTICK_DATA_START  0xAA
#define JOYSTICK_DATA_END    0xBB

const char* ssid = "putinHUYLO";
const char* password = "1488#8841";

void startCameraServer();
IPAddress ip;

// Joystick Message Retrieving Parts
bool newData       = false;
uint8_t numReceived = 0;
uint8_t receivedBytes[JOYSTICK_DATA_LENGTH];

int       i2cSDA = 14;
int       i2cSCL = 15;
int       i2cNUM = 33;  // 32 - main camera, 33 - secoondary camera
uint32_t  i2cFRQ = 100000;
//TwoWire i2cDevice = TwoWire(0);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  //  digitalWrite(powerControlSettings.powerControlOutputPin, HIGH);
  //  delay(500);
  //  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); //enable brownout detector
  //
  Serial.begin(115200);
  //  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  //Start the BLE Serial
  //Enter the Bluetooth name here
  ble.begin("BleCamera");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  ip = WiFi.localIP();

  String m1 = "Camera Ready! Use 'http://";
  m1 += ip.toString();
  m1 += "' to connect";  
  Serial.println(m1);

  String m2 = "I2C endpoint started at address 0x";
  m2 += String(i2cNUM, HEX);
  
  Wire.begin(i2cNUM, i2cSDA, i2cSCL, i2cFRQ);
  //Wire.onReceive(i2cEvent1);
         Wire.onRequest(i2cEvent2);
  
  Serial.println(m2);
}

//void i2cEvent1(int num)
//{
//  String msg;
//  
//  while(1 < i2cDevice.available()) // loop through all but the last
//  {
//    char c = i2cDevice.read(); // receive byte as a character
//    msg += c;
//  }
//
//  Serial.print("Received from I2C: ");
//  Serial.println(msg);
////
////  int x = Wire.read();    // receive byte as an integer
////  Serial.println(x);         // print the integer
//}

void i2cEvent2()
{
  String x = ip.toString() + "                                                                ";
  unsigned char buf[x.length()];
  x.getBytes(buf, x.length());

  Wire.write(buf, 64);
  
  Serial.println("Wrote IP to I2C");  
}

void bleOut(String m)
{
  int l = m.length();
  byte bytesStart[2]; bytesStart[0] = (byte)0x11; bytesStart[1] = (byte)0x22;
  byte bytesEnd[2]; bytesEnd[0] = (byte)0x22; bytesEnd[1] = (byte)011;
  byte bytesBuf[l];
  m.getBytes(bytesBuf, l);

  ble.write(bytesStart, 2);
  ble.write(bytesBuf, l);
  ble.write(bytesEnd, 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  // delay(10000);

  //The usage is similar to Serial
  //ble.println("Hello!");
  //delay(1000);
  Stream* _ctrl = &Serial;
  String cmd = "";
  //receiveBytes(_ctrl);
  //if(showNewData())
  if (_ctrl->available())
  {
    cmd = _ctrl->readString();
    //    char c0 = (char)receivedBytes[0];
    //    char c1 = (char)receivedBytes[1];
    //    char c2 = (char)receivedBytes[2];
    //    char c3 = (char)receivedBytes[3];
    //    char c4 = (char)receivedBytes[4];
    //    char c5 = (char)receivedBytes[5];
    //    String cmd = String(c0); cmd += c1; cmd += c2; cmd += c3; cmd += c4; cmd += c5;
    if (cmd.startsWith("?ip")) {
      Serial.println(ip);
    } else {
      if (cmd.startsWith("?stream")) {
        String x = "http://";
        x += ip.toString();
        x += ":81/stream";
        Serial.println(x);
      } else
        Serial.println("Received strange input:" + cmd);
    }
  }
}


void receiveBytes(Stream* stream)
{
  static bool recvInProgress = false;
  static uint8_t ndx = 0;
  uint8_t rb;

  while (stream->available() > 0 && newData == false)
  {
    rb = stream->read();

    if (recvInProgress == true)
    {
      if (rb != JOYSTICK_DATA_END)
      {
        receivedBytes[ndx] = rb;
        if (ndx++ >= JOYSTICK_DATA_LENGTH) ndx = JOYSTICK_DATA_LENGTH - 1;
      }
      else if (stream->available() /*? stream->read() == JOYSTICK_DATA_ENDLO : false*/)
      {
        receivedBytes[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        numReceived = ndx;  // save the number for use when printing
        ndx = 0;
        newData = true;
      }
    }
    else if (rb == JOYSTICK_DATA_START /*&& ((stream->available() ? stream->read() == JOYSTICK_DATA_START : false))*/) recvInProgress = true;
  }
}

bool showNewData()
{
  if (newData == true)
  {
    String m = "This came in: ";
    for (byte n = 0; n < numReceived; n++) {
      m += String(receivedBytes[n], HEX);
      m += " ";
    }

    Serial.println(m);

    newData = false;
    return true;
  }

  return false;
}
