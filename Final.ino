#include <SoftwareSerial.h>
SoftwareSerial BT1(4, 3); // RX | TX

String configInicial[] = //Para configurar a serial indicado
{
  "AT+CIOBAUD=9600",
  "END" //Para reconocer el final de la orden
};

String configModo[] =   //Si es que es necesario volver a configurar como Access Point
{
  "AT+CIPMUX=1",
  "AT+CIPSERVER=1,8888",
  "END"
};

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
float v = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);
  BT1.begin(115200);
  int index = 0;
  while (configInicial[index] != "END")
  {
    BT1.println(configInicial[index++]);
  }
  Serial.end();
  BT1.end();
  Serial.begin(9600);
  BT1.begin(9600);

  delay(100);
  index = 0;
  while (configModo[index] != "END")
  {
    BT1.println(configModo[index++]);
  }
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);     //Hay overflow

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  if (BT1.available())
  { char c = BT1.read() ;
    Serial.print(c);
  }
  if (Serial.available())
  { char c = Serial.read();
    BT1.print(c);
  }

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if ( v - (ypr[2] * 180 / M_PI) > 6.0)
    {
      escribir("b");
      delay(100);
    }
    else if ( v - (ypr[2] * 180 / M_PI) > 4.0)
    {
      escribir("c");
      delay(100);
    }
    else if ( v - (ypr[2] * 180 / M_PI) > 2.0)
    {
      escribir("d");
      delay(100);
    }
    v = ypr[2] * 180 / M_PI;
#endif
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void escribir(String text)
{
  BT1.println("AT+CIPSEND=0,1");
  if (BT1.find(">"))             // Si se recibe el mensaje
  {
    Serial.println(text);
    BT1.println(text);            //mandamos el mensaje por el wifi
    delay(10);
    while ( BT1.available() > 0 )
    {
      if (  BT1.find("SEND OK") )  //buscamos "ok" y luego salimos
        break;
    }
  }
}
