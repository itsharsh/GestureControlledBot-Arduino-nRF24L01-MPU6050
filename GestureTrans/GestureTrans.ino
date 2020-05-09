#include<SPI.h>
#include<RF24.h>
#include<nRF24L01.h>
#include "printf.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3], yaw, pitch, roll;        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
MPU6050 mpu;


int CE_PIN = 8, CSN_PIN = 7;
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeOut = 0x10FAB52AE1LL;

int data = 0;

void setup()
{
  Serial.begin(115200);
  start_mpu();
  start_nrf();
  /*
    delay(1000);
    digitalWrite(radio_led, HIGH);
    get_mpu_data();
    //  pitch_err = pitch;

    delay(1000);
    digitalWrite(radio_led, LOW);
  */
  //  delay(2000);
}

void loop()
{
  get_mpu_data();
  Serial.print("ypr\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");

  if (pitch < 15 && pitch > -15 && roll < 15 && pitch > -15) {
    Serial.print("Stop");
    data = 0;
  }
  else if (pitch < -30) {
    Serial.print("Forward");
    data = 1;
  }
  else if (pitch > 30) {
    Serial.print("Backward");
    data = 2;
  }

  else if (roll < -30) {
    Serial.print("Left");
    data = 3;
  }
  else if (roll > 30) {
    Serial.print("Right");
    data = 4;
  }
  Serial.println();
  radio.write(&data, sizeof(data));
//  delay(200);
}

void start_nrf()
{
  radio.begin();
  printf_begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(110);
  radio.openWritingPipe(pipeOut);
  radio.printDetails();
  delay(2000);
}


void start_mpu()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void get_mpu_data()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  { /*
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);

      unsigned long currentMillis = millis();

      if (currentMillis - time1Hz >= 1000)
      {
       loopAt1Hz();
       time1Hz = currentMillis;
      }

      if (currentMillis - time5Hz >= 5000)
      {
       loopAt5Hz();
       time5Hz = currentMillis;
      }*/
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
//    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = (ypr[0] * 180) / M_PI;
    pitch = (ypr[1] * 180) / M_PI;
    roll = (ypr[2] * 180) / M_PI;
  }
}

