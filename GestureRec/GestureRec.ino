#include<SPI.h>
#include<RF24.h>
#include<nRF24L01.h>
#include "printf.h"

int CE_PIN = 8, CSN_PIN = 7;
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeIn = 0x10FAB52AE1LL;

int data;

int lm1 = 3, lm2 = 4, rm1 = 5, rm2 = 6;
int throttle = 255;

void setup()
{
  Serial.begin(115200);
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);

  start_radio();
  //  delay(2000);
}

void loop()
{
  get_radio_data();
}


void start_radio()
{
  radio.begin();
  printf_begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(110);
  radio.openReadingPipe(0, pipeIn);
  radio.startListening();
  radio.printDetails();
  delay(2000);
}

void get_radio_data()
{
  if (radio.available())
  {
    radio.read(&data, sizeof(&data));
    switch (data) {
      case 0: go_stop();
        break;
      case 1: go_forward();
        break;
      case 2: go_reverse();
        break;
      case 3: go_left();
        break;
      case 4: go_right();
        break;
      default: break;
    }
  }
}


void motor_val(int a, int b, int c, int d)
{
  analogWrite(lm1, a);
  analogWrite(lm2, b);
  analogWrite(rm1, c);
  analogWrite(rm2, d);
}

void go_forward ()
{
  Serial.println("Forward");
  motor_val(0, throttle, 0, throttle);
}

void go_reverse()
{
  Serial.println("Reverse");
  motor_val(throttle, 0, throttle, 0);
}

void go_left()
{
  Serial.println("Left");
  motor_val(throttle, 0, 0, throttle);
}

void go_right()
{
  Serial.println("Right");
  motor_val(0, throttle, throttle, 0);
}
void go_stop()
{
  Serial.println("Stop");
  motor_val(0, 0, 0, 0);
}
