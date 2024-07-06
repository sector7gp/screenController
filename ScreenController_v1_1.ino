//Release 1.1 0607024 sector7gp
#include <Arduino.h>
#include <BasicEncoder.h>
#include <Servo.h>
#include <EEPROM.h>

#define upperLimit 500
#define lowerLimit 1
#define millisMin 2000
#define millisMax 1000

//Define Switch Pins
#define S1 A1
#define S2 A2
#define S3 A4  //quedo mal cableado por eso esta invertido el orden
#define S4 A3  //quedo mal cableado por eso esta invertido el orden

//Define Led Pins
#define L1 4
#define L2 7
#define L3 8
#define L4 13
#define BR 5  // PWM para controlar el brillo

//Define servo Pins
#define M1 9
#define M2 10
#define M3 11
#define M4 6

//Define Day/Night
#define DAYNIGHT A7

//Define Encoder pins
const int8_t pinA = 3;
const int8_t pinB = 2;
#define SPST A5

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


int dayNight = false;
bool daynightFlag = false;
bool spst = false;
bool flag_spst = true;
byte sw[4] = { S1, S2, S3, S4 };
byte led[4] = { L1, L2, L3, L4 };
unsigned int ledValue[4];
unsigned int servo[4] = { M1, M2, M3, M4 };
unsigned int servoValue[4];

byte bright = 250;

unsigned long timer1 = millis();
bool state1 = true;
byte ledIndex = 1;  //indice para recorrer los leds
BasicEncoder encoder(pinA, pinB);

void setup() {
  Serial.begin(9600);

  if (analogRead(DAYNIGHT) > 100) {
    daynightFlag = true;
  }
  pinMode(SPST, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) {
    pinMode(sw[i], INPUT_PULLUP);
    pinMode(led[i], OUTPUT);
  }
  pinMode(DAYNIGHT, INPUT_PULLUP);
  pinMode(BR, OUTPUT);
  analogWrite(BR, bright);  // set the led brigthness, for the moment is fixed
  Serial.print("<");
  for (int i = 0; i < 4; i++) {
    servoValue[i] = EEPROM.get(i * 2, servoValue[i]);
    Serial.print(servoValue[i]);
    if (i < 3) Serial.print("-");
  }
  Serial.println(">");
}

void loop() {
  encoder.service();
  spst = digitalRead(SPST);

  //A6 and A7 can be use as digital, so the fix
  dayNight = analogRead(DAYNIGHT);
  //Serial.println(dayNight);
  if (dayNight > 100) {
    dayNight = 1;
  } else {
    dayNight = 0;
  }



  if (dayNight && daynightFlag) {  //NIGHTMODE
    daynightFlag = 0;
    servoValue[0] = EEPROM.get(0, servoValue[0]);
    servoValue[1] = EEPROM.get(2, servoValue[1]);
    servoValue[2] = EEPROM.get(4, servoValue[2]);
    servoValue[3] = EEPROM.get(6, servoValue[3]);



    //set the servos
    servo1.write(map(servoValue[0], lowerLimit, upperLimit, millisMax, millisMin));
    servo2.write(map(servoValue[1], lowerLimit, upperLimit, millisMax, millisMin));
    servo3.write(map(servoValue[2], lowerLimit, upperLimit, millisMax, millisMin));
    servo4.write(map(servoValue[3], lowerLimit, upperLimit, millisMax, millisMin));
    delay(500);

    servo1.attach(M1);
    servo2.attach(M2);
    servo3.attach(M3);
    servo4.attach(M4);
    Serial.println("Attaching servos...-night-");
    delay(5000);

    Serial.println("NIGHT MODE");
    Serial.print(servoValue[0]);
    Serial.print(";");
    Serial.print(servoValue[1]);
    Serial.print(";");
    Serial.print(servoValue[2]);
    Serial.print(";");
    Serial.println(servoValue[3]);

    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    delay(2000);
  }
  if (!dayNight && !daynightFlag) {  //DAYMODE
    daynightFlag = 1;
    //setting the starting default position
    Serial.println("DAY MODE");
    Serial.println("Default position...-day-");
    servo1.write(map(lowerLimit, lowerLimit, upperLimit, millisMax, millisMin));
    servo2.write(map(lowerLimit, lowerLimit, upperLimit, millisMax, millisMin));
    servo3.write(map(lowerLimit, lowerLimit, upperLimit, millisMax, millisMin));
    servo4.write(map(lowerLimit, lowerLimit, upperLimit, millisMax, millisMin));
    delay(500);

    //Attaching the servos
    servo1.attach(M1, millisMin, millisMax);
    servo2.attach(M2, millisMin, millisMax);
    servo3.attach(M3, millisMin, millisMax);
    servo4.attach(M4, millisMin, millisMax);
    delay(500);
    Serial.println(map(lowerLimit, lowerLimit, upperLimit, millisMax, millisMin));


    //
    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    delay(100);
    // //end recall
  }


  int encoder_change = encoder.get_change();
  if (encoder_change && (ledIndex != 254)) {
    servoValue[ledIndex] += encoder_change;
    if (servoValue[ledIndex] <= (lowerLimit)) {
      servoValue[ledIndex] = lowerLimit;
    } else if (servoValue[ledIndex] >= (upperLimit)) {
      servoValue[ledIndex] = upperLimit;
    }
    Serial.println(servoValue[ledIndex]);
  }

  switch (readSwitch()) {
    case 8:
      ledIndex = 0;
      servo1.attach(M1, millisMin, millisMax);
      servo1.write(map(servoValue[ledIndex], lowerLimit, upperLimit, millisMax, millisMin));
      //Serial.println(servoValue[ledIndex]);
      break;
    case 4:
      ledIndex = 1;
      servo1.attach(M2, millisMin, millisMax);
      servo1.write(map(servoValue[ledIndex], lowerLimit, upperLimit, millisMax, millisMin));
      break;
    case 2:
      ledIndex = 2;
      servo1.attach(M3, millisMin, millisMax);
      servo1.write(map(servoValue[ledIndex], lowerLimit, upperLimit, millisMax, millisMin));
      break;
    case 1:
      ledIndex = 3;
      servo1.attach(M4, millisMin, millisMax);
      servo1.write(map(servoValue[ledIndex], lowerLimit, upperLimit, millisMax, millisMin));
      break;
    case 0:
      delay(50);
      ledIndex = 254;
      servo1.detach();
      servo2.detach();
      servo3.detach();
      servo4.detach();
      break;
    default:
      while (millis() - timer1 > 200) {
        digitalWrite(led[0], state1);
        digitalWrite(led[1], state1);
        digitalWrite(led[2], state1);
        digitalWrite(led[3], state1);
        state1 = !state1;
        timer1 = millis();
      }
      break;
  }
  if (ledIndex == 254) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(led[i], LOW);
    }
  } else digitalWrite(led[ledIndex], HIGH);

  //Read the SPST to save the value
  if (spst && flag_spst && (ledIndex != 254)) {
    flag_spst = 0;
  } else if (!spst && !flag_spst) {
    flag_spst = 1;


    Serial.print(servoValue[0]);
    EEPROM.put(0, servoValue[0]);
    Serial.print(";");
    Serial.print(servoValue[1]);
    EEPROM.put(2, servoValue[1]);
    Serial.print(";");
    Serial.print(servoValue[2]);
    EEPROM.put(4, servoValue[2]);
    Serial.print(";");
    Serial.println(servoValue[3]);
    EEPROM.put(6, servoValue[3]);

    //set the servos
    servo1.write(map(servoValue[0], lowerLimit, upperLimit, millisMax, millisMin));
    servo2.write(map(servoValue[1], lowerLimit, upperLimit, millisMax, millisMin));
    servo3.write(map(servoValue[2], lowerLimit, upperLimit, millisMax, millisMin));
    servo4.write(map(servoValue[3], lowerLimit, upperLimit, millisMax, millisMin));

    servo1.attach(M1, millisMin, millisMax);
    servo2.attach(M2, millisMin, millisMax);
    servo3.attach(M3, millisMin, millisMax);
    servo4.attach(M4, millisMin, millisMax);
    Serial.println("Attaching servos -saving-...");

    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    delay(1);
    Serial.println("SAVED");
    Serial.println("Data in memory:");
    Serial.print(EEPROM.get(0, servoValue[0]));
    Serial.print(";");
    Serial.print(EEPROM.get(2, servoValue[1]));
    Serial.print(";");
    Serial.print(EEPROM.get(4, servoValue[2]));
    Serial.print(";");
    Serial.println(EEPROM.get(6, servoValue[3]));
  }
}

unsigned int readSwitch() {
  unsigned int _read = 0;
  _read |= (!digitalRead(S1) << 3) & 0x08;
  _read |= (!digitalRead(S2) << 2) & 0x04;
  _read |= (!digitalRead(S3) << 1) & 0x02;
  _read |= (!digitalRead(S4) << 0) & 0x01;
  //Serial.println(_read);
  return _read;
}