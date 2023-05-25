#include <DHT.h>

#include <WiFi.h>



#define BLYNK_TEMPLATE_ID "TMPL37OK_5WUR"
#define BLYNK_DEVICE_NAME "IOTKIT"
#define BLYNK_FIRMWARE_VERSION  "0.1.0"
#define BLYNK_PRINT Serial
#define APP_DEBUG
#include "BlynkEdgent.h"
BlynkTimer timer;





#define DHT11PIN 16  //IO16  pin connected with DHT
#define LDR_PIN 33  //D33  pin connected with LDR




#define RelayPin1 17  //IO17
#define RelayPin2 25  //IO25
#define RelayPin3 26  //IO26
#define RelayPin4 27  //IO27

#define SwitchPin1 15  //IO15
#define SwitchPin2 2  //IO2
#define SwitchPin3 4  //IO4
#define SwitchPin4 13  //IO13

// blink button
#define VPIN_1    V0
#define VPIN_2    V1
#define VPIN_3    V2
#define VPIN_4    V3
#define VPIN_SPEED V4
#define VPIN_ALL_OFF V5
#define VPIN_HUMIDITY V6
#define VPIN_TEMPERATURE V7


// Relay State
bool toggleState_1 = LOW; //Define integer to remember the toggle state for relay 1
bool toggleState_2 = LOW; //Define integer to remember the toggle state for relay 2
bool toggleState_3 = LOW; //Define integer to remember the toggle state for relay 3
bool toggleState_4 = LOW; //Define integer to remember the toggle state for relay 4

// Switch State
bool SwitchState_1 = LOW;
bool SwitchState_2 = LOW;
bool SwitchState_3 = LOW;
bool SwitchState_4 = LOW;


float temperature = 0;
float humidity   = 0;
int   ldrVal;

//int pwm;      // BOTH INT for Fan Speed
//int map_pwm;  // BOTH INT for Fan Speed

DHT dht(DHT11PIN, DHT11);

void readSensor() {
  ldrVal = map(analogRead(LDR_PIN), 0, 4095, 10, 0);

  float humi = dht.readHumidity();
  float temp = dht.readTemperature();

  if (isnan(humi) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  else {
    humidity = humi;
    temperature = temp;
    // Serial.println(temperature1);
    // Serial.println(ldrVal);
  }
}

void sendSensor()
{
  readSensor();
  Blynk.virtualWrite(VPIN_HUMIDITY, humidity);
  Blynk.virtualWrite(VPIN_TEMPERATURE, temperature);
  // blynk write

}

BLYNK_WRITE(VPIN_1) {
  toggleState_1 = param.asInt();
  //  digitalWrite(RelayPin1, toggleState_1);
  if (toggleState_1 == 1) {
    digitalWrite(RelayPin1, LOW);
  }
  else {
    digitalWrite(RelayPin1, HIGH);
  }
}

BLYNK_WRITE(VPIN_2) {
  toggleState_2 = param.asInt();
  //  digitalWrite(RelayPin2, toggleState_2);
  if (toggleState_2 == 1) {
    digitalWrite(RelayPin2, LOW);
  }
  else {
    digitalWrite(RelayPin2, HIGH);
  }
}

BLYNK_WRITE(VPIN_3) {
  toggleState_3 = param.asInt();
  //  digitalWrite(RelayPin3, toggleState_3);
  if (toggleState_3 == 1) {
    digitalWrite(RelayPin3, LOW);
  }
  else {
    digitalWrite(RelayPin3, HIGH);
  }
}

BLYNK_WRITE(VPIN_4) {
  toggleState_4 = param.asInt();
  //  digitalWrite(RelayPin4, toggleState_4);
  if (toggleState_4 == 1) {
    digitalWrite(RelayPin4, LOW);
  }
  else {
    digitalWrite(RelayPin4, HIGH);
  }
}

BLYNK_WRITE(VPIN_ALL_OFF) {
  switchoff();
}

void switchoff() {
  toggleState_1 = 0; digitalWrite(RelayPin1, HIGH); Blynk.virtualWrite(VPIN_1, toggleState_1); delay(100);
  toggleState_2 = 0; digitalWrite(RelayPin2, HIGH); Blynk.virtualWrite(VPIN_2, toggleState_2); delay(100);
  toggleState_3 = 0; digitalWrite(RelayPin3, HIGH); Blynk.virtualWrite(VPIN_3, toggleState_3); delay(100);
  toggleState_4 = 0; digitalWrite(RelayPin4, HIGH); Blynk.virtualWrite(VPIN_4, toggleState_4); delay(100);
}

void manual_control() {
  if (digitalRead(SwitchPin1) == LOW && SwitchState_1 == LOW) {
    digitalWrite(RelayPin1, LOW);
    Blynk.virtualWrite(VPIN_1, HIGH);
    toggleState_1 = HIGH;
    SwitchState_1 = HIGH;
    Serial.println("Switch-1 on");
  }
  if (digitalRead(SwitchPin1) == HIGH && SwitchState_1 == HIGH) {
    digitalWrite(RelayPin1, HIGH);
    Blynk.virtualWrite(VPIN_1, LOW);
    toggleState_1 = LOW;
    SwitchState_1 = 0;
    Serial.println("Switch-1 off");
  }
  if (digitalRead(SwitchPin2) == LOW && SwitchState_2 == LOW) {
    digitalWrite(RelayPin2, LOW);
    Blynk.virtualWrite(VPIN_2, HIGH);
    toggleState_2 = HIGH;
    SwitchState_2 = HIGH;
    Serial.println("Switch-2 on");
  }
  if (digitalRead(SwitchPin2) == HIGH && SwitchState_2 == HIGH) {
    digitalWrite(RelayPin2, HIGH);
    Blynk.virtualWrite(VPIN_2, LOW);
    toggleState_2 = LOW;
    SwitchState_2 = LOW;
    Serial.println("Switch-2 off");
  }
  if (digitalRead(SwitchPin3) == LOW && SwitchState_3 == LOW) {
    digitalWrite(RelayPin3, LOW);
    Blynk.virtualWrite(VPIN_3, HIGH);
    toggleState_3 = HIGH;
    SwitchState_3 = HIGH;
    Serial.println("Switch-3 on");
  }
  if (digitalRead(SwitchPin3) == HIGH && SwitchState_3 == HIGH) {
    digitalWrite(RelayPin3, HIGH);
    Blynk.virtualWrite(VPIN_3, LOW);
    toggleState_3 = LOW;
    SwitchState_3 = LOW;
    Serial.println("Switch-3 off");
  }
  if (digitalRead(SwitchPin4) == LOW && SwitchState_4 == LOW) {
    digitalWrite(RelayPin4, LOW);
    Blynk.virtualWrite(VPIN_4, HIGH);
    toggleState_4 = HIGH;
    SwitchState_4 = HIGH;
    Serial.println("Switch-4 on");
  }
  if (digitalRead(SwitchPin4) == HIGH && SwitchState_4 == HIGH) {
    digitalWrite(RelayPin4, HIGH);
    Blynk.virtualWrite(VPIN_4, LOW);
    toggleState_4 = LOW;
    SwitchState_4 = LOW;
    Serial.println("Switch-4 off");
  }



}

void setup() {
  Serial.begin(115200);
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);

  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP);
  pinMode(SwitchPin4, INPUT_PULLUP);

  digitalWrite(RelayPin1, HIGH);
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);
  digitalWrite(RelayPin4, HIGH);

  BlynkEdgent.begin();
  timer.setInterval(2000L, sendSensor);
  dht.begin();
}

void loop() {
  manual_control();
  BlynkEdgent.run();
  timer.run();

}
