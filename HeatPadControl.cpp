/*
 * HeatPadControl.cpp
 *
 *  Created on: Oct 4, 2023
 *      Author: Charlie
 */
#include "HeatPadControl.h"

#include <Arduino.h>

#define _HEATER_PIN A2

// Constructor/Destructor

HeatPadControl::HeatPadControl() {
  pinMode(_HEATER_PIN, OUTPUT);
  lastHeaterPin = HIGH;						//OFF at startup
  digitalWrite(_HEATER_PIN, lastHeaterPin);
}

HeatPadControl::~HeatPadControl(){

}

float HeatPadControl::getLastTemp() {
  return lastTemp;
}

float HeatPadControl::getSetPoint() {
  return setpoint;
}

uint8_t HeatPadControl::getStatus() {
  uint8_t rc;
  if (lastHeaterPin == HIGH)
	rc = HEAT_OFF;
  else
	rc = HEAT_ON;

  return rc;
}

void HeatPadControl::test() {
  if (lastHeaterPin == HIGH)
	digitalWrite(_HEATER_PIN, LOW);
  else
	digitalWrite(_HEATER_PIN, HIGH);

  lastHeaterPin = !lastHeaterPin;
}

void HeatPadControl::poll(float tempF) {
  lastTemp = tempF;
  Serial.print("\nActual-");Serial.print(tempF);
  Serial.print(" Setpoint-");Serial.print(setpoint);
  Serial.print(" Idleband-");Serial.println(idleband);
  if (tempF < (setpoint-idleband/2)) {
	digitalWrite(_HEATER_PIN, LOW);
	lastHeaterPin = LOW;
    Serial.println(" -ON");
  } else if (tempF > (setpoint+idleband/2)) {
	digitalWrite(_HEATER_PIN, HIGH);
	lastHeaterPin = HIGH;
    Serial.println(" -OFF");
  } else {
	// let it ride!
	Serial.print(" -SOAKING ");
	if (lastHeaterPin==LOW)
	  Serial.println("HEAT ON");
	else
	  Serial.println("HEAT OFF");
	digitalWrite(_HEATER_PIN, lastHeaterPin);
  }

}

void HeatPadControl::set(float setPointF, float idlebandF) {
  setpoint = setPointF;
  idleband = idlebandF;
  Serial.println("sync");
}



