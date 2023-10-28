/*
 * HeatPadControl.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Charlie
 */
#include <Arduino.h>

#ifndef HEATPADCONTROL_H_
#define HEATPADCONTROL_H_

#define HEAT_OFF 0
#define HEAT_ON  1

class HeatPadControl {
  public:
    HeatPadControl();
    ~HeatPadControl();

    void poll(float tempF);
    void set(float setPointF, float hysteresisF);
    void test();
    uint8_t getStatus();
    float getLastTemp();
    float getSetPoint();
  private:
    float setpoint, idleband, lastTemp;
    uint8_t lastHeaterPin;

};

#endif /* HEATPADCONTROL_H_ */
