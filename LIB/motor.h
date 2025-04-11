#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "main.h"

void forward();
void backward();
void start_wheel();
void stop_wheel();
void clock_wise();
void counter_clock_wise();
void Motor_SetPWM(TIM_HandleTypeDef *htim, uint8_t channel, uint16_t pwm);
float map(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t constrain(uint32_t x, uint32_t min, uint32_t max);


#endif /* MOTOR_H_ */
