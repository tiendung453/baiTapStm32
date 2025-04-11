#include "motor.h"
extern TIM_HandleTypeDef htim3;

// xe tien 
void forward()
{
	
}
// xe lui
void backward()
{
}
// khoi dong
void start_wheel()
{
	HAL_GPIO_WritePin(GPIOA,BRAKE_Pin,1);
}
// tat banh da
void stop_wheel()
{
	HAL_GPIO_WritePin(GPIOA,BRAKE_Pin,0);
}
// tao pwm
void Motor_SetPWM(TIM_HandleTypeDef *htim, uint8_t channel, uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(htim, channel, pwm);
}

void clock_wise()
{
	HAL_GPIO_WritePin(GPIOA,DIR_Pin,1);
}

void counter_clock_wise()
{
	HAL_GPIO_WritePin(GPIOA,DIR_Pin,0);
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t constrain(uint32_t x, uint32_t min, uint32_t max)
{
	if(x <= min) return min;
	else if(x >= max) return max;
	else return x;
}
