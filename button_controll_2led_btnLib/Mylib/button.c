#include "button.h"

// --------- var button --------

#define DEBOUNCE_TIME_MS 15
#define SHORT_PRESS_MAX_DURATION_MS 1000
#define LONG_PRESS_DURATION_MS 3000

__weak void btn_pressing_callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {}

__weak void btn_press_short_callback() {}

__weak void btn_release_callback() {}

__weak void btn_timeout_callback() {}

void button_handle(Button_Typedef *ButtonX) 
	{
    uint8_t sta = HAL_GPIO_ReadPin(ButtonX->GPIOx, ButtonX->GPIO_Pin);
    if (sta != ButtonX -> btn_filter) {
        ButtonX -> btn_filter = sta;
        ButtonX -> is_debouncing = 1;
        ButtonX -> time_debounce = HAL_GetTick();
    }
    if (ButtonX -> is_debouncing && (HAL_GetTick() - ButtonX -> time_debounce >= DEBOUNCE_TIME_MS)) {
        ButtonX -> btn_current = ButtonX -> btn_filter;
        ButtonX -> is_debouncing = 0;
    }
    // ----------- handle state change -----------
    if (ButtonX -> btn_current != ButtonX -> btn_last) {
        if (ButtonX -> btn_current == 0) { // button pressed
            ButtonX -> is_press_timeout = 1;
            btn_pressing_callback(ButtonX->GPIOx, ButtonX->GPIO_Pin);
            ButtonX -> time_start_press = HAL_GetTick();
        } else { // button released
            if (HAL_GetTick() - ButtonX -> time_start_press <= SHORT_PRESS_MAX_DURATION_MS) {
                btn_press_short_callback();
            }
            btn_release_callback();
            ButtonX -> is_press_timeout = 0;
        }
        ButtonX -> btn_last = ButtonX -> btn_current;
    }
    // ------ handle long press ------
    if (ButtonX -> is_press_timeout && (HAL_GetTick() - ButtonX -> time_start_press >= LONG_PRESS_DURATION_MS)) {
        btn_timeout_callback();
        ButtonX -> is_press_timeout = 0;
    }
}

void button_init(Button_Typedef *ButtonX, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	ButtonX -> GPIOx = GPIOx;
	ButtonX -> GPIO_Pin = GPIO_Pin;
}