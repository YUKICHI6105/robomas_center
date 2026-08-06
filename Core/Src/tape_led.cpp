#include <tape_led.hpp>

extern TIM_HandleTypeDef htim2;

namespace tapeled{
	State led_state = {0, 0, 0};

	// TIM2 channel wiring (robomas_center.ioc): CH1=Blue12, CH2=Red12, CH3=Green12
	// TLP172AM photorelay switching lag compensation (see robomas_center notes):
	// driven at IF≈2mA (1kΩ/3.3V), below Toshiba's recommended min (5mA), so tON/tOFF
	// run slower than the datasheet's typ spec (0.6ms/0.1ms @5mA). Conservative estimate
	// at this drive current: tON≈1.5ms, tOFF≈0.1-0.2ms, net offset (tON-tOFF)≈1.4ms.
	// At the 50Hz/256-step PWM below (20ms period, ~78us/step), that is ~18 steps.
	constexpr uint16_t kOnOffsetTicks = 18;

	static uint16_t compensate(uint8_t value){
		return value == 0 ? 0 : (uint16_t)value + kOnOffsetTicks;
	}

	bool updateLedState(State state){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compensate(state.blue));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compensate(state.red));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compensate(state.green));
		return true;
	}

	State getLedState(){
		return led_state;
	}

	bool setLedState(State state){
		led_state = state;
		return updateLedState(state);
	}
}
