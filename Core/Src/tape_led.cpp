#include <tape_led.hpp>

extern TIM_HandleTypeDef htim2;

namespace tapeled{
	State led_state = {0, 0, 0};

	// TIM2 channel wiring (robomas_center.ioc): CH1=Blue12, CH2=Red12, CH3=Green12
	// TLP172AM photorelay switching lag compensation (see robomas_center notes):
	// driven at IF≈2mA (1kΩ/3.3V), below Toshiba's recommended min (5mA), so tON/tOFF
	// run slower than the datasheet's typ spec (0.6ms/0.1ms @5mA). Conservative estimate
	// at this drive current: tON≈1.5ms, tOFF≈0.1-0.2ms, net offset (tON-tOFF)≈1.4ms.
	// TIM2 runs at 200Hz over 256 steps (5ms period, ~19.5us/step), so 1.4ms is
	// ~72 steps. Keep this in sync with the TIM2 prescaler in MX_TIM2_Init():
	// the offset is a fixed *time*, so its tick count scales with the PWM period.
	constexpr uint16_t kOnOffsetTicks = 72;

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

	// Defaults to the green-at-50 % the tick used to hard-code, so behaviour is
	// unchanged until the host actually sends a colour.
	static volatile uint32_t requested_packed = ((uint32_t)128 << 8);

	void setRequestedState(State state){
		requested_packed = ((uint32_t)state.red << 16)
		                 | ((uint32_t)state.green << 8)
		                 |  (uint32_t)state.blue;
	}

	State getRequestedState(){
		uint32_t packed = requested_packed;
		State state;
		state.red   = (uint8_t)(packed >> 16);
		state.green = (uint8_t)(packed >> 8);
		state.blue  = (uint8_t)packed;
		return state;
	}
}
