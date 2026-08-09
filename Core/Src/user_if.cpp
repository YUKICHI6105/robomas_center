/*
 * user_if.c
 *
 *  Created on: Dec 26, 2022
 *      Author: taman
 */

#include <MotorCtrl.hpp>
#include <tape_led.hpp>
#include "main.h"
#include "can_usb.h"
#include "usbd_cdc_if.h"

#include "led.h"

extern"C"{
extern MotorCtrl motor;
extern CAN_TxHeaderTypeDef TxHeader1;
extern CAN_TxHeaderTypeDef TxHeader2;
extern uint32_t TxMailbox;
extern CAN_HandleTypeDef hcan;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // data structure
    /*
    uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
    uint8_t id[4] : can id
    uint8_t dlc : data length
    uint8_t data[8] : data (it is pre-writtten.)
    */
    uint8_t Data[14];
    CAN_RxHeaderTypeDef RxHeader;
    // the Data is used for USB buffer. can_process set header infomation to Data[0~5].
    // It is a terrible code. Sorry for hard work to read the code.
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Data + 6) == HAL_OK)
    {
    	if(motor.update(RxHeader.StdId,Data + 6)){
    		if(motor.diag == 1){can_process(&RxHeader, Data);}
    	}else{
    		can_process(&RxHeader, Data);
    	}
    }else{
		
	}
}

// EMS and START sit on an open contact in one switch position, so the pin is
// held only by the weak internal pull-down and picks up noise from the
// neighbouring PA0-PA2 (TIM2 PWM / 1 kHz GPIO writes). Sample them from the
// 1 kHz TIM3 tick rather than trusting raw edges.
//
// The EMS result is shared by the USB notification, the tape LED and the motor
// cut-off so a single noise sample cannot make them disagree. Debouncing is
// deliberately one-sided: releasing has to hold for release_debounce_ticks,
// but engaging is taken on the first low sample, so a real emergency stop is
// never delayed.
static bool ems_is_released(void)
{
	static const uint16_t release_debounce_ticks = 20;   // 1 kHz tick -> 20 ms

	static bool     released   = false;
	static uint16_t high_count = 0;

	if(HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin) == GPIO_PIN_SET){
		if(high_count < release_debounce_ticks){
			high_count++;
			if(high_count == release_debounce_ticks){
				released = true;
			}
		}
	}else{
		high_count = 0;
		released   = false;
	}
	return released;
}

static void notify_user_inputs(bool ems_released)
{
	static const uint16_t debounce_ticks = 20;   // 1 kHz tick -> 20 ms

	static bool          ems_reported    = false;
	static GPIO_PinState start_reported  = GPIO_PIN_RESET;
	static GPIO_PinState start_candidate = GPIO_PIN_RESET;
	static uint16_t      start_count     = 0;

	if(ems_released != ems_reported){
		ems_reported = ems_released;
		led_on(red);
		if(ems_reported){
			static uint8_t ems_high_encoded[] = {0x03, 0x02 << 4,'H', 0x00};
			CDC_Transmit_FS(ems_high_encoded, 2 + 2);
		}else{
			static uint8_t ems_low_encoded[] = {0x03, 0x02 << 4,'L', 0x00};
			CDC_Transmit_FS(ems_low_encoded, 2 + 2);
		}
	}

	GPIO_PinState start_now = HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
	if(start_now != start_candidate){
		start_candidate = start_now;
		start_count = 0;
	}else if(start_count < debounce_ticks){
		start_count++;
		if((start_count == debounce_ticks) && (start_candidate != start_reported)){
			start_reported = start_candidate;
			// notify on press (Low->High) only, and only while EMS is released
			if((start_reported == GPIO_PIN_SET) && ems_released){
				static uint8_t start_encoded[] = {0x03, 0x02 << 4,'S', 0x00};
				CDC_Transmit_FS(start_encoded, 2 + 2);
			}
		}
	}
}

// Tape LED policy, also driven from the 1 kHz tick. Blue12/Red12/Green12 are
// TIM2 PWM outputs now, so the plain HAL_GPIO_WritePin() calls that used to
// live in the tick did nothing - the compare registers have to be written
// instead. The colour is stepped every 20 ms, which sets the pulse rate below;
// the TIM2 carrier itself runs far faster (200 Hz) to keep the tape from
// visibly strobing, so each step simply holds for a few PWM periods.
static void update_tape_led(bool ems_released)
{
	static const uint16_t step_ticks = 20;   // 1 kHz tick -> 20 ms per sine step

	// 250 steps * 20 ms = 5 s per cycle (0.2 Hz), spanning 10 %..70 % of full
	// scale (26..178) rather than 0..255: with compensate() adding 72 the top
	// would otherwise run past ARR=255 and clip flat, and the very bottom sits
	// in the photorelay's non-linear region.
	//
	// The curve is a raised cosine raised to gamma 1.8 rather than the plain
	// cosine. Perceived brightness goes roughly as duty^(1/2.2), so a linear
	// duty sweep reads as "bright nearly all the time"; pre-distorting puts the
	// *perceived* brightness back near a sine. 1.8 rather than a full 2.2 keeps
	// a bit more time on the bright side: 1.9 s in the top half instead of 2.5 s
	// uncorrected (1.7 s at 2.2), with 2.2 s of dark dwell.
	static const uint8_t pulse[] = {
		 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
		 26,  26,  26,  26,  26,  26,  26,  27,  27,  27,
		 27,  27,  27,  28,  28,  28,  29,  29,  29,  30,
		 30,  31,  31,  32,  32,  33,  34,  34,  35,  36,
		 37,  38,  39,  40,  41,  42,  43,  44,  46,  47,
		 48,  50,  51,  53,  54,  56,  58,  59,  61,  63,
		 65,  67,  69,  71,  73,  75,  77,  79,  81,  83,
		 85,  88,  90,  92,  95,  97,  99, 102, 104, 106,
		109, 111, 113, 116, 118, 120, 123, 125, 127, 130,
		132, 134, 137, 139, 141, 143, 145, 147, 149, 151,
		153, 155, 157, 158, 160, 161, 163, 165, 166, 167,
		169, 170, 171, 172, 173, 174, 175, 175, 176, 176,
		177, 177, 178, 178, 178, 178, 178, 178, 178, 177,
		177, 176, 176, 175, 175, 174, 173, 172, 171, 170,
		169, 167, 166, 165, 163, 161, 160, 158, 157, 155,
		153, 151, 149, 147, 145, 143, 141, 139, 137, 134,
		132, 130, 127, 125, 123, 120, 118, 116, 113, 111,
		109, 106, 104, 102,  99,  97,  95,  92,  90,  88,
		 85,  83,  81,  79,  77,  75,  73,  71,  69,  67,
		 65,  63,  61,  59,  58,  56,  54,  53,  51,  50,
		 48,  47,  46,  44,  43,  42,  41,  40,  39,  38,
		 37,  36,  35,  34,  34,  33,  32,  32,  31,  31,
		 30,  30,  29,  29,  29,  28,  28,  28,  27,  27,
		 27,  27,  27,  27,  26,  26,  26,  26,  26,  26,
		 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
	};

	static uint16_t       tick         = 0;
	static uint16_t       phase        = 0;
	static bool           was_released = false;
	static tapeled::State applied      = {0, 0, 0};

	if(++tick < step_ticks){ return; }
	tick = 0;

	if(ems_released){
		// Coming out of a stop always hands back a known state: any colour the
		// host asked for is dropped and green is forced, so it has to re-send
		// once it sees the release. 50 % keeps CCR at 200 once compensate()
		// adds 72 - full scale would push it past ARR=255 and saturate.
		static const tapeled::State released_colour = {0, 128, 0};
		if(!was_released){
			tapeled::setRequestedState(released_colour);
		}
		// then follow the host again; only written on a change, so this is not
		// hammering the compare registers every step
		tapeled::State want = tapeled::getRequestedState();
		if(!was_released
		   || want.red   != applied.red
		   || want.green != applied.green
		   || want.blue  != applied.blue){
			tapeled::setLedState(want);
			applied = want;
		}
		phase = 0;
	}else{
		// forced red sine pulse for as long as the emergency stop is engaged
		tapeled::setLedState({pulse[phase], 0, 0});
		phase++;
		if(phase >= (sizeof(pulse) / sizeof(pulse[0]))){ phase = 0; }
	}
	was_released = ems_released;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
	//TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		bool ems_released = ems_is_released();
		notify_user_inputs(ems_released);
		update_tape_led(ems_released);
		if(ems_released){
			motor.transmit1();
			motor.transmit2();
		}else{
			motor.ems();
			for(uint8_t i = 0;i<8;i++){
				motor.reset(i);
			}
		}
	}
}
}
