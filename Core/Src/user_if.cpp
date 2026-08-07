/*
 * user_if.c
 *
 *  Created on: Dec 26, 2022
 *      Author: taman
 */

#include <MotorCtrl.hpp>
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
// neighbouring PA0-PA2 (TIM2 PWM / 1 kHz GPIO writes). Raw EXTI edges chatter
// under that noise, so the levels are sampled from the 1 kHz TIM3 tick below
// and only reported once they have been stable for debounce_ticks.
static void notify_user_inputs(void)
{
	static const uint16_t debounce_ticks = 20;   // 1 kHz tick -> 20 ms

	static GPIO_PinState ems_reported    = GPIO_PIN_RESET;
	static GPIO_PinState ems_candidate   = GPIO_PIN_RESET;
	static uint16_t      ems_count       = 0;
	static GPIO_PinState start_reported  = GPIO_PIN_RESET;
	static GPIO_PinState start_candidate = GPIO_PIN_RESET;
	static uint16_t      start_count     = 0;

	GPIO_PinState ems_now = HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin);
	if(ems_now != ems_candidate){
		ems_candidate = ems_now;
		ems_count = 0;
	}else if(ems_count < debounce_ticks){
		ems_count++;
		if((ems_count == debounce_ticks) && (ems_candidate != ems_reported)){
			ems_reported = ems_candidate;
			led_on(red);
			if(ems_reported == GPIO_PIN_SET){
				static uint8_t ems_high_encoded[] = {0x03, 0x02 << 4,'H', 0x00};
				CDC_Transmit_FS(ems_high_encoded, 2 + 2);
			}else{
				static uint8_t ems_low_encoded[] = {0x03, 0x02 << 4,'L', 0x00};
				CDC_Transmit_FS(ems_low_encoded, 2 + 2);
			}
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
			if((start_reported == GPIO_PIN_SET) && (ems_reported == GPIO_PIN_SET)){
				static uint8_t start_encoded[] = {0x03, 0x02 << 4,'S', 0x00};
				CDC_Transmit_FS(start_encoded, 2 + 2);
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
	//TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		notify_user_inputs();
		if(HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin)){
			HAL_GPIO_WritePin(Red12_GPIO_Port,Red12_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Green12_GPIO_Port,Green12_Pin,GPIO_PIN_SET);
			motor.transmit1();
			motor.transmit2();
		}else{
			HAL_GPIO_WritePin(Red12_GPIO_Port,Red12_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(Blue12_GPIO_Port,Blue12_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Green12_GPIO_Port,Green12_Pin,GPIO_PIN_RESET);
			motor.ems();
			for(uint8_t i = 0;i<8;i++){
				motor.reset(i);
			}
		}
	}
}
}
