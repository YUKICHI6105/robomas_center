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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == EMS_Pin){
		led_on(red);
		if(HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin)){
	        static uint8_t HelloSLCAN_encoded[] = {0x03, 0x02 << 4,'H', 0x00};
	        CDC_Transmit_FS(HelloSLCAN_encoded, 2 + 2);
		}else{
	        static uint8_t HelloSLCAN_encoded[] = {0x03, 0x02 << 4,'L', 0x00};
	        CDC_Transmit_FS(HelloSLCAN_encoded, 2 + 2);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
	//TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
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
