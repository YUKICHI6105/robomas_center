/*
 *  MoterCtrl.cpp
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */

#include <math.h>
#include <MotorCtrl.hpp>

#define control_period 0.001

void MotorCtrl::setAng(uint16_t data, uint32_t receiveID){
	param[receiveID-0x201].mechanical_angle = 360.0*data/8191;
}

void MotorCtrl::setVel(uint16_t data, uint32_t receiveID){
	if(data < 0x8000){
		param[receiveID-0x201].velocity = data*2*3.141592/60.0;
	}else{
		// NOTE: two's-complement negation should be ~data + 1; missing +1 here
		// biases the decoded magnitude low by 1 LSB. Left as-is (unverified on
		// real hardware) - see MotorCtrl review notes.
		data =~ data;
		param[receiveID-0x201].velocity = -1*data*2*3.141592/60.0;
	}
}

void MotorCtrl::setCurC610(uint16_t data, uint32_t receiveID){
	if((data & 0x8000) == 0x8000){
		// NOTE: two's-complement negation should be ~data + 1; missing +1 here
		// biases the decoded magnitude low by 1 LSB. Left as-is (unverified on
		// real hardware) - see MotorCtrl review notes.
		data =~ data;
		param[receiveID-0x201].current = -10*data/10000;
	}else{
		param[receiveID-0x201].current = 10*data/10000;
	}
}

void MotorCtrl::setCurC620(uint16_t data, uint32_t receiveID){
	if((data & 0x8000) == 0x8000){
		// NOTE: two's-complement negation should be ~data + 1; missing +1 here
		// biases the decoded magnitude low by 1 LSB. Left as-is (unverified on
		// real hardware) - see MotorCtrl review notes.
		data =~ data;
		param[receiveID-0x201].current = -20*data/16384;
	}else{
		param[receiveID-0x201].current = 20*data/16384;
	}
}

void MotorCtrl::setRevolution(uint8_t& number){
	//param.revolution_vel[number] = param.revolution_vel[number] + param.velocity[number]*360.0*0.001/(2*3.141592);
	if((param[number].mechanical_angle - param[number].pre_mechanical_angle) > 180){
		param[number].rotation--;
	}else if((param[number].mechanical_angle - param[number].pre_mechanical_angle) < -180){
		param[number].rotation++;
	}
	param[number].revolution = param[number].rotation*360.0 + param[number].mechanical_angle - param[number].init_mechanical_angle;
	param[number].pre_mechanical_angle = param[number].mechanical_angle;
}

void PIDParam::velPID(float target, float& goal, float& velocity, Motor& motorKinds){
	float e = target - velocity;
	float de = (e-vel_e_pre)/control_period;
	float du = velKp*de + velKi*e;
	goal = goal + (du + vel_du_pre)/2*control_period;
	vel_du_pre = du;
	if(motorKinds == Motor::C610){
		if(goal > 10){
			goal =10;
		}else if(goal < -10){
			goal = -10;
		}
	}else{
		if(goal > 20){
			goal = 20;
		}else if(goal < -20){
			goal = -20;
		}
	}
	vel_e_pre = e;
}

float PIDParam::posPID(float target, float revolution){
	return posKp*(target - revolution);
}

void PIDParam::stableposPID(float target, float& goal, float& velocity, float& stableLimitVel, float revolution, Motor& motorKinds){
	float value = posPID(target, revolution);
	if(stableLimitVel < 0){
		stableLimitVel = -1*stableLimitVel;
	}
	if(value > stableLimitVel){
		value = stableLimitVel;
	}else if(value < -1*stableLimitVel){
		value = -1*stableLimitVel;
	}
	velPID(value, goal, velocity, motorKinds);
}

void MotorCtrl::tyokuReset(uint8_t i){
	value1[i]=0;
	value2[i]=0;
	param[i].goal=0;
	pidParam[i].vel_e_pre=0;
	pidParam[i].vel_du_pre=0;
	param[i].revolution = param[i].revolution-param[i].tyoku_pos_target;
	param[i].rotation = param[i].revolution/360;
	if(param[i].revolution < 0){
		param[i].rotation--;
	}
	param[i].target=0;
}

bool MotorCtrl::update(uint32_t ReceiveID,uint8_t receiveData[8]){
	if(ReceiveID<0x201||ReceiveID>0x208){return false;}
	uint8_t number = ReceiveID - 0x201;
	setAng(((static_cast<uint16_t>(receiveData[0]) << 8) | receiveData[1]), ReceiveID);
	setVel(((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]), ReceiveID);
	if(param[number].motorKinds == Motor::C610){
		setCurC610(((static_cast<uint16_t>(receiveData[4]) << 8) | receiveData[5]), ReceiveID);
	}else{
		setCurC620(((static_cast<uint16_t>(receiveData[4]) << 8) | receiveData[5]), ReceiveID);
		param[number].temp = receiveData[6];
	}
	//vel = ((static_cast<uint16_t>(receiveData[2]) << 8) | receiveData[3]);
	switch(param[number].mode){
	case Mode::dis:
		reset(number);
		return true;
	case Mode::vel:
		pidParam[number].velPID(param[number].target, param[number].goal, param[number].velocity, param[number].motorKinds);
		return true;
	case Mode::pos:
		setRevolution(number);
		pidParam[number].velPID(pidParam[number].posPID(param[number].target, param[number].revolution), param[number].goal, param[number].velocity, param[number].motorKinds);
		return true;
	case Mode::berutyoku:
		setRevolution(number);
		if(param[number].target == 1){
			if(param[number].revolution < param[number].tyoku_pos_target/2){//vel_ctrl
				pidParam[number].velPID(param[number].tyoku_vel_target, param[number].goal, param[number].velocity, param[number].motorKinds);
			}else{//pos_ctrl
				pidParam[number].velPID(pidParam[number].posPID(param[number].tyoku_pos_target, param[number].revolution), param[number].goal, param[number].velocity, param[number].motorKinds);
				if((param[number].velocity == 0.0) && (param[number].revolution > (param[number].tyoku_pos_target-10)) && (param[number].revolution < (param[number].tyoku_pos_target+10))){
					tyokuReset(number);
				}
			}
		}
		return true;
	case Mode::stablepos:
		setRevolution(number);
		pidParam[number].stableposPID(param[number].target, param[number].goal, param[number].velocity, param[number].stableLimitVel, param[number].revolution, param[number].motorKinds);
		return true;
	default:
		return true;
	}
}

void MotorCtrl::reset(uint8_t i){
		value1[i]=0;
		value2[i]=0;
		param[i].target=0;
		param[i].mode=Mode::dis;
		param[i].goal=0;
		pidParam[i].vel_e_pre=0;
		pidParam[i].vel_du_pre=0;
		param[i].revolution=0;
		param[i].rotation=0;
		param[i].pre_mechanical_angle=0;
}

void MotorCtrl::setFrame(uint8_t usb_msg[], const uint8_t len){
//	if((len < 31) && ((usb_msg[1] & 0x7f)==3)){return;}
	uint8_t number = (usb_msg[0] & 0x07);
	if((usb_msg[1] & 0x80) == 0){
		param[number].motorKinds = Motor::C610;
	}else{
		param[number].motorKinds = Motor::C620;
		param[number].limitTemp = usb_msg[2];
	}
	switch(usb_msg[1] & 0x7f){
	case 0:
		reset(number);
		param[number].mode = Mode::dis;
		if(usb_msg[3] == 1){
			diag=1;
		}else if(usb_msg[3] == 0){
			diag=0;
		}
		break;
	case 1:
		reset(number);
		param[number].mode = Mode::vel;
		std::memcpy(&pidParam[number].velKp,usb_msg + 3,sizeof(float));
		std::memcpy(&pidParam[number].velKi,usb_msg + 7,sizeof(float));
		break;
	case 2:
		reset(number);
		param[number].mode = Mode::pos;
		std::memcpy(&pidParam[number].velKp,usb_msg + 3,sizeof(float));
		std::memcpy(&pidParam[number].velKi,usb_msg + 7,sizeof(float));
		std::memcpy(&pidParam[number].posKp,usb_msg + 11,sizeof(float));
		//param.revolution[number] = param.mechanical_angle[number];
		param[number].pre_mechanical_angle = param[number].mechanical_angle;
		param[number].init_mechanical_angle = param[number].mechanical_angle;
		break;
	case 3:
		if(param[number].mode != Mode::berutyoku){
			reset(number);
			param[number].mode = Mode::berutyoku;
			param[number].pre_mechanical_angle = param[number].mechanical_angle;
			param[number].init_mechanical_angle = param[number].mechanical_angle;
		}
		std::memcpy(&pidParam[number].velKp,usb_msg + 3,sizeof(float));
		std::memcpy(&pidParam[number].velKi,usb_msg + 7,sizeof(float));
		std::memcpy(&pidParam[number].posKp,usb_msg + 11,sizeof(float));
		std::memcpy(&param[number].tyoku_vel_target,usb_msg + 15,sizeof(float));
		std::memcpy(&param[number].tyoku_pos_target,usb_msg + 19,sizeof(float));
		break;
	case 4:
		reset(number);
		// param[number].revolution = param[number].mechanical_angle;
		param[number].pre_mechanical_angle = param[number].mechanical_angle;
		param[number].init_mechanical_angle = param[number].mechanical_angle;
		param[number].mode = Mode::stablepos;
		std::memcpy(&pidParam[number].velKp,usb_msg + 3,sizeof(float));
		std::memcpy(&pidParam[number].velKi,usb_msg + 7,sizeof(float));
		std::memcpy(&pidParam[number].posKp,usb_msg + 11,sizeof(float));
		std::memcpy(&param[number].stableLimitVel,usb_msg + 15,sizeof(float));
		break;
	default:
		break;
	}
//	if((usb_msg[1] & 0x7f)==0){
//		reset(number);
//		param.mode[number] = Mode::dis;
//		if(usb_msg[3] == 1){
//			diag=1;
//		}else if(usb_msg[3] == 0){
//			diag=0;
//		}
//	}else if((usb_msg[1] & 0x7f)==1){
//		reset(number);
//		param.mode[number] = Mode::vel;
//		std::memcpy(&velPIDParam.Kp[number],usb_msg + 3,sizeof(float));
//		std::memcpy(&velPIDParam.Ki[number],usb_msg + 7,sizeof(float));
//		std::memcpy(&velPIDParam.Kd[number],usb_msg + 11,sizeof(float));
//		std::memcpy(&velPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
//	}else if((usb_msg[1] & 0x7f)==2){
//		reset(number);
//		param.mode[number] = Mode::pos;
//		std::memcpy(&posPIDParam.Kp[number],usb_msg + 3,sizeof(float));
//		std::memcpy(&posPIDParam.Ki[number],usb_msg + 7,sizeof(float));
//		std::memcpy(&posPIDParam.Kd[number],usb_msg + 11,sizeof(float));
//		std::memcpy(&posPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
//		//param.revolution[number] = param.mechanical_angle[number];
//		param.pre_mechanical_angle[number] = param.mechanical_angle[number];
//		param.init_mechanical_angle[number] = param.mechanical_angle[number];
//	}else if((usb_msg[1] & 0x7f)==3){
//		if(param.mode[number] != Mode::berutyoku){
//			reset(number);
//			param.mode[number] = Mode::berutyoku;
//			param.pre_mechanical_angle[number] = param.mechanical_angle[number];
//			param.init_mechanical_angle[number] = param.mechanical_angle[number];
//		}
//		std::memcpy(&velPIDParam.Kp[number],usb_msg + 3,sizeof(float));
//		std::memcpy(&velPIDParam.Ki[number],usb_msg + 7,sizeof(float));
//		std::memcpy(&velPIDParam.Kd[number],usb_msg + 11,sizeof(float));
//		std::memcpy(&velPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
//		std::memcpy(&posPIDParam.Kp[number],usb_msg + 19,sizeof(float));
//		std::memcpy(&posPIDParam.Ki[number],usb_msg + 23,sizeof(float));
//		std::memcpy(&posPIDParam.Kd[number],usb_msg + 27,sizeof(float));
//		std::memcpy(&posPIDParam.limitIe[number],usb_msg + 31,sizeof(float));
//		std::memcpy(&param.tyoku_vel_target[number],usb_msg + 35,sizeof(float));
//		std::memcpy(&param.tyoku_pos_target[number],usb_msg + 39,sizeof(float));
//	}else if((usb_msg[1] & 0x7f)==4){
//		reset(number);
//		param.revolution[number] = param.mechanical_angle[number];
//		param.mode[number] = Mode::stablepos;
//		std::memcpy(&velPIDParam.Kp[number],usb_msg + 3,sizeof(float));
//		std::memcpy(&velPIDParam.Ki[number],usb_msg + 7,sizeof(float));
//		std::memcpy(&velPIDParam.Kd[number],usb_msg + 11,sizeof(float));
//		std::memcpy(&velPIDParam.limitIe[number],usb_msg + 15,sizeof(float));
//		std::memcpy(&posPIDParam.Kp[number],usb_msg + 19,sizeof(float));
//		std::memcpy(&posPIDParam.Ki[number],usb_msg + 23,sizeof(float));
//		std::memcpy(&posPIDParam.Kd[number],usb_msg + 27,sizeof(float));
//		std::memcpy(&posPIDParam.limitIe[number],usb_msg + 31,sizeof(float));
//	}
}

void MotorCtrl::setTarget(uint8_t usb_msg[]){
	uint8_t number = (usb_msg[0]&0x07);
	std::memcpy(&param[number].target,usb_msg + 1,sizeof(float));
	if(param[number].mode == Mode::vel){
		if(param[number].motorKinds == Motor::C610){
			if(param[number].target < -1885){
				param[number].target = -1885;
			}else if(param[number].target > 1885){
				param[number].target = 1885;
			}
		}else if(param[number].motorKinds == Motor::C620){
			if(param[number].target < -943){
				param[number].target = -943;
			}else if(param[number].target > 943){
				param[number].target = 943;
			}
		}
	}
}

/*
void MotorCtrl::setMode(uint8_t usb_msg[]){
	for(int i = 0;i<8;i++){
		if(usb_msg[i+1]==0){
			param.mode[i] = Mode::dis;
			reset(i);
		}else if(usb_msg[i+1]==1){
			param.mode[i] = Mode::vel;
			reset(i);
		}else if(usb_msg[i+1]==2){
			param.mode[i] = Mode::pos;
			param.revolution[i] = param.mechanical_angle[i];
			param.pre_mechanical_angle[i] = param.mechanical_angle[i];
			reset(i);
		}else if(usb_msg[i+1]==3){
			param.mode[i] = Mode::hom;
			reset(i);
		}
	}
	if(usb_msg[8] == 1){
		diag=1;
	}else if(usb_msg[8] == 1){
		diag=0;
	}
}

void MotorCtrl::setLimitTemp(uint8_t usb_msg[]){
	for(int i = 0;i<8;i++){
		param.limitTemp[i]=usb_msg[i];
	}
}

void MotorCtrl::setTarget(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.target[i],&buf,1);
		if(param.mode == Mode::vel){
			if(param.target[i] < -932){
				param.target[i] = -932;
			}else if(param.target[i] > 932){
				param.target[i] = 932;
			}
		}
	}
}

void MotorCtrl::setKp(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.Kp[i],&buf,1);
	}
}

void MotorCtrl::setKi(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.Ki[i],&buf,1);
	}
}

void MotorCtrl::setKd(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.Kp[i],&buf,1);
	}
}

void MotorCtrl::setLimitIe(uint8_t usb_msg[]){
	for(int i =0;i<8;i++){
		uint32_t buf = (usb_msg[4*i+1] << 24) | (usb_msg[4*i+2] << 16) | (usb_msg[4*i+3] << 8) | (usb_msg[4*i+4] << 0);
		std::memcpy(&param.limitIe[i],&buf,1);
	}
}
*/
uint16_t getC620Value(float target){
	uint16_t value;
	if(target < 0.0){
		target = -target;
		if(target < 20.0){
			value = target/20*16384;
			// NOTE: two's-complement negation should be ~value + 1; missing +1
			// here biases the encoded magnitude low by 1 LSB. Left as-is
			// (unverified on real hardware) - see MotorCtrl review notes.
			value =~ value;
		}else{
			value = 16384;
			value =~ value;
		}
	}else{
		if(target < 20.0){
			value = target/20*16384;
		}else{
			value = 16384;
		}
	}
	return value;
}

uint16_t getC610Value(float target){
	uint16_t value;
	if(target < 0.0){
		target = -target;
		if(target < 10.0){
			value = target/10*10000;
			// NOTE: two's-complement negation should be ~value + 1; missing +1
			// here biases the encoded magnitude low by 1 LSB. Left as-is
			// (unverified on real hardware) - see MotorCtrl review notes.
			value =~ value;
		}else{
			value = 10000;
			value =~ value;
		}
	}else{
		if(target < 10.0){
			value = target/10*10000;
		}else{
			value = 10000;
		}
	}
	return value;
}

void MotorCtrl::transmit1(){
	for(int i=0;i<4;i++){
		if(param[i].temp < param[i].limitTemp){
			if(param[i].motorKinds == Motor::C620){
				value1[2*i] = static_cast<uint8_t>(getC620Value(param[i].goal) >> 8);
				value1[2*i+1] = static_cast<uint8_t>(getC620Value(param[i].goal) & 0xFF);
			}else{
				value1[2*i] = static_cast<uint8_t>(getC610Value(param[i].goal) >> 8);
				value1[2*i+1] = static_cast<uint8_t>(getC610Value(param[i].goal) & 0xFF);
			}
		}else{
			value1[2*i]=0;
			value1[2*i+1]=0;
		}
	}
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
	{
	    led_on(can);
	    HAL_CAN_AddTxMessage(&hcan, &TxHeader1, value1, &TxMailbox);
	}
}

void MotorCtrl::transmit2(){
	for(int i=4;i<8;i++){
		if(param[i].temp < param[i].limitTemp){
			if(param[i].motorKinds == Motor::C620){
				value2[2*(i-4)] = static_cast<uint8_t>(getC620Value(param[i].goal) >> 8);
				value2[2*(i-4)+1] = static_cast<uint8_t>(getC620Value(param[i].goal) & 0xFF);
			}else{
				value2[2*(i-4)] = static_cast<uint8_t>(getC610Value(param[i].goal) >> 8);
				value2[2*(i-4)+1] = static_cast<uint8_t>(getC610Value(param[i].goal) & 0xFF);
			}
		}else{
			value2[2*(i-4)]=0;
			value2[2*(i-4)+1]=0;
		}
	}
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
	{
	    led_on(can);
	    HAL_CAN_AddTxMessage(&hcan, &TxHeader2, value2, &TxMailbox);
	}
}

void MotorCtrl::ems(){
	uint8_t value3[8] = {0,0,0,0,0,0,0,0};
	led_on(can);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader1, value3, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader2, value3, &TxMailbox);
}
