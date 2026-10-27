#pragma once
#include <main.h>
#include <led.h>

struct Data{
    uint8_t data[14] = {};
    bool sended = true;
};

class Buffer{
private:
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    Data buffer[4];
public:
    void setBuffer(uint8_t usb_msg[]);
    void resend(CAN_HandleTypeDef& hcan);
};

inline void Buffer::setBuffer(uint8_t usb_msg[]){
    for(uint8_t j = 0; j < 4; j++){
        if(buffer[j].sended == true){
            for(uint8_t i = 0; i < 14; i++){
                buffer[j].data[i] = usb_msg[i];
            }
            buffer[j].sended = false;
            break;
        }
    }
}

inline void Buffer::resend(CAN_HandleTypeDef& hcan){
    // USB and the motor timer also access CAN. Keep this bounded scan atomic.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for(uint8_t i = 0; i < 4; i++){
        if(buffer[i].sended == false){
            if(buffer[i].data[0] & 0x02){
                TxHeader.IDE = CAN_ID_EXT;
                TxHeader.ExtId = (buffer[i].data[1] << 24) | (buffer[i].data[2] << 16) |
                                 (buffer[i].data[3] << 8) | buffer[i].data[4];
            }else{
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.StdId = (buffer[i].data[1] << 24) | (buffer[i].data[2] << 16) |
                                 (buffer[i].data[3] << 8) | buffer[i].data[4];
            }
            TxHeader.RTR = (buffer[i].data[0] & 0x04) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
            TxHeader.TransmitGlobalTime = DISABLE;
            TxHeader.DLC = buffer[i].data[5];
            if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
                if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, buffer[i].data + 6, &TxMailbox) == HAL_OK){
                    led_on(can);
                    buffer[i].sended = true;
                }
            }
        }
    }
    __set_PRIMASK(primask);
}
