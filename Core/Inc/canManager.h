/*
 * canManager.h
 *
 */

#ifndef INC_CANMANAGER_H_
#define INC_CANMANAGER_H_

#include "stm32g4xx_hal.h"

// tx_queue.h
#define SW_TX_QUEUE_SIZE 32  // Adjust as needed

typedef struct {
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CAN_TxMessage_t;

typedef struct {
    CAN_TxMessage_t buffer[SW_TX_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} CAN_TxQueue_t;

extern CAN_TxQueue_t canTxQueue;

bool CAN_Enqueue(FDCAN_TxHeaderTypeDef *header, uint8_t *data);

void CAN_ProcessTxQueue(FDCAN_HandleTypeDef* hcan);

#endif /* INC_CANMANAGER_H_ */
