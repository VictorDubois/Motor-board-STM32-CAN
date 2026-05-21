// tx_queue.c

#include "canManager.h"
#include <string.h> // memcpy

CAN_TxQueue_t canTxQueue = {0};

// Call this from your ISRs - just enqueues, never touches HAL
bool CAN_Enqueue(FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    if (canTxQueue.count >= SW_TX_QUEUE_SIZE)
        return false;  // Queue full!

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    canTxQueue.buffer[canTxQueue.tail].header = *header;
   memcpy(canTxQueue.buffer[canTxQueue.tail].data, data, 8);
    canTxQueue.tail = (canTxQueue.tail + 1) % SW_TX_QUEUE_SIZE;
    canTxQueue.count++;

    __set_PRIMASK(primask);
    return true;
}

// Call this from main loop ONLY - safe HAL access
void CAN_ProcessTxQueue(FDCAN_HandleTypeDef* hcan)
{
    while (canTxQueue.count > 0)
    {
        // Check HW FIFO not full (max 3 slots)
        if ((hcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U)
            continue;  // HW full, try later

        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        CAN_TxMessage_t msg = canTxQueue.buffer[canTxQueue.head];
        canTxQueue.head = (canTxQueue.head + 1) % SW_TX_QUEUE_SIZE;
        canTxQueue.count--;

        __set_PRIMASK(primask);

        // Only called from main loop = safe!
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &msg.header, msg.data);
    }
}
