#include "can.h"
#include "mti_can.h"
#include "vmt_uart.h"
#include "mti_system.h"
#include "mti_measure.h"
#include <math.h>
#include "mti_imu.h"

CAN_RxHeaderTypeDef rxHeader;                              // CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader;                              // CAN Bus Receive Header
uint8_t             canRX[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // CAN Bus Receive Buffer
CAN_FilterTypeDef   canfil;                                // CAN Bus Filter
uint32_t            canMailbox;                            // CAN Bus Mail box variable
float               detectedPoint[20][2];
float               maxSNR;
uint32_t            totalPacketLength;
uint32_t            frameNumber;
uint8_t             numDetPoints;
uint32_t            pointIndex;
float               radar_depth;

bool can_setup(void)
{
#ifdef PCB_CANBUS
#define FILTER_ID ((0x00000085 << 3) | 0x4)
// #define FILTER_MASK ((0x000000D2 << 3) | 0x4)
#define FILTER_MASK 0

    canfil.FilterBank           = 0;
    canfil.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh         = FILTER_ID >> 16;
    canfil.FilterIdLow          = FILTER_ID & 0xFFFF;
    canfil.FilterMaskIdHigh     = FILTER_MASK >> 16;
    canfil.FilterMaskIdLow      = FILTER_MASK & 0xFFFF;
    canfil.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation     = ENABLE;
    // canfil.SlaveStartFilterBank = 14;

    // 0x80 1000 0000
    // 0xA0 1010 0000
    // 0xA1 1010 0001
    // 0xA2 1010 0010
    // 0xA3 1010 0011
    // 0xA4 1010 0100

    HAL_CAN_ConfigFilter(&hcan1, &canfil); // Initialize CAN Filter
    HAL_CAN_Start(&hcan1);                 // Initialize CAN Bus
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
        return true; // Initialize CAN Bus Rx Interrupt
#endif
    return false;
}

bool can_send(uint32_t ID, uint8_t message)
{
#ifdef PCB_CANBUS
    if (ID == CAN_CMD)
    {
        if (message == CAN_START)
            syn_radar_set(RADAR_CHIRPING);
        else if (message == CAN_STOP)
            syn_radar_set(RADAR_STOPPED);
    }
    txHeader.DLC                = 1; // Number of bites to be transmitted max- 8
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;
    DEBUG_SEND("CAN TX (0x%02X) 0x%02X", ID, message);
    uint8_t csend[1];
    csend[0] = message;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) == HAL_OK)
        return true; // Send Message
#endif
    return false;
}

bool can_send_array(uint32_t ID, uint8_t *message, size_t length)
{
#ifdef PCB_CANBUS
    txHeader.DLC                = length; // Number of bites to be transmitted max- 8
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.StdId              = ID;
    txHeader.ExtId              = ID;
    txHeader.TransmitGlobalTime = DISABLE;

    char data_hex[17];
    memset(data_hex, 0, 17);
    for (int i = 0; i < length; i++)
    {
        sprintf(data_hex, "%s%02X", data_hex, message[i]);
    }
    DEBUG_SEND("CAN TX (0x%02X) 0x%s", ID, data_hex);
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, message, &canMailbox) == HAL_OK)
        return true; // Send Message
#endif
    return false;
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan1)
{
    DEBUG_SEND("CAN FIFO FULL");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) == HAL_OK)
    { // Receive CAN bus message to canRX buffer
// DEBUG_SEND("CAN RX: %02X",rxHeader.ExtId);
#ifdef SENSOR_RADAR
        if (rxHeader.ExtId == 0xA0)
        { // Header

            memcpy(&totalPacketLength, canRX, 4);
            memcpy(&frameNumber, canRX + 4, 4);
            numDetPoints = totalPacketLength - 129;
            if (true || numDetPoints)
            {
                // DEBUG_SEND("0xA0 %d",frameNumber);
                DEBUG_SEND("#frame,%d,%d,%.0f", frameNumber, numDetPoints, imu_get_angle());
            }
            pointIndex = 0;
            maxSNR     = 0;
            radar_profile_switch();
            // radar_measure(radar_depth);
            // DEBUG_SEND("Length: %d",totalPacketLength);
        }

        if (rxHeader.ExtId == 0xA1)
        { // Detected point
            memcpy(&detectedPoint[pointIndex], canRX, 8);
            DEBUG_SEND("0xA1: %f (%.0f)", detectedPoint[pointIndex][0], detectedPoint[pointIndex][1]);
            pointIndex++;
            if (pointIndex == numDetPoints)
                radar_measure(detectedPoint, numDetPoints);
        }

        //		else if(rxHeader.ExtId == 0xA2) {
        //			DEBUG_SEND("0xA2");
        //		}

        else if (rxHeader.ExtId == 0xA3 || rxHeader.ExtId == 0xA5)
        { // Status
            //			if(canRX[0] == RADAR_READY) {
            //				uint8_t message[2] = {CAN_PROFILE, 1}; //set profile to 100m
            //				can_send_array(CAN_CMD,message,2);
            //			}
            syn_radar_set(0);
            system_status_set(MODULE_RADAR, canRX[0]);
            char data_hex[17];
            memset(data_hex, 0, 17);
            for (int i = 0; i < rxHeader.DLC; i++)
            {
                sprintf(data_hex, "%s%02X", data_hex, canRX[i]);
            }
            DEBUG_SEND("\nCAN RX: (0x%02X) 0x%s", rxHeader.ExtId, data_hex);
        }
        else if (rxHeader.ExtId == 0xA4)
        { // Version
            version_set(MODULE_DOWNHOLE, canRX[0], canRX[1], canRX[2]);
        }
        else if (rxHeader.ExtId == CAN_CMD && canRX[0] == CAN_STATUS)
        { // For PCB testing
            can_send(CAN_ID_STATUS, RADAR_READY);
        }
        //		else if(rxHeader.ExtId == CAN_ID_STATUS) { //Sensor status
        //			system_status_set(MODULE_RADAR,canRX[0]);
        //		}
#endif
    }
    else
    {
        DEBUG_SEND("CAN RX ERROR");
    }
}
