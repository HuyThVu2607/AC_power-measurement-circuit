#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define SCH_MAX_TASKS 20

#define NO_ERROR 0x00
#define ERROR_SCH_TOO_MANY_TASKS 0x01
#define ERROR_SCH_DELETE_NULL_TASK 0x02
#define ERROR_SCH_WAITING_FOR_SLAVE_ACK 0x04 // Corrected typo
#define ERROR_SCH_WAITING_FOR_START_COMMAND_FROM_MASTER 0x08
#define ERROR_SCH_ONE_OR_MORE_SLAVES_DID_NOT_START 0x10
#define ERROR_SCH_LOST_SLAVE 0x20
#define ERROR_SCH_CAN_BUS_ERROR 0x40
#define ERROR_I2C_WRITE_BYTE_AT24C64 0x80

#define TIMER_CYCLE 10

//MORE OPTIONS DEFINE HERE !
#define SCH_REPORT_ERROR
#define SCH_EXP

typedef struct sTask // Added typedef
{
    void (*pFunc)(void); // function pointer
    uint32_t Delay;      // Set time delay task
    uint32_t Period;     // Care
    uint16_t TaskID;     // Don't care
    struct sTask* Next;  // Pointer to the next task
} sTask; // Added typedef

extern uint16_t TaskIdJustRun;

void SCH_Init(void);
void SCH_Update(void);
sTask *SCH_Add_Task(void (*pFunc)(void), uint32_t Delay, uint32_t Period);
uint8_t SCH_Dispatch_Tasks(void);
uint8_t SCH_Delete_Task(sTask *delete_Task);
void SCH_sleep(void);
void SCH_Report_Status(void);

#endif // __ SCHEDULER_H_