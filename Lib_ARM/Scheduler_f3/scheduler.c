
#include "scheduler.h"
#include "stdlib.h"
#include "stm32f3xx.h"

sTask* head_task;

uint16_t time_skip;

uint16_t count_task;

uint16_t next_id;

uint16_t TaskIdJustRun = 0;

uint16_t error_code;

void SCH_Init(void)
{
    head_task = NULL;
    time_skip = 0;
    count_task = 0;
    next_id = 1;
    error_code = 0;
}

void SCH_Update(void)
{
    if(head_task ==  NULL)
    {
       time_skip = (count_task>0)?time_skip + 1 : 0;
       return;
    }
    
    time_skip++;
    if (head_task->Delay > 0)
    {
       int temp = head_task -> Delay - time_skip;
       if (temp>=0)
       {
            head_task->Delay = temp;
            time_skip = 0;
       }else
       {    
            head_task->Delay =0;
            time_skip = 0 - temp;
       }
    }
}

sTask* Create_Task(void (*pFunc)(void), uint32_t Delay, uint32_t Period )
{
    if (count_task > SCH_MAX_TASKS)
    {
        error_code = ERROR_SCH_TOO_MANY_TASKS;
        return NULL;
    }
    uint16_t newID = next_id++;

    Delay /= TIMER_CYCLE;
    Period /= TIMER_CYCLE;

    sTask* newTask = malloc(sizeof(sTask));

    newTask->pFunc      = pFunc;
    newTask->Delay      = Delay + time_skip;
    newTask->Period     = Period;
    newTask->TaskID     = newID;
    newTask->Next       = NULL;

    return newTask;
}

sTask* Enqueue_Task(sTask* newTask)
{
    if (head_task == NULL)
    {
        head_task=newTask;
        return newTask;
    }

    sTask *ini = head_task;
    sTask *pre = NULL;
    uint32_t sum = 0;
    while (ini != NULL)
    {
        if (sum + ini->Delay > newTask->Delay)
        {
            if (ini==head_task)
            {
                newTask->Next = head_task;
                head_task = newTask;
                newTask->Delay -= sum;
                ini->Delay     -= newTask->Delay;
            }
            return newTask;
        }
        sum     +=ini->Delay;
        pre      =ini;
        ini      =ini->Next;
    }

    if (ini == NULL)
    {
        pre->Next   = newTask;
        newTask->Delay -= sum;
    }

    return newTask;
}

sTask* SCH_Add_Task(void (*pFunc)(void), uint32_t Delay, uint32_t Period )
{
    sTask* newTask = Create_Task(pFunc,Delay,Period);
    if (newTask == NULL)
    {
        return NULL;
    }
    return Enqueue_Task(newTask);
}

uint8_t SCH_Dispatch_Tasks(void)
{
    if (head_task==NULL || head_task->Delay > 0 ) { return 0; }

    sTask* runningTask = head_task;
    head_task = head_task->Next;

    runningTask->Next   =NULL;
    runningTask->Delay  =runningTask->Period;

    //Run task
    runningTask->pFunc();

    TaskIdJustRun = runningTask->TaskID;

    if (runningTask->Period !=0) {Enqueue_Task(runningTask); }
    else{ free(runningTask); }
    
    return 1;
}

uint8_t SCH_Delete_Task(sTask* delete_task)
{
    if (delete_task == NULL)
    {
        error_code = ERROR_SCH_DELETE_NULL_TASK;
        return 0;
    }

    if (delete_task == head_task)
    {
        count_task--;
        head_task = head_task->Next;
        delete_task->Next->Delay += delete_task->Delay;
        free(delete_task);
        return 1;
    }

    sTask* pre = NULL;
    sTask* ini = head_task;

    while (ini != delete_task && ini != NULL)
    {
        pre = ini;
        ini = ini->Next;
    }

    if (ini == NULL)
    {
        error_code = ERROR_SCH_DELETE_NULL_TASK;
        return 0;
    }

    if (delete_task->Next != NULL)
    {
        delete_task->Next->Delay += delete_task->Delay;
    }

    if (pre != NULL)
    {
        pre->Next = delete_task->Next;
    }

    free(delete_task);
    return 1;
}

void SCH_sleep(void){
    if (head_task->Delay == 0 )  { return; }
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
}

void SCH_Report_Status(void)
{
    #ifdef  SCH_REPORT_ERRORS   
        uint8_t tempErrorCode = errorCode;
        errorCode = 0;
        //return tempErrorCode;
    #endif
}
