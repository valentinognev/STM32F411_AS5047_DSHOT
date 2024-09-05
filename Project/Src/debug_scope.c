
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "debug_scope.h" 
#include "main.h" 

int64_t TickMSCounter = 0;

int64_t getTickMSCounter()
{
    return TickMSCounter;
}

DebugWriteState DebugScopeInsertData(DebugScope_Handle_t *pHandle, const float data[DEBUGSCOPENUMOFCH])
{
    if (!pHandle->startWriteFlag)
        return START_FLAG_IS_OFF ;

    if (pHandle->id == DEBUGSCOPESIZE)
    {
        pHandle->startWriteFlag = false;
        return NO_MORE_PLACE_TO_WRITE; 
    }
	pHandle->timeus[pHandle->id] = TIM5->CNT;
#if DEBUGSCOPENUMOFCH > 0
	pHandle->Ch1[pHandle->id] = data[0];
#if DEBUGSCOPENUMOFCH > 1
	pHandle->Ch2[pHandle->id] = data[1];
#if DEBUGSCOPENUMOFCH > 2
	pHandle->Ch3[pHandle->id] = data[2];
#if DEBUGSCOPENUMOFCH > 3
	pHandle->Ch4[pHandle->id] = data[3];
#if DEBUGSCOPENUMOFCH > 4
	pHandle->Ch5[pHandle->id] = data[4];
#endif
#endif
#endif
#endif
#endif
	pHandle->id++;
	return NO_ERROR;
}

void DebugScopeStartWrite(DebugScope_Handle_t *pHandle)
{
    pHandle->startWriteFlag = true;
    pHandle->id = 0;
    LL_TIM_SetCounter(TIM5, 0);
    LL_TIM_EnableCounter(TIM5);
}
