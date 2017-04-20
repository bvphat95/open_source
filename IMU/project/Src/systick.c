#include "SysTick.h"

/******************************************************************************/
/* DEFINITION OF TYPES                                                        */
/******************************************************************************/

/******************************************************************************/
/* DECLARATION OF VARIABLES                                    */
/******************************************************************************/
static __IO uint32_t __systemTickTimmingDelay = 0;
static __IO uint32_t __systemTickCount  = 0;
static __IO uint32_t __systemTickTimeout  = 0;
/**
  * @brief  The function inits SysTick modules
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{ 
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */
    NVIC_SystemReset();
  }
  else
  {
    /* do nothing */
  }
}
/**
  * @brief  System Tick hardware interrupt event.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* Incre __systemTickCount by one */
  __systemTickCount++;

  /* used for delay */
  if (__systemTickTimmingDelay != 0)
  {
    /* Decre __systemTickTimmingDelay by one */
    __systemTickTimmingDelay--;
  }
  else
  {
    /*do nothing*/
  }
  /* used for Timeout */
  if (__systemTickTimeout != 0)
  {
    /* Decre __systemTickTimeout by one */
    __systemTickTimeout--;
  }
  else
  {
    /*do nothing*/
  }
}

/**
  * @brief  Delay in milisecond
  * @param  None
  * @retval None
  */
__IO void Delay(uint32_t nTime)
{
  __systemTickTimmingDelay = nTime;
  while(__systemTickTimmingDelay != 0);
}

/**
  * @brief  Get current value from SySTick
  * @param  None
  * @retval None
  */
uint32_t SysTick_GetTickCount(void)
{
  return __systemTickCount;
}

/**
  * @brief  Start one Timeout in milisecond
  * @param  None
  * @retval None
  */
void SysTick_StartTimeout(uint32_t nTime)
{
  __systemTickTimeout = nTime;
}

/**
  * @brief  Start one timeout in milisecond
  * @param  None
  * @retval None
  */
BOOLEAN SysTick_IsReachTimeout(void)
{
  return (__systemTickTimeout == 0);
}
