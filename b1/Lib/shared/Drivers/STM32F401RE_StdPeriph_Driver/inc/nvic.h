 /* File name: nvic.h
 *
 * Description: Header file for NVIC
 *
 *
 * Last Changed By:  $Author: $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $April 15, 2022
 *
 * Code sample:
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NVIC_H
#define __NVIC_H

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f401re.h"

/******************************************************************************/
/*                     PRIVATE TYPES and DEFINITIONS                         */
/******************************************************************************/


/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
 /**
   * @brief  NVIC Init Structure definition
   */

 typedef struct
 {
   uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                    This parameter can be an enumerator of @ref IRQn_Type
                                                    enumeration (For the complete STM32 Devices IRQ Channels
                                                    list, please refer to stm32f4xx.h file) */

   uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                    specified in NVIC_IRQChannel. This parameter can be a value
                                                    between 0 and 15 as described in the table @ref NVIC_NVIC_Priority_Table
                                                    A lower priority value indicates a higher priority */

   uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                    in NVIC_IRQChannel. This parameter can be a value
                                                    between 0 and 15 as described in the table @ref NVIC_NVIC_Priority_Table
                                                    A lower priority value indicates a higher priority */

   FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                    will be enabled or disabled.
                                                    This parameter can be set either to ENABLE or DISABLE */
 } NVIC_InitTypeDef;


 /** @defgroup NVIC_Vector_Table_Base
   * @{
   */

 #define NVIC_VectTab_RAM             ((uint32_t)0x20000000)
 #define NVIC_VectTab_FLASH           ((uint32_t)0x08000000)
 #define IS_NVIC_VECTTAB(VECTTAB) (((VECTTAB) == NVIC_VectTab_RAM) || \
                                   ((VECTTAB) == NVIC_VectTab_FLASH))

 /** @defgroup NVIC_System_Low_Power
   * @{
   */
 #define NVIC_LP_SEVONPEND            ((uint8_t)0x10)
 #define NVIC_LP_SLEEPDEEP            ((uint8_t)0x04)
 #define NVIC_LP_SLEEPONEXIT          ((uint8_t)0x02)
 #define IS_NVIC_LP(LP) (((LP) == NVIC_LP_SEVONPEND) || \
                         ((LP) == NVIC_LP_SLEEPDEEP) || \
                         ((LP) == NVIC_LP_SLEEPONEXIT))

 /** @defgroup NVIC_Preemption_Priority_Group
   * @{
   */
 #define NVIC_PriorityGroup_0         ((uint32_t)0x700) /*!< 0 bits for pre-emption priority
                                                             4 bits for subpriority */
 #define NVIC_PriorityGroup_1         ((uint32_t)0x600) /*!< 1 bits for pre-emption priority
                                                             3 bits for subpriority */
 #define NVIC_PriorityGroup_2         ((uint32_t)0x500) /*!< 2 bits for pre-emption priority
                                                             2 bits for subpriority */
 #define NVIC_PriorityGroup_3         ((uint32_t)0x400) /*!< 3 bits for pre-emption priority
                                                             1 bits for subpriority */
 #define NVIC_PriorityGroup_4         ((uint32_t)0x300) /*!< 4 bits for pre-emption priority
                                                             0 bits for subpriority */

 #define IS_NVIC_PRIORITY_GROUP(GROUP) (((GROUP) == NVIC_PriorityGroup_0) || \
                                        ((GROUP) == NVIC_PriorityGroup_1) || \
                                        ((GROUP) == NVIC_PriorityGroup_2) || \
                                        ((GROUP) == NVIC_PriorityGroup_3) || \
                                        ((GROUP) == NVIC_PriorityGroup_4))

 #define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

 #define IS_NVIC_SUB_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

 #define IS_NVIC_OFFSET(OFFSET)  ((OFFSET) < 0x000FFFFF)

 /** @defgroup NVIC_SysTick_clock_source
   * @{
   */

 #define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
 #define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)
 #define IS_SYSTICK_CLK_SOURCE(SOURCE) (((SOURCE) == SysTick_CLKSource_HCLK) || \
                                        ((SOURCE) == SysTick_CLKSource_HCLK_Div8))

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
 void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
 void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
 void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
 void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
 void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);
/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __NVIC_H */
