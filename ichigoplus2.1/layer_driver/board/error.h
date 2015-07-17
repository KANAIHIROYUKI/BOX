#ifndef ERROR_H
#define ERROR_H
/*******************************************
 * error ver1.0 2015/3/3
 * Actions when error occurs.
 * At default, output error details via logging.
 *
 * [Dependency]
 * stdio
 * stm32f4xx
 * logging
 *
 * [Note]
 *
 * [Change history]
 * ver1.0 2015/ 3/ 3 The first version.
 ******************************************/
#ifdef __cplusplus
 extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
#ifdef __cplusplus
}
#endif

#endif//ERROR_H
