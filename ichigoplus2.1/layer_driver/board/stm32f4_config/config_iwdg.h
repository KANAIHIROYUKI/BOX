#ifdef STM32F407VG
#ifndef CONFIG_IWDG_H
#define CONFIG_IWDG_H

#ifdef __cplusplus
 extern "C" {
#endif

int Init_watchdog();
void watchdog_reset();


#ifdef __cplusplus
}
#endif


#endif
#endif//STM32F407VG
