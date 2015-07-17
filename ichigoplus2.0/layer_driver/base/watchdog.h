/*******************************************
 * watchdog ver1.0 2015/2/5
 * Using watchdog timer.
 *
 * [Dependency]
 * config_iwdg
 *
 * [Note]
 *
 * [Change history]
 * ver1.0 2015/ 3/ 3 The first version.
 ******************************************/
#ifdef STM32F407VG

#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
 extern "C" {
#endif

 int watchdog_setup();
 void watchdog_cycle();

#ifdef __cplusplus
}
#endif

#endif//WATCHDOG_H
#endif
