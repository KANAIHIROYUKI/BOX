#ifndef SYSTEM_H
#define SYSTEM_H

#define PRINT_WHOAMI INFO("filename,%s",__FILE__);INFO("Compiled at,%s,%s",__DATE__,__TIME__);whoami();

#ifdef __cplusplus
 extern "C" {
#endif
int system_setup();
void whoami();
#ifdef __cplusplus
}
#endif
#endif//SYSTEM_H
