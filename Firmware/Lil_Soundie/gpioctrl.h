#ifndef __GPIOCTRL_H__
#define __GPIOCTRL_H__

#ifdef ASM

#else /*ASM*/

#include <vstypes.h>

void GPIOCtrlIdleHook(void);
void GPIOInit(void);

#endif /* elseASM*/

#endif /* !__GPIOCTRL_H__ */
