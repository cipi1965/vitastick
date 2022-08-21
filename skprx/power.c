#include "power.h"
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/suspend.h>

static int lock_power = 0;

static int power_tick_thread(SceSize args, void *argp) {
  while (1) {
    if (lock_power > 0) {
      ksceKernelPowerTick(SCE_KERNEL_POWER_TICK_DISABLE_OLED_OFF);
    }

    ksceKernelDelayThread(10 * 1000 * 1000);
  }

  return 0;
}

void initPowerTickThread() {
  SceUID thid = ksceKernelCreateThread("power_tick_thread", power_tick_thread, 0x10000100, 0x40000, 0, 0, NULL);
  if (thid >= 0)
    ksceKernelStartThread(thid, 0, NULL);
}

void powerLock() {
  lock_power++;
}

void powerUnlock() {
  lock_power--;
  if (lock_power < 0)
    lock_power = 0;
}